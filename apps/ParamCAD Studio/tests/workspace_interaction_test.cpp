#include "app/MainWindow.h"
#include "app/CadReconstructionTask.h"
#include "app/CaptureController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/ModelingController.h"
#include "app/ReconstructionController.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <STEPControl_Writer.hxx>

#include <QAction>
#include <QAbstractButton>
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QTreeWidget>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

QAction* action(MainWindow& window, const QString& text)
{
    for (auto* candidate : window.findChildren<QAction*>()) {
        if (candidate->text() == text) return candidate;
    }
    throw std::runtime_error("missing window action");
}

void chooseFile(MainWindow& window, QAction* command, const QString& path)
{
    // Exercise the real action/dialog path without displaying modal UI.
    bool dialogFound = false;
    QTimer::singleShot(0, &window, [&window, path, &dialogFound] {
        auto* dialog = window.findChild<QFileDialog*>();
        if (!dialog) return;
        dialogFound = true;
        dialog->setAttribute(Qt::WA_DontShowOnScreen);
        dialog->selectFile(path);
        QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
    });
    command->trigger();
    require(dialogFound, "file dialog was parented to window");
}

bool sameVector(const double first[3], const double second[3])
{
    for (int i = 0; i < 3; ++i) {
        if (std::abs(first[i] - second[i]) > 1.0e-6) return false;
    }
    return true;
}

void waitForCad(MainWindow& window, bool includeAnalysis = false)
{
    auto* task = window.findChild<CadReconstructionTask*>();
    auto* reconstruct = action(window, "Reconstruct CAD");
    QElapsedTimer elapsed;
    elapsed.start();
    while ((task->isRunning() || (includeAnalysis && !reconstruct->isEnabled()))
           && elapsed.elapsed() < 30000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    require(!task->isRunning() && (!includeAnalysis || reconstruct->isEnabled()), "CAD/analysis completed within deadline");
}

void testCadReconstruction(MainWindow& window, const QString& cloudPath)
{
    auto* viewer = window.findChild<VTKViewer*>();
    auto* model = window.findChild<ModelingController*>();
    auto* workspace = window.findChild<PointCloudWorkspaceController*>();
    auto* task = window.findChild<CadReconstructionTask*>();
    auto* reconstruct = action(window, "Reconstruct CAD");
    auto* cancel = action(window, "Cancel CAD Reconstruction");
    auto* cancelButton = window.findChild<QPushButton*>("cancelCadReconstructionButton");
    auto* status = window.findChild<QLabel*>("pointCloudRelationStatus");
    QString error;
    require(viewer->loadPointCloud(cloudPath, error), "load metric CAD fixture");
    const auto initialRevision = model->revision();
    int starts = 0;
    bool commitsOnOwner = true;
    const auto started = QObject::connect(task, &CadReconstructionTask::runningChanged, &window,
        [&](bool running) { starts += running; });
    const auto rebuilt = QObject::connect(model, &ModelingController::modelRebuilt, &window,
        [&] { commitsOnOwner &= QThread::currentThread() == window.thread(); });
    reconstruct->trigger();
    require(task->isRunning() && !reconstruct->isEnabled() && cancel->isEnabled()
            && !cancelButton->isHidden(), "running reconstruction exposes cancel and prevents duplicate commands");
    workspace->reconstructCad();
    require(starts == 1, "direct duplicate intent does not launch another worker");
    waitForCad(window, true);
    require(model->revision() > initialRevision && viewer->hasBody() && commitsOnOwner,
            "background candidate commits to model on GUI thread");
    require(!cancel->isEnabled() && cancelButton->isHidden(), "completion restores controls");

    const auto completedRevision = model->revision();
    reconstruct->trigger();
    cancelButton->click();
    waitForCad(window);
    require(model->revision() == completedRevision && status->text().contains("已取消"),
            "cancel button discards pending reconstruction without modifying model");

    reconstruct->trigger();
    modeling::FeatureId sketchId = modeling::kInvalidFeatureId;
    require(model->addSketch({}, sketchId), "edit model while CAD worker owns snapshot");
    const auto editedRevision = model->revision();
    waitForCad(window);
    require(model->revision() == editedRevision && status->text().contains("未应用"),
            "model revision prevents old candidate from overwriting new commands");

    reconstruct->trigger();
    viewer->selectPointCloudPoint(0);
    viewer->deleteSelectedPointCloudPoints();
    waitForCad(window);
    require(model->revision() == editedRevision && viewer->pointCloudPointCount() == 599
            && status->text().contains("未应用"), "point deletion invalidates candidate and preserves working cloud");

    require(viewer->loadPointCloud(cloudPath, error), "restore metric cloud");
    reconstruct->trigger();
    require(viewer->loadPointCloud(cloudPath, error), "replace cloud with identical file while worker is pending");
    waitForCad(window);
    require(model->revision() == editedRevision && status->text().contains("未应用"),
            "cloud revision invalidates same-file replacement");

    reconstruct->trigger();
    window.findChild<ReconstructionController*>()->finished(cloudPath, 600, "fixed", false);
    waitForCad(window, true);
    require(model->revision() == editedRevision && viewer->pointCloudSelection().regions().size() == 6,
            "new photo reconstruction defers automatic analysis until the old CAD job drains");

    reconstruct->trigger();
    viewer->setBodyShape(BRepPrimAPI_MakeBox(2, 3, 4).Shape());
    const auto importedRevision = viewer->bodyRevision();
    waitForCad(window);
    require(model->revision() == editedRevision && viewer->bodyRevision() == importedRevision
            && status->text().contains("未应用"), "imported display model is protected independently of feature history");

    reconstruct->trigger();
    action(window, "Close Model")->trigger();
    waitForCad(window);
    require(model->revision() == editedRevision && status->text().contains("已取消"),
            "closing model cancels pending CAD replacement");

    reconstruct->trigger();
    viewer->clearPointCloud();
    waitForCad(window);
    require(!reconstruct->isEnabled() && !cancel->isEnabled() && model->revision() == editedRevision,
            "clearing cloud prevents model commit and leaves reconstruction disabled");

    // Confirming unknown scale uses a nested event loop. A document replacement
    // inside that dialog must not pair old observations with the new revision.
    require(viewer->loadPointCloud(cloudPath, error), "load scale-dialog fixture");
    QFile metadata(QFileInfo(cloudPath).dir().filePath(QFileInfo(cloudPath).completeBaseName() + ".json"));
    require(metadata.open(QIODevice::WriteOnly | QIODevice::Truncate), "remove metric scale for modal test");
    metadata.write("{\"unit\":\"arbitrary\"}"); metadata.close();
    bool declined = false;
    QTimer::singleShot(0, &window, [&] {
        auto* box = window.findChild<QMessageBox*>();
        if (!box) return;
        box->setAttribute(Qt::WA_DontShowOnScreen);
        declined = true;
        box->button(QMessageBox::No)->click();
    });
    reconstruct->trigger();
    require(declined && !task->isRunning() && reconstruct->isEnabled() && model->revision() == editedRevision,
            "declining unknown scale starts no work and restores controls");
    bool confirmed = false;
    bool replaced = false;
    QTimer::singleShot(0, &window, [&] {
        auto* box = window.findChild<QMessageBox*>();
        if (!box) return;
        box->setAttribute(Qt::WA_DontShowOnScreen);
        replaced = viewer->loadPointCloud(cloudPath, error);
        workspace->reconstructCad();
        confirmed = true;
        box->button(QMessageBox::Yes)->click();
    });
    reconstruct->trigger();
    require(confirmed && replaced && !task->isRunning() && reconstruct->isEnabled()
            && status->text().contains("请重新启动"), "nested scale dialog cannot launch stale or duplicate snapshot");
    require(metadata.open(QIODevice::WriteOnly | QIODevice::Truncate), "restore metric metadata");
    metadata.write("{\"unit\":\"mm\"}"); metadata.close();
    QObject::disconnect(started);
    QObject::disconnect(rebuilt);

    auto closing = std::make_unique<MainWindow>();
    require(closing->findChild<VTKViewer*>()->loadPointCloud(cloudPath, error), "load CAD shutdown fixture");
    action(*closing, "Reconstruct CAD")->trigger();
    require(closing->findChild<CadReconstructionTask*>()->isRunning(), "worker running when window is destroyed");
    closing.reset();
    QCoreApplication::processEvents();
}
}

int main(int argc, char** argv)
{
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication application(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary workspace");
        MainWindow window;
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        application.processEvents();

        auto* viewer = window.findChild<VTKViewer*>();
        auto* model = window.findChild<ModelingController*>();
        auto* sketch = window.findChild<SketchPanel*>();
        auto* palette = window.findChild<SketchToolPalette*>();
        auto* properties = window.findChild<PropertyPanel*>();
        auto* console = window.findChild<QPlainTextEdit*>("console");
        require(viewer && model && sketch && palette && properties && console, "workspace views and command controller");

        auto* newSketch = action(window, "New Sketch");
        require(!newSketch->isEnabled() && model->features().empty(), "empty scene requires a plane before sketching");
        const auto cameraBefore = viewer->cameraParameters();
        viewer->datumPlanePicked(modeling::DatumPlane::XY);
        require(newSketch->isEnabled(), "plane pick enables sketch action");
        newSketch->trigger();
        require(sketch->hasSketch() && !palette->isHidden() && !newSketch->isEnabled()
                && viewer->cameraParameters().parallelProjection, "sketch entry shows workspace and aligns camera");
        const auto sketchId = model->features().back().id;
        palette->toolChanged(ViewportTool::Rectangle);
        viewer->sketchPointRequested({-5, -5});
        require(model->sketchParams(sketchId)->entities.empty(), "first rectangle click remains a draft");
        viewer->sketchPointRequested({5, 5});
        require(model->sketchParams(sketchId)->entities.size() == 1, "second click commits one rectangle to core");
        const auto rectangleId = model->sketchParams(sketchId)->entities.front().id;
        viewer->sketchEntityPicked(rectangleId);
        properties->findChild<QDoubleSpinBox*>("sketchRectWidth")->setValue(11);
        require(std::get<modeling::Rectangle2D>(model->sketchParams(sketchId)->entities.front().geometry).width == 11,
                "property edit updates selected entity through core");
        sketch->extrudeRequested(4, false, modeling::ExtrudeOperation::NewBody);
        require(model->features().size() == 2 && viewer->hasBody(), "panel extrude rebuilds displayed body");
        palette->exitSketchRequested();
        const auto cameraAfter = viewer->cameraParameters();
        require(palette->isHidden() && sameVector(cameraBefore.position, cameraAfter.position)
                && sameVector(cameraBefore.target, cameraAfter.target)
                && cameraBefore.parallelProjection == cameraAfter.parallelProjection,
                "sketch exit restores camera and hides tools");

        viewer->datumPlanePicked(modeling::DatumPlane::XY);
        newSketch->trigger();
        palette->toolChanged(ViewportTool::Circle);
        viewer->sketchPointRequested({0, 0});
        viewer->sketchPointRequested({1, 0});
        sketch->cutRequested(4, true, true);
        require(model->features().size() == 4 && model->lastError().isEmpty(), "circle sketch can cut through extruded body");
        palette->exitSketchRequested();

        // Deleting the currently edited sketch must close its UI synchronously.
        viewer->datumPlanePicked(modeling::DatumPlane::YZ);
        newSketch->trigger();
        const auto deletedId = model->features().back().id;
        palette->toolChanged(ViewportTool::Point);
        viewer->sketchPointRequested({0, 0});
        viewer->sketchEntityPicked(model->sketchParams(deletedId)->entities.front().id);
        viewer->sketchDeleteRequested();
        require(model->sketchParams(deletedId)->entities.empty(), "viewport delete removes selected entity");
        require(model->removeFeature(deletedId, true), "remove active sketch through core");
        require(!sketch->hasSketch() && palette->isHidden(), "rebuild removes deleted sketch session");

        // Import replaces the displayed body without modifying the feature graph.
        const QString modelPath = temp.filePath("box.step");
        STEPControl_Writer writer;
        require(writer.Transfer(BRepPrimAPI_MakeBox(10, 8, 6).Shape(), STEPControl_AsIs) == IFSelect_RetDone,
                "prepare STEP fixture");
        require(writer.Write(modelPath.toLocal8Bit().constData()) == IFSelect_RetDone, "write STEP fixture");
        chooseFile(window, action(window, "Open Model"), modelPath);
        require(viewer->hasBody() && model->features().size() == 4
                && console->toPlainText().contains("STEP model loaded from box.step"), "model import action retains document behavior");

        QImage texture(32, 32, QImage::Format_RGB32);
        texture.fill(Qt::cyan);
        const QString texturePath = temp.filePath("texture.png");
        require(texture.save(texturePath), "texture fixture");
        chooseFile(window, action(window, "Apply Texture..."), texturePath);
        require(console->toPlainText().contains("Texture projected from texture.png"), "texture action reaches model view controller");
        properties->renderPanel()->textureCleared();
        require(console->toPlainText().contains("Texture removed"), "texture clear intent reaches controller");

        auto* capture = window.findChild<CaptureController*>();
        auto* capturePanel = properties->capturePanel();
        const QString captureRoot = temp.filePath("captures");
        capturePanel->setBaseDirectory(captureRoot);
        capturePanel->baseDirectoryChanged(captureRoot);
        capturePanel->capturePhotoRequested();
        require(capture->shots().size() == 1, "photo intent produces exactly one shot");
        const auto& shot = capture->shots().back();
        const QString groupPath = QDir(captureRoot).filePath(shot.groupName);
        require(!QImage(QDir(groupPath).filePath(shot.imageName)).isNull(), "photo output is a readable image");
        require(QFile::exists(QDir(groupPath).filePath("manifest.json")), "photo has aggregate camera metadata");
        require(capturePanel->findChild<QTreeWidget*>("captureShotList")->topLevelItemCount() == 1,
                "capture results refresh shot list");
        auto* orbitDistance = capturePanel->findChild<QDoubleSpinBox*>("captureDistance");
        orbitDistance->setValue(1);
        action(window, "Reset Camera")->trigger();
        require(std::abs(orbitDistance->value() - capture->currentDistance()) < 0.01,
                "camera framing synchronizes capture distance");

        // Verify the existing command-line validation entry and report schema.
        QString error;
        const QString validationRoot = temp.filePath("validation");
        require(window.runPerformanceCapture(modelPath, validationRoot, QString(), error), "validation capture delegates successfully");
        QFile report(QDir(validationRoot).filePath("capture_report.json"));
        require(report.open(QIODevice::ReadOnly), "validation report exists");
        const auto data = QJsonDocument::fromJson(report.readAll()).object();
        require(data.value("imageCount").toInt() == 62 && data.value("posedImageCount").toInt() == 62
                && QFile::exists(QDir(validationRoot).filePath("camera_info.json")), "validation keeps 62 poses and camera metadata");
        require(capturePanel->findChild<QPushButton*>("capturePhotoButton")->isEnabled(),
                "orbit completion restores capture controls");
        require(!window.runPerformanceCapture(modelPath, validationRoot, QString(), error)
                && error.contains("already exists"), "validation refuses to overwrite images");

        // A finished photo reconstruction feeds the same cloud workflow as import.
        const QString cloudPath = temp.filePath("plane.ply");
        QFile cloud(cloudPath);
        require(cloud.open(QIODevice::WriteOnly), "cloud fixture");
        cloud.write("ply\nformat ascii 1.0\nelement vertex 100\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
        for (int x = 0; x < 10; ++x) for (int y = 0; y < 10; ++y) {
            cloud.write(QStringLiteral("%1 %2 0\n").arg(x).arg(y).toUtf8());
        }
        cloud.close();
        auto* reconstruction = window.findChild<ReconstructionController*>();
        reconstruction->finished(cloudPath, 100, "estimated", false);
        require(viewer->hasPointCloud()
                && console->toPlainText().contains("Point Cloud loaded: 100 points"), "reconstruction completion loads cloud and preserves automation log");
        waitForCad(window, true);

        const QString cadCloudPath = temp.filePath("cad-box.ply");
        QFile cadCloud(cadCloudPath);
        require(cadCloud.open(QIODevice::WriteOnly), "create closed CAD cloud");
        cadCloud.write("ply\nformat ascii 1.0\nelement vertex 600\nproperty float x\nproperty float y\nproperty float z\n"
                       "property float nx\nproperty float ny\nproperty float nz\nend_header\n");
        for (int axis = 0; axis < 3; ++axis) for (float sign : {-1.0F, 1.0F})
            for (int i = 0; i < 10; ++i) for (int j = 0; j < 10; ++j) {
                float p[3]{}, n[3]{};
                p[axis] = 3 * sign; n[axis] = sign;
                p[(axis + 1) % 3] = -3 + (i + 0.5F) * 0.6F;
                p[(axis + 2) % 3] = -3 + (j + 0.5F) * 0.6F;
                cadCloud.write(QStringLiteral("%1 %2 %3 %4 %5 %6\n")
                    .arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]).toUtf8());
            }
        cadCloud.close();
        QFile units(temp.filePath("cad-box.json"));
        require(units.open(QIODevice::WriteOnly), "create metric unit sidecar");
        units.write("{\"unit\":\"mm\"}"); units.close();
        testCadReconstruction(window, cadCloudPath);

        // Start analysis and destroy the window immediately, before queued results
        // are delivered. This verifies the workspace's worker/view lifetime boundary.
        auto shutdownWindow = std::make_unique<MainWindow>();
        auto* shutdownViewer = shutdownWindow->findChild<VTKViewer*>();
        require(shutdownViewer->loadPointCloud(cloudPath, error), "load shutdown fixture");
        shutdownWindow->findChild<PointCloudRelationsPanel*>()->detectRequested();
        shutdownWindow.reset();
        std::cout << "Workspace interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
