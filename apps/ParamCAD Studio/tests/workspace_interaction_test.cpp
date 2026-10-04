#include "app/MainWindow.h"
#include "app/CaptureController.h"
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
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
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
        require(viewer->hasPointCloud() && action(window, "Reconstruct CAD")->isEnabled()
                && console->toPlainText().contains("Point Cloud loaded: 100 points"), "reconstruction completion loads cloud and preserves automation log");

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
