#include "app/MainWindow.h"
#include "app/ProjectController.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/CadReconstructionTask.h"
#include "core/PointCloudDocument.h"
#include "io/PlyWriter.h"
#include "io/ProjectStore.h"
#include "ui/viewport/VTKViewer.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/SketchPanel.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <QAction>
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QImage>
#include <QMessageBox>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok, const char* message)
{
    if (!ok)
        throw std::runtime_error(message);
}
double volume(const TopoDS_Shape& shape)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(shape, p);
    return p.Mass();
}
QAction* action(MainWindow& window, const QString& text)
{
    for (auto* a : window.findChildren<QAction*>())
        if (a->text() == text)
            return a;
    throw std::runtime_error("missing project action");
}
void choose(MainWindow& window, QAction* command, const QString& path)
{
    bool found = false;
    QTimer::singleShot(0, &window, [&] {
        auto* dialog = window.findChild<QFileDialog*>();
        if (!dialog)
            return;
        found = true;
        dialog->setAttribute(Qt::WA_DontShowOnScreen);
        dialog->selectFile(path);
        QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
    });
    command->trigger();
    require(found, "project action opens a file dialog");
}
PlyCloud boxCloud()
{
    PlyCloud cloud;
    for (int face = 0; face < 6; ++face)
        for (int a = 0; a < 10; ++a)
            for (int b = 0; b < 10; ++b) {
                std::array<float, 3> p{};
                const int fixed = face / 2;
                p[fixed] = (face % 2) ? 20.0f : 0.0f;
                p[(fixed + 1) % 3] = (a + 0.5f) * 2;
                p[(fixed + 2) % 3] = (b + 0.5f) * 2;
                cloud.positions.push_back(p);
            }
    return cloud;
}
void waitForCad(CadReconstructionTask* task)
{
    QElapsedTimer timer;
    timer.start();
    while (task->isRunning() && timer.elapsed() < 30000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    require(!task->isRunning(), "cancelled CAD task completes");
    QCoreApplication::processEvents();
}
} // namespace
int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontUseNativeDialogs);
    // Fail a test with useful diagnostics instead of hanging in a warning box.
    QString warning;
    QTimer dismiss;
    dismiss.setInterval(20);
    QObject::connect(&dismiss, &QTimer::timeout, &app, [&] {
        for (auto* widget : QApplication::topLevelWidgets())
            if (auto* box = qobject_cast<QMessageBox*>(widget)) {
                warning = box->text();
                box->accept();
            }
    });
    dismiss.start();
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary project folder");
        QString error;
        MainWindow window;
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        app.processEvents();
        auto* project = window.findChild<ProjectController*>();
        auto* model = window.findChild<ModelingController*>();
        auto* modelView = window.findChild<ModelViewController*>();
        auto* viewer = window.findChild<VTKViewer*>();
        auto* cloud = viewer->pointCloudDocument();
        auto* panel = window.findChild<PropertyPanel*>();
        require(project && model && modelView && cloud && panel, "project composition is wired");
        modeling::SketchFeatureParams sketch;
        sketch.entities = {{1, modeling::Rectangle2D{0, 0, 10, 8}}};
        modeling::FeatureId sketchId = 0, extrudeId = 0;
        require(
            model->addSketch(sketch, sketchId) &&
                model->extrude(sketchId, 6, false, modeling::ExtrudeOperation::NewBody, extrudeId),
            "create editable model");
        // The imported/native active body can differ from the feature graph.
        viewer->setBodyShape(BRepPrimAPI_MakeBox(gp_Pnt(100, -40, 15), 2, 3, 4).Shape());
        const auto source = temp.filePath("cloud.ply");
        require(PlyWriter::write(boxCloud(), source, error), "write project observations");
        QFile units(temp.filePath("cloud.json"));
        require(units.open(QIODevice::WriteOnly), "write metric sidecar");
        units.write("{\"unit\":\"mm\"}");
        units.close();
        require(cloud->load(source, error), "load project observations with scale metadata");
        cloud->select({0, 1, 2});
        const int first = cloud->assignRegion("plane", "面 A");
        cloud->select({100, 101, 102});
        const int second = cloud->assignRegion("plane", "面 B");
        const int relation = cloud->setRelation(0, first, second, "parallel");
        cloud->selectPoint(599);
        cloud->deleteSelectedPoints();
        cloud->selectRelation(relation);
        QImage image(32, 32, QImage::Format_RGBA8888);
        image.fill(Qt::cyan);
        const auto texture = temp.filePath("texture.png");
        require(image.save(texture) && viewer->loadBodyTexture(texture, error), "load texture");
        viewer->setTextureProjection(4);
        viewer->setObjectTransform(11, 12, 13, 14, 15, 16, 2, 3, 4);
        viewer->setObjectOpacity(0.3);
        viewer->setObjectColor(Qt::yellow);
        viewer->setPointCloudPointSize(5);
        viewer->setPointCloudSelectionMode(2);
        action(window, "Datum Planes")->setChecked(false);
        panel->cameraPanel()->setOriginLock(false);
        CameraParameters camera;
        camera.position[0] = 42;
        camera.position[1] = -20;
        camera.position[2] = 80;
        camera.target[0] = 5;
        camera.target[1] = 4;
        camera.target[2] = 1;
        camera.parallelProjection = true;
        viewer->applyCamera(camera);
        viewer->setParallelScale(17.321);
        const auto projectPath = temp.filePath(QStringLiteral("可编辑项目.pcad"));
        choose(window, action(window, "Save"), projectPath);
        if (!warning.isEmpty())
            throw std::runtime_error(warning.toStdString());
        require(project->projectPath() == projectPath && QFile::exists(projectPath),
                "first Save writes standalone project");
        require(QFile::remove(source) && QFile::remove(temp.filePath("cloud.json")) &&
                    QFile::remove(texture),
                "remove source assets");
        const auto orphanPath = temp.filePath("orphan.pcad");
        viewer->clearBodyTexture();
        require(project->saveTo(orphanPath, error),
                "cloud remains saveable after its source file is removed");
        auto orphan = ProjectStore::load(orphanPath, error);
        require(orphan && orphan->cloud->sourceUnit() == "mm",
                "document retains scale after source sidecar removal");

        MainWindow restored;
        restored.setAttribute(Qt::WA_DontShowOnScreen);
        restored.show();
        app.processEvents();
        auto* restoredProject = restored.findChild<ProjectController*>();
        auto* restoredModel = restored.findChild<ModelingController*>();
        auto* restoredViewer = restored.findChild<VTKViewer*>();
        auto* restoredBody = restored.findChild<ModelViewController*>();
        auto* restoredCloud = restoredViewer->pointCloudDocument();
        auto* restoredPanel = restored.findChild<PropertyPanel*>();
        choose(restored, action(restored, "Open Project..."), projectPath);
        if (!warning.isEmpty())
            throw std::runtime_error(warning.toStdString());
        require(restoredModel->features().size() == 2 &&
                    std::abs(volume(restoredModel->bodyShape()) - 480) < 1e-6,
                "project opens editable history through window action");
        require(std::abs(volume(restoredBody->currentBody()) - 24) < 1e-6 &&
                    action(restored, "Export STEP...")->isEnabled(),
                "active native body is restored independently and export becomes available");
        require(restoredCloud->pointCount() == 599 && !restoredCloud->selection().active(599) &&
                    restoredCloud->selection().relation(relation) &&
                    restoredCloud->selection().selected().size() == 6,
                "document restores original IDs, deletions, annotations and selection");
        require(restoredViewer->cameraParameters().parallelProjection &&
                    std::abs(restoredViewer->parallelScale() - 17.321) < 1e-10 &&
                    restoredViewer->cameraParameters().target[0] == 5 &&
                    !restoredPanel->cameraPanel()->originLocked(),
                "panned orthographic camera and origin lock restore exactly");
        const auto object = restoredViewer->objectPresentation();
        require(object.position[0] == 11 && object.scale[2] == 4 &&
                    std::abs(object.opacity - 0.3) < 1e-9 &&
                    restoredPanel->presentation().scale[2] == 4 &&
                    restoredViewer->pointCloudPointSize() == 5 &&
                    restoredViewer->gradientBackground() &&
                    restoredViewer->pointCloudSelectionMode() == 2,
                "presentation controls and actual actors restore together");
        require(restoredViewer->hasBodyTexture() && restoredViewer->bodyTextureEnabled() &&
                    restoredViewer->textureProjection() == 4 &&
                    QFile::exists(restoredViewer->bodyTexturePath()),
                "embedded texture is published after decoding");

        // Successful opening changes live revisions monotonically, even when the
        // prepared document has fewer edits than the old document.
        const auto modelRevision = restoredModel->revision(),
                   cloudRevision = restoredCloud->revision();
        require(restoredProject->loadFrom(projectPath, error) &&
                    restoredModel->revision() > modelRevision &&
                    restoredCloud->revision() > cloudRevision,
                "load advances both live revisions");
        require(!restoredCloud->undo(), "old point edit undo cannot cross project boundary");
        sketch.entities = {{1, modeling::Rectangle2D{0, 0, 12, 8}}};
        require(restoredModel->editSketch(sketchId, sketch) &&
                    std::abs(volume(restoredBody->currentBody()) - 576) < 1e-6,
                "restored history can be edited and republishes current body");
        action(restored, "Save")->trigger();
        auto edited = ProjectStore::load(projectPath, error);
        require(edited && std::abs(volume(edited->model.bodyShape()) - 576) < 1e-6,
                "subsequent Save writes existing path without dialog");
        const auto asPath = temp.filePath("copied.pcad");
        choose(restored, action(restored, "Save Project As..."), asPath);
        require(restoredProject->projectPath() == asPath,
                "Save As changes subsequent save destination");
        bool cancelled = false;
        QTimer::singleShot(0, &restored, [&] {
            if (auto* dialog = restored.findChild<QFileDialog*>()) {
                cancelled = true;
                dialog->reject();
            }
        });
        action(restored, "Save Project As...")->trigger();
        require(cancelled && restoredProject->projectPath() == asPath,
                "cancelled Save As preserves destination");

        // Exercise an active sketch and a failed load: neither the editing
        // session nor the live documents should be touched by invalid input.
        restoredViewer->datumPlanePicked(modeling::DatumPlane::XY);
        action(restored, "New Sketch")->trigger();
        require(restoredViewer->sketchInteractionEnabled(), "enter sketch before failed load");
        const auto beforeModel = restoredModel->revision(), beforeCloud = restoredCloud->revision(),
                   beforeBody = restoredViewer->bodyRevision();
        const auto beforeCamera = restoredViewer->cameraParameters();
        const auto beforeSelection = restoredCloud->selection().selected();
        QFile bad(temp.filePath("broken.pcad"));
        require(bad.open(QIODevice::WriteOnly), "write corrupt project");
        bad.write("PCADPRJ1");
        bad.close();
        require(!restoredProject->loadFrom(bad.fileName(), error) && !error.isEmpty() &&
                    restoredModel->revision() == beforeModel &&
                    restoredCloud->revision() == beforeCloud &&
                    restoredViewer->bodyRevision() == beforeBody &&
                    restoredProject->projectPath() == asPath &&
                    restoredViewer->sketchInteractionEnabled() &&
                    restoredCloud->selection().selected() == beforeSelection &&
                    restoredViewer->cameraParameters().position[0] == beforeCamera.position[0],
                "failed open preserves all live state and sketch session");
        require(!restoredProject->saveTo(temp.filePath("missing/file.pcad"), error) &&
                    restoredProject->projectPath() == asPath,
                "failed save preserves current project destination");
        restoredProject->setCaptureBusy(true);
        require(!action(restored, "Save")->isEnabled() &&
                    !restoredProject->loadFrom(projectPath, error),
                "capture prevents project replacement");
        restoredProject->setCaptureBusy(false);
        require(restoredProject->loadFrom(projectPath, error) &&
                    !restoredViewer->sketchInteractionEnabled(),
                "valid open resets transient sketch session");

        // Queue a worker result against the old document and immediately replace
        // it. Cancellation and revisions must both keep the new project intact.
        auto* task = restored.findChild<CadReconstructionTask*>();
        action(restored, "Reconstruct CAD")->trigger();
        require(task->isRunning(), "CAD worker launched from restored metric cloud");
        require(restoredProject->loadFrom(projectPath, error),
                "project opens while CAD task is pending");
        const auto loadedRevision = restoredModel->revision();
        waitForCad(task);
        require(restoredModel->revision() == loadedRevision &&
                    std::abs(volume(restoredBody->currentBody()) - 576) < 1e-6,
                "old CAD completion cannot overwrite restored project");
        action(restored, "Close Model")->trigger();
        require(restoredProject->saveTo(asPath, error) &&
                    restoredProject->loadFrom(asPath, error) && !restoredViewer->hasBody() &&
                    !action(restored, "Export STEP...")->isEnabled() &&
                    restoredModel->features().size() == 2,
                "closed body remains closed while feature graph is retained");
        ProjectSnapshot empty;
        const auto emptyPath = temp.filePath("empty.pcad");
        require(ProjectStore::save(emptyPath, empty, error) &&
                    restoredProject->loadFrom(emptyPath, error) &&
                    restoredModel->features().empty() && !restoredCloud->hasCloud() &&
                    !restoredViewer->hasBody() && !restoredViewer->hasBodyTexture(),
                "empty project clears previous documents and texture");
        require(warning.isEmpty(), "project interactions emitted no warnings");
        std::cout << "Project interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
