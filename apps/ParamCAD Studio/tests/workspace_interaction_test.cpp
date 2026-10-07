#include "app/MainWindow.h"
#include "app/CadReconstructionTask.h"
#include "app/CaptureController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "io/StepModelLoader.h"
#include "io/ProjectStore.h"
#include "app/ReconstructionController.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Writer.hxx>

#include <QAction>
#include <QAbstractButton>
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDialog>
#include <QToolButton>
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

#include <algorithm>
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

double volume(const TopoDS_Shape& shape)
{
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

void verifyExport(MainWindow& window, const QString& chosenPath,
                  const QString& outputPath, const TopoDS_Shape& expected)
{
    auto* viewer = window.findChild<VTKViewer*>();
    auto* model = window.findChild<ModelingController*>();
    const auto revision = model->revision();
    const auto displayedRevision = viewer->bodyRevision();
    chooseFile(window, action(window, "Export STEP..."), chosenPath);
    StepModelLoader loader;
    LoadedModel loaded;
    QString error;
    require(loader.load(outputPath, loaded, error), "UI export produces importable STEP");
    require(BRepCheck_Analyzer(loaded.shape).IsValid() && loaded.solidCount == 1
            && std::abs(volume(loaded.shape) - volume(expected)) < 1.0e-5,
            "UI export retains current solid, cut geometry and millimetre scale");
    require(model->revision() == revision && viewer->bodyRevision() == displayedRevision,
            "export preserves model history and displayed body");
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

void replayDenseCloud(const QString& path)
{
    MainWindow window;
    window.setAttribute(Qt::WA_DontShowOnScreen);
    window.show();
    QCoreApplication::processEvents();
    auto* document = window.findChild<PointCloudDocument*>();
    auto* model = window.findChild<ModelingController*>();
    auto* task = window.findChild<CadReconstructionTask*>();
    auto* reconstruct = action(window, "Reconstruct CAD");
    QElapsedTimer elapsed;
    elapsed.start();
    QString error;
    require(document->load(path, error), error.toStdString().c_str());
    std::cout << "Dense cloud load: " << elapsed.elapsed() << " ms, "
              << document->pointCount() << " points." << std::endl;
    QObject::connect(task, &CadReconstructionTask::stageChanged, &window,
        [&](CadReconstructionTask::Stage stage) {
            std::cout << "CAD stage " << static_cast<int>(stage) << " at "
                      << elapsed.elapsed() << " ms." << std::endl;
        });
    QObject::connect(model, &ModelingController::modelRebuilt, &window, [&] {
        std::cout << "CAD model rebuilt at " << elapsed.elapsed() << " ms." << std::endl;
    });
    QObject::connect(document, &PointCloudDocument::documentChanged, &window, [&] {
        std::cout << "Cloud annotations updated at " << elapsed.elapsed() << " ms." << std::endl;
    });
    elapsed.restart();
    reconstruct->trigger();
    qint64 longestDispatch = elapsed.elapsed();
    std::cout << "Reconstruct action returned after " << longestDispatch << " ms." << std::endl;
    while ((task->isRunning() || !reconstruct->isEnabled()) && elapsed.elapsed() < 60000) {
        QElapsedTimer dispatch;
        dispatch.start();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        longestDispatch = std::max(longestDispatch, dispatch.elapsed());
        QThread::msleep(1);
    }
    std::cout << "Dense reconstruction and analysis: " << elapsed.elapsed()
              << " ms; longest GUI dispatch: " << longestDispatch << " ms." << std::endl;
    require(!task->isRunning() && reconstruct->isEnabled(), "dense CAD and analysis complete");
    require(!model->features().empty() && BRepCheck_Analyzer(model->bodyShape()).IsValid(),
            "dense reconstruction publishes a valid parametric body");
    require(longestDispatch < 5000, "dense reconstruction must keep GUI dispatches below Windows hang detection");
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
    require(viewer->pointCloudDocument()->load(cloudPath, error), "load metric CAD fixture");
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
    const QString exportPath = QFileInfo(cloudPath).dir().filePath("reconstructed.step");
    verifyExport(window, exportPath, exportPath, model->bodyShape());

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
    viewer->pointCloudDocument()->selectPoint(0);
    viewer->pointCloudDocument()->deleteSelectedPoints();
    waitForCad(window);
    require(model->revision() == editedRevision && viewer->pointCloudDocument()->pointCount() == 599
            && status->text().contains("未应用"), "point deletion invalidates candidate and preserves working cloud");

    require(viewer->pointCloudDocument()->load(cloudPath, error), "restore metric cloud");
    reconstruct->trigger();
    require(viewer->pointCloudDocument()->load(cloudPath, error), "replace cloud with identical file while worker is pending");
    waitForCad(window);
    require(model->revision() == editedRevision && status->text().contains("未应用"),
            "cloud revision invalidates same-file replacement");

    reconstruct->trigger();
    window.findChild<ReconstructionController*>()->finished(cloudPath, 600, "fixed", false);
    waitForCad(window, true);
    require(model->revision() == editedRevision && viewer->pointCloudDocument()->selection().regions().size() == 6,
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
    viewer->pointCloudDocument()->clear();
    waitForCad(window);
    require(!reconstruct->isEnabled() && !cancel->isEnabled() && model->revision() == editedRevision,
            "clearing cloud prevents model commit and leaves reconstruction disabled");

    // Confirming unknown scale uses a nested event loop. A document replacement
    // inside that dialog must not pair old observations with the new revision.
    require(viewer->pointCloudDocument()->load(cloudPath, error), "load scale-dialog fixture");
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
        replaced = viewer->pointCloudDocument()->load(cloudPath, error);
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
    require(closing->findChild<VTKViewer*>()->pointCloudDocument()->load(cloudPath, error), "load CAD shutdown fixture");
    action(*closing, "Reconstruct CAD")->trigger();
    require(closing->findChild<CadReconstructionTask*>()->isRunning(), "worker running when window is destroyed");
    closing.reset();
    QCoreApplication::processEvents();
}

void testGuidedReconstruction(const QString& cloudPath)
{
    MainWindow window;
    auto* document = window.findChild<VTKViewer*>()->pointCloudDocument();
    auto* model = window.findChild<ModelingController*>();
    auto* task = window.findChild<CadReconstructionTask*>();
    auto* relations = window.findChild<PointCloudRelationsPanel*>();
    auto* assign = window.findChild<QAction*>("createPointCollectionAction");
    auto* rebuild = window.findChild<QToolButton*>("rebuildCloudTemplatesButton");
    auto* status = window.findChild<QLabel*>("pointCloudRelationStatus");
    QString error;
    require(document->load(cloudPath, error), "load guided workflow fixture");
    // Compact worker IDs differ from original IDs after editing the cloud.
    document->selectPoint(0);
    document->deleteSelectedPoints();
    std::vector<vtkIdType> selected;
    for (int i = 100; i < 200; ++i) selected.push_back(i);
    document->select(selected);
    bool created = false;
    QTimer::singleShot(0, &window, [&] {
        auto* dialog = relations->findChild<QDialog*>("createCloudCollectionDialog");
        if (!dialog) return;
        dialog->setAttribute(Qt::WA_DontShowOnScreen);
        auto* geometry = dialog->findChild<QComboBox*>("pointCloudTemplateGeometry");
        geometry->setCurrentIndex(geometry->findData("plane"));
        created = true; dialog->accept();
    });
    assign->trigger();
    require(created, "menu creates a manual collection through the template dialog");
    require(task->isRunning(), "assigning a metric template automatically starts CAD reconstruction");
    waitForCad(window, true);
    const auto* region = document->selection().region(1);
    require(region && region->templateLocked && region->source == "manual" && !region->fitStale
            && std::abs(std::abs(region->parameters["offset"].toDouble()) - 3) < 1e-6
            && document->selection().regionPoints(1) == selected,
            "guided fit preserves manual provenance, source units and original point membership");
    require(model->features().size() == 1 && model->features()[0].reconstructed
            && std::abs(volume(model->bodyShape()) - 216) < 1e-5
            && document->selection().regions().size() > 1,
            "partial plane guides complete CAD and remaining points are analyzed");
    modeling::FeatureId authored = modeling::kInvalidFeatureId;
    require(model->addSketch({}, authored), "add independent authored history");
    rebuild->click();
    waitForCad(window, true);
    require(model->features().size() == 2 && model->features()[0].id == authored
            && !model->features()[0].reconstructed && model->features()[1].reconstructed
            && std::abs(volume(model->bodyShape()) - 216) < 1e-5,
            "rebuilding replaces generated features and preserves independent authored history");
    const auto revision = model->revision();
    relations->editRegionRequested(1, "wrong sphere", "sphere");
    waitForCad(window);
    require(model->revision() == revision && status->text().contains("模板")
            && status->text().contains("失败"), "wrong template reports conflict without changing CAD");
    // Rapid changes cancel the obsolete job and reconstruct the latest template.
    relations->editRegionRequested(1, "obsolete sphere", "sphere");
    relations->editRegionRequested(1, "corrected plane", "plane");
    QElapsedTimer deadline;
    deadline.start();
    while ((model->revision() == revision || task->isRunning()
            || !action(window, "Reconstruct CAD")->isEnabled()) && deadline.elapsed() < 30000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    require(model->revision() > revision && model->features().size() == 2
            && document->selection().region(1)->geometry == "plane"
            && document->selection().regionPoints(1) == selected,
            "queued retry uses latest template snapshot without duplicating generated features");
    // A complete cuboid can also be specified using a local observed face.
    relations->editRegionRequested(1, "cuboid patch", "cuboid");
    waitForCad(window, true);
    require(document->selection().region(1)->parameters.contains("size_x")
            && model->features().size() == 2, "cuboid template completes from whole-cloud observations");
    window.findChild<QAction*>("undoCloudEditAction")->trigger();
    waitForCad(window, true);
    require(document->selection().region(1)->geometry == "plane"
            && document->selection().region(1)->parameters.contains("offset")
            && model->features().size() == 2,
            "one undo restores the template edit and reconstructs it without an automatic-fit undo loop");
    relations->editRegionRequested(1, "cuboid patch", "cuboid");
    waitForCad(window, true);

    document->selectRegion(1);
    relations->unassignRequested();
    waitForCad(window, true);
    require(document->selection().regionPointCount(1) == 0 && model->features().size() == 2
            && std::abs(volume(model->bodyShape()) - 216) < 1e-5,
            "empty manual groups stop constraining global reconstruction after unassignment");
    window.findChild<QAction*>("undoCloudEditAction")->trigger();
    waitForCad(window, true);
    require(document->selection().regionPoints(1) == selected,
            "undo unassignment restores original membership and guided reconstruction");

    ProjectSnapshot snapshot;
    snapshot.features = model->features(); snapshot.cloud = document;
    snapshot.currentBody = model->bodyShape();
    const auto projectPath = QFileInfo(cloudPath).dir().filePath("guided.pcad");
    require(ProjectStore::save(projectPath, snapshot, error), "save guided model and templates together");
    auto restored = ProjectStore::load(projectPath, error);
    if (!restored) throw std::runtime_error(error.toStdString());
    require(restored->cloud->selection().region(1)->templateLocked
            && restored->cloud->selection().regionPoints(1) == selected
            && restored->cloud->selection().region(1)->parameters.contains("size_x")
            && restored->model.features()[1].reconstructed,
            "project round-trip preserves fitted cuboid, IDs, lock and generated feature provenance");
    model->adoptProject(std::move(restored->model));
    document->adoptProject(std::move(*restored->cloud));
    rebuild->click(); waitForCad(window, true);
    require(model->features().size() == 2 && std::abs(volume(model->bodyShape()) - 216) < 1e-5,
            "rebuilding a restored guided project replaces generated features without growth");

    // Unknown-scale edits store the template without opening an automatic modal.
    QFile metadata(QFileInfo(document->sourcePath()).dir().filePath(
        QFileInfo(document->sourcePath()).completeBaseName() + ".json"));
    require(metadata.open(QIODevice::WriteOnly | QIODevice::Truncate), "set unknown scale for template test");
    metadata.write("{\"unit\":\"arbitrary\"}"); metadata.close();
    const auto beforeScale = model->revision();
    relations->editRegionRequested(1, "unknown-scale cuboid", "cuboid");
    require(!task->isRunning() && status->text().contains("尺度未知"),
            "automatic template reconstruction requires explicit unknown-scale confirmation");
    bool declined = false;
    QTimer::singleShot(0, &window, [&] {
        if (auto* box = window.findChild<QMessageBox*>()) {
            declined = true; box->button(QMessageBox::No)->click();
        }
    });
    rebuild->click();
    require(declined && !task->isRunning() && model->revision() == beforeScale,
            "declining guided scale assumption preserves CAD and template");
    bool accepted = false;
    QTimer::singleShot(0, &window, [&] {
        if (auto* box = window.findChild<QMessageBox*>()) {
            accepted = true; box->button(QMessageBox::Yes)->click();
        }
    });
    rebuild->click(); waitForCad(window, true);
    require(accepted && model->revision() > beforeScale && model->features().size() == 2,
            "explicit guided scale confirmation completes whole reconstruction");
}
}

int main(int argc, char** argv)
{
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication application(argc, argv);
    try {
        if (argc == 3 && QString::fromLocal8Bit(argv[1]) == "--replay-cloud") {
            replayDenseCloud(QString::fromLocal8Bit(argv[2]));
            return 0;
        }
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
        auto* exportStep = action(window, "Export STEP...");
        require(!exportStep->isEnabled(), "empty model disables STEP export");

        auto* newSketch = action(window, "New Sketch");
        require(!newSketch->isEnabled() && model->features().empty(), "empty scene requires a plane before sketching");
        const auto cameraBefore = viewer->cameraParameters();
        viewer->datumPlanePicked(modeling::DatumPlane::XY);
        require(newSketch->isEnabled(), "plane pick enables sketch action");
        newSketch->trigger();
        require(sketch->hasSketch() && !palette->isHidden() && !newSketch->isEnabled()
                && viewer->cameraParameters().parallelProjection && !exportStep->isEnabled(),
                "sketch entry shows workspace and keeps export disabled before a body exists");
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
        require(exportStep->isEnabled(), "extrude enables STEP export");
        verifyExport(window, temp.filePath("extruded"), temp.filePath("extruded.step"), model->bodyShape());

        viewer->datumPlanePicked(modeling::DatumPlane::XY);
        newSketch->trigger();
        palette->toolChanged(ViewportTool::Circle);
        viewer->sketchPointRequested({0, 0});
        viewer->sketchPointRequested({1, 0});
        sketch->cutRequested(4, true, true);
        require(model->features().size() == 4 && model->lastError().isEmpty(), "circle sketch can cut through extruded body");
        palette->exitSketchRequested();

        verifyExport(window, temp.filePath("cut.step"), temp.filePath("cut.step"), model->bodyShape());

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
        viewer->setObjectTransform(100, 0, 0, 0, 0, 0, 2, 2, 2);
        verifyExport(window, temp.filePath("imported.STP"), temp.filePath("imported.STP"),
                     BRepPrimAPI_MakeBox(10, 8, 6).Shape());
        viewer->setObjectTransform(0, 0, 0, 0, 0, 0, 1, 1, 1);

        // Cancellation has no file or success log; the existing body stays active.
        const QString beforeCancel = console->toPlainText();
        bool cancelled = false;
        QTimer::singleShot(0, &window, [&] {
            auto* dialog = window.findChild<QFileDialog*>();
            if (!dialog) return;
            cancelled = true;
            dialog->reject();
        });
        exportStep->trigger();
        require(cancelled && console->toPlainText() == beforeCancel && exportStep->isEnabled(),
                "cancelled export preserves body and feedback");

        // A replacement while the modal chooser is open must not export a different body.
        const QString stalePath = temp.filePath("stale.step");
        bool replaced = false;
        QTimer::singleShot(0, &window, [&] {
            auto* dialog = window.findChild<QFileDialog*>();
            if (!dialog) return;
            replaced = true;
            viewer->setBodyShape(BRepPrimAPI_MakeBox(2, 3, 4).Shape());
            dialog->selectFile(stalePath);
            QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
        });
        exportStep->trigger();
        require(replaced && !QFile::exists(stalePath) && console->toPlainText().contains("model changed"),
                "modal model replacement prevents stale export");
        verifyExport(window, temp.filePath("replaced.step"), temp.filePath("replaced.step"),
                     BRepPrimAPI_MakeBox(2, 3, 4).Shape());
        action(window, "Close Model")->trigger();
        require(!exportStep->isEnabled(), "closing a model disables export despite retained render cache");
        QString closedError;
        require(!window.findChild<ModelViewController*>()->exportStepTo(temp.filePath("closed.step"), closedError)
                && !QFile::exists(temp.filePath("closed.step")), "closed body cannot be exported through controller");
        chooseFile(window, action(window, "Open Model"), modelPath);
        require(exportStep->isEnabled(), "new import reenables STEP export");

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
        testGuidedReconstruction(cadCloudPath);

        // Start analysis and destroy the window immediately, before queued results
        // are delivered. This verifies the workspace's worker/view lifetime boundary.
        auto shutdownWindow = std::make_unique<MainWindow>();
        auto* shutdownViewer = shutdownWindow->findChild<VTKViewer*>();
        require(shutdownViewer->pointCloudDocument()->load(cloudPath, error), "load shutdown fixture");
        shutdownWindow->findChild<PointCloudRelationsPanel*>()->detectRequested();
        shutdownWindow.reset();
        std::cout << "Workspace interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
