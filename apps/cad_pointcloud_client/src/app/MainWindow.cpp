#include "app/MainWindow.h"

#include "app/CaptureController.h"
#include "app/ProjectPaths.h"
#include "app/ReconstructionController.h"
#include "core/CameraController.h"
#include "io/ModelLoader.h"
#include "io/PointCloudAdapter.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/panels/SketchEntityPanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <modeling/SketchValidation.h>
#include <reconstruction/BoxRecognizer.h>

#include <QAction>
#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStatusBar>
#include <QStringList>
#include <QStyle>
#include <QToolBar>
#include <QToolButton>

#include <algorithm>
#include <cmath>
#include <string>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("CAD & Point Cloud Studio"));
    resize(1500, 920);
    setDockNestingEnabled(true);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    createActions();
    createMenus();
    createToolBar();
    createCentralArea();
    createDockPanels();
    createStatusBar();
    createModelingController();
    createCaptureController();
    createReconstructionController();
    connectUi();
    applyDarkTheme();

    m_scenePanel->setScene(m_scene);
    m_propertyPanel->cameraPanel()->setParameters(m_viewer->cameraParameters());

    logInfo(tr("Application started"));
    logInfo(tr("VTK renderer initialized"));

    // An empty document has no body to fit, so the camera is framed on the datum
    // planes that are what the user picks a sketch plane from.
    m_viewer->resetCamera();
}

void MainWindow::createModelingController()
{
    m_modelingController = new ModelingController(this);
    connect(m_modelingController, &ModelingController::modelRebuilt,
            this, &MainWindow::onModelRebuilt);
    connect(m_modelingController, &ModelingController::modelError,
            this, &MainWindow::onModelError);
}

void MainWindow::createCaptureController()
{
    m_captureController = new CaptureController(m_viewer, this);

    // The panel shows the directory the controller will actually write to, and
    // seeds its orbit distance from wherever the camera currently sits.
    CapturePanel* capturePanel = m_propertyPanel->capturePanel();
    capturePanel->setBaseDirectory(m_captureController->baseDirectory());
    capturePanel->setDefaultDistance(m_captureController->currentDistance());
}

void MainWindow::createReconstructionController()
{
    m_reconstructionController = new ReconstructionController(this);

    // "Use Latest Capture" has to know where captures land, which only the
    // capture controller decides; the panel just mirrors it.
    m_propertyPanel->reconstructPanel()->setCaptureBaseDirectory(
        m_captureController->baseDirectory());
}

void MainWindow::startReconstruction(const ReconstructRequest& request)
{
    ReconstructPanel* panel = m_propertyPanel->reconstructPanel();
    panel->setBusy(true);
    if (!m_reconstructionController->start(request)) {
        panel->setBusy(false);
        const QString error = m_reconstructionController->lastError();
        logError(error);
        QMessageBox::warning(this, tr("Point Cloud Reconstruction"), error);
    }
}

void MainWindow::onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                          const QString& cameraMode, bool dense)
{
    ReconstructPanel* panel = m_propertyPanel->reconstructPanel();
    panel->setBusy(false);

    QString error;
    if (!m_viewer->loadPointCloud(pointCloudPath, error)) {
        logError(error);
        panel->setStatusText(error);
        QMessageBox::warning(this, tr("Point Cloud Reconstruction"), error);
        return;
    }

    // The freshly built cloud is now what the recognized-model entry works on.
    m_pointCloudPath = pointCloudPath;
    m_reconstructCadAction->setEnabled(true);
    m_viewer->resetCameraToPointCloud();
    syncCaptureDistance();
    m_scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(QFileInfo(pointCloudPath).fileName()).arg(pointCount));

    panel->setStatusText(tr("Point cloud loaded: %1 points (%2)")
                             .arg(pointCount)
                             .arg(dense ? tr("dense") : tr("sparse")));
    // Fixed wording: the UI automation script asserts on this exact line.
    logInfo(tr("Point Cloud loaded: %1 points").arg(pointCount));

    if (cameraMode == QStringLiteral("estimated")) {
        logWarning(tr("Camera poses were estimated by COLMAP; the point cloud "
                      "scale is arbitrary until a reconstruction with capture "
                      "metadata is available."));
    }
}

void MainWindow::onModelRebuilt()
{
    m_viewer->setBodyShape(m_modelingController->bodyShape());
    m_viewer->renderNow();

    m_modelingPanel->setFeatures(m_modelingController->features());

    const QString error = m_modelingController->lastError();
    m_modelingPanel->setStatusText(
        error.isEmpty()
            ? tr("Rebuild succeeded | %1 features").arg(m_modelingController->features().size())
            : error);

    // The feature graph is the only sketch the client has, so whatever the core
    // now holds is what both the in-viewport preview and the panel show.
    pushSketchState();
    updateSketchEntryState();
}

void MainWindow::pushSketchState()
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }

    const modeling::SketchFeatureParams* params =
        m_modelingController->sketchParams(m_sketchFeatureId);
    if (params == nullptr) {
        // The sketch was deleted (directly or by a cascade), so the workspace
        // closes with it instead of describing a feature that no longer exists.
        m_sketchFeatureId = modeling::kInvalidFeatureId;
        m_hasSketchFrame = false;
        m_viewer->clearSketch();
        m_sketchPanel->clearSketch();
        m_propertyPanel->sketchEntityPanel()->setEntity(nullptr);
        m_propertyPanel->showSketchEntityPanel(false);
        if (m_sketchMode) {
            exitSketchMode();
        }
        return;
    }

    sketchapp::PlaneFrame frame;
    if (!sketchapp::resolveSketchFrame(m_modelingController->features(), params->plane,
                                       frame)) {
        frame = sketchapp::datumPlaneFrame(modeling::DatumPlane::XY);
    }
    m_sketchFrame = frame;
    m_hasSketchFrame = true;
    m_sketchPanel->setSketch(*params, frame);
    m_viewer->setSketch(*params, frame);
    if (m_sketchMode) {
        // A sketch on a face follows the body when an upstream feature changes, so
        // the plane the tools project onto is refreshed from the rebuilt frame.
        m_viewer->setSketchPlane(frame);
    }
    // The property editor keeps describing the selected entity, whose values the
    // core may have just re-validated.
    m_propertyPanel->sketchEntityPanel()->setEntity(
        m_sketchPanel->findEntity(m_sketchPanel->selectedEntityId()));
}

void MainWindow::onModelError(const QString& message)
{
    logError(message);
    // A rejected command or a failed rebuild must be visible in the panel, not
    // only in the console; the previous body stays on screen untouched.
    m_modelingPanel->setFeatures(m_modelingController->features());
    m_modelingPanel->setStatusText(message);
    m_statusLabel->setText(tr("Ready | Model: None | Camera: %1")
                               .arg(m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
}

void MainWindow::clearPendingSelection()
{
    m_pendingSketchPlane.reset();
    m_pendingFaceId = -1;
    m_viewer->clearFaceHighlight();
    m_viewer->clearDatumPlaneHighlight();
    updateSketchEntryState();
}

void MainWindow::updateSketchEntryState()
{
    // "New Sketch" is inert until a plane has been picked, which is what makes
    // the entry point meaningful on an empty scene and on a modeled body alike.
    m_newSketchAction->setEnabled(m_pendingSketchPlane.has_value() && !m_sketchMode);
}

void MainWindow::onDatumPlaneSelected(modeling::DatumPlane plane)
{
    if (m_sketchMode) {
        return;
    }
    m_pendingSketchPlane = modeling::datumPlane(plane);
    m_pendingFaceId = -1;
    m_viewer->clearFaceHighlight();
    m_viewer->highlightDatumPlane(plane);
    updateSketchEntryState();
    const QString label = sketchapp::planeLabel(*m_pendingSketchPlane);
    statusBar()->showMessage(tr("Sketch plane: %1 - press New Sketch to draw on it").arg(label),
                             4000);
    logInfo(tr("Sketch plane %1 selected").arg(label));
}

void MainWindow::onFacePicked(int faceId, bool planar, const sketchapp::PlaneFrame& plane)
{
    if (m_sketchMode) {
        return;
    }
    if (!planar) {
        // Only planar faces can host a sketch: a cylindrical face has no plane to
        // lay a profile out on, so the pick is refused with an explanation rather
        // than silently ignored.
        clearPendingSelection();
        statusBar()->showMessage(tr("Face %1 is curved and cannot host a sketch").arg(faceId),
                                 4000);
        logWarning(tr("Face %1 is curved and cannot host a sketch; pick a planar face.")
                       .arg(faceId));
        return;
    }

    // The face is captured as an explicit world-space plane, so the sketch works
    // on this face whatever its orientation is - not only on XY/YZ/XZ.
    m_pendingSketchPlane = sketchapp::plane3dOf(plane);
    m_pendingFaceId = faceId;
    m_viewer->clearDatumPlaneHighlight();
    m_viewer->highlightFace(faceId);
    updateSketchEntryState();
    statusBar()->showMessage(
        tr("Face %1 (planar) selected - press New Sketch to draw on it").arg(faceId), 4000);
    logInfo(tr("Face %1 selected as sketch plane").arg(faceId));
}

void MainWindow::onEmptyPicked()
{
    if (m_sketchMode) {
        return;
    }
    clearPendingSelection();
}

void MainWindow::onNewSketchRequested()
{
    if (!m_pendingSketchPlane.has_value()) {
        logWarning(tr("Select a planar face or a datum plane before starting a sketch."));
        return;
    }
    const modeling::SketchPlaneReference plane = *m_pendingSketchPlane;
    clearPendingSelection();
    beginSketch(plane);
}

void MainWindow::beginSketch(const modeling::SketchPlaneReference& plane)
{
    // The document starts empty and only sketching fills it, so a new sketch is
    // simply appended to the feature graph the previous ones built.
    //
    // The sketch enters the feature graph right away, so an unfinished one is
    // persisted and editable instead of living only on the client (core request
    // F3 makes an open profile a normal, non-fatal state).
    modeling::SketchFeatureParams params;
    params.plane = plane;
    modeling::FeatureId createdId = modeling::kInvalidFeatureId;
    if (!m_modelingController->addSketch(params, createdId)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    // The rebuild this command triggered ran before the id was known, so the new
    // sketch is pushed here; from now on onModelRebuilt() keeps it in sync.
    m_sketchFeatureId = createdId;
    pushSketchState();
    enterSketchMode();
    logInfo(tr("Sketch started (feature %1) on %2")
                .arg(createdId)
                .arg(sketchapp::planeLabel(plane)));
}

void MainWindow::enterSketchMode()
{
    m_sketchMode = true;
    m_hasDraftAnchor = false;
    m_toolPalette->setActiveTool(ViewportTool::Select);
    m_toolPalette->setVisible(true);
    m_viewer->setDatumPlanesVisible(false);
    m_viewer->setSketchInteractionEnabled(true);
    m_viewer->setViewportTool(ViewportTool::Select);
    if (m_hasSketchFrame) {
        m_viewer->setSketchPlane(m_sketchFrame);
        // A sketch is drawn flat, so the view turns onto its plane in an
        // orthographic projection; the 3D view it replaced is restored on exit.
        m_cameraBeforeSketch = m_viewer->cameraParameters();
        m_hasCameraBeforeSketch = true;
        m_viewer->alignToSketchPlane(m_sketchFrame);
    }
    updateSketchEntryState();
    updateSketchHint();
}

void MainWindow::exitSketchMode()
{
    m_sketchMode = false;
    m_hasDraftAnchor = false;
    m_viewer->clearSketchDraft();
    m_viewer->setSketchInteractionEnabled(false);
    m_viewer->clearSketchPlane();
    // The flat sketch view is left behind and the 3D camera the user came from
    // (with its projection mode) is put back.
    if (m_hasCameraBeforeSketch) {
        m_viewer->applyCamera(m_cameraBeforeSketch);
        m_hasCameraBeforeSketch = false;
    }
    // The drawing tools and the reference planes swap back: the model mode picks
    // its next sketch plane on the datum planes.
    m_toolPalette->setVisible(false);
    m_viewer->clearDatumPlaneHighlight();
    m_viewer->setDatumPlanesVisible(true);
    updateSketchEntryState();
    m_viewer->renderNow();
    logInfo(tr("Sketch closed; the drawing tools are hidden until a sketch is started again"));
}

void MainWindow::onExitSketch()
{
    if (!m_sketchMode) {
        return;
    }
    exitSketchMode();
}

void MainWindow::onViewportToolChanged(ViewportTool tool)
{
    m_viewer->setViewportTool(tool);
    m_hasDraftAnchor = false;
    m_viewer->clearSketchDraft();
    m_viewer->renderNow();

    if (!m_sketchMode) {
        return;
    }
    updateSketchHint();
}

void MainWindow::updateSketchHint()
{
    if (!m_sketchMode) {
        return;
    }
    switch (m_viewer->viewportTool()) {
    case ViewportTool::Select:
        m_toolPalette->setSelectionText(tr("Select: click a sketch entity"));
        break;
    case ViewportTool::Point:
        m_toolPalette->setSelectionText(tr("Point: click the sketch plane"));
        break;
    case ViewportTool::Line:
        m_toolPalette->setSelectionText(m_hasDraftAnchor ? tr("Line: click the end point")
                                                         : tr("Line: click the start point"));
        break;
    case ViewportTool::Rectangle:
        m_toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? tr("Rectangle: click the opposite corner")
                                            : tr("Rectangle: click one corner"));
        break;
    case ViewportTool::Circle:
        m_toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? tr("Circle: click a point on the radius")
                                            : tr("Circle: click the center"));
        break;
    }
}

void MainWindow::ensureSketchPoint(const modeling::Point2D& point)
{
    // The viewer snaps a click onto an existing vertex, so a point that is not
    // already there came from a free click and has to become a point entity
    // before the line that uses it. A snapped point is a vertex's exact
    // coordinates, so a sub-micron tolerance is all this needs.
    constexpr double kExistingVertexTolerance = 1.0e-6;
    if (m_viewer->hasSketchVertexAt(point, kExistingVertexTolerance)) {
        return;
    }
    onSketchEntityAdded(modeling::Point2D{point.x, point.y});
}

void MainWindow::onSketchPointRequested(const modeling::Point2D& point)
{
    if (!m_sketchMode || m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }

    const ViewportTool tool = m_viewer->viewportTool();
    if (tool == ViewportTool::Point) {
        onSketchEntityAdded(modeling::Point2D{point.x, point.y});
        return;
    }

    if (!m_hasDraftAnchor) {
        // The first click only anchors the operation; the viewer rubber-bands from
        // it until the second click completes the shape. A line anchors on a real
        // point, so a click that missed existing geometry becomes one.
        if (tool == ViewportTool::Line) {
            ensureSketchPoint(point);
        }
        m_draftAnchor = point;
        m_hasDraftAnchor = true;
        m_viewer->setSketchDraftAnchor(point);
        updateSketchHint();
        return;
    }

    const modeling::Point2D anchor = m_draftAnchor;
    m_hasDraftAnchor = false;
    m_viewer->clearSketchDraft();

    switch (tool) {
    case ViewportTool::Line:
        ensureSketchPoint(point);
        onSketchEntityAdded(modeling::Line2D{anchor.x, anchor.y, point.x, point.y});
        // SolidWorks keeps drawing from the last point, so the chain continues
        // until the tool is cancelled or changed.
        m_draftAnchor = point;
        m_hasDraftAnchor = true;
        m_viewer->setSketchDraftAnchor(point);
        break;
    case ViewportTool::Rectangle:
        onSketchEntityAdded(modeling::Rectangle2D{
            std::min(anchor.x, point.x), std::min(anchor.y, point.y),
            std::abs(point.x - anchor.x), std::abs(point.y - anchor.y)});
        break;
    case ViewportTool::Circle: {
        const double radius = std::hypot(point.x - anchor.x, point.y - anchor.y);
        onSketchEntityAdded(modeling::Circle2D{anchor.x, anchor.y, radius});
        break;
    }
    default:
        break;
    }
    updateSketchHint();
    m_viewer->renderNow();
}

void MainWindow::onSketchEntityPicked(modeling::SketchEntityId entityId)
{
    // The entity list is the one place a selection is kept, so the viewport pick
    // is expressed by selecting the matching row.
    m_sketchPanel->selectEntity(entityId);
}

void MainWindow::onSketchSelectionCleared()
{
    m_sketchPanel->selectEntity(modeling::kInvalidSketchEntityId);
}

void MainWindow::onSketchCancelled()
{
    m_hasDraftAnchor = false;
    m_viewer->clearSketchDraft();
    m_viewer->renderNow();
    if (m_sketchMode) {
        updateSketchHint();
    }
}

void MainWindow::onSketchEntityAdded(const modeling::SketchGeometry& geometry)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        m_sketchPanel->setStatusText(tr("Start a new sketch before drawing"));
        return;
    }

    // The core allocates the entity id and validates the geometry, so the entity
    // only reaches the canvas once the command has been accepted.
    modeling::SketchEntityId createdId = modeling::kInvalidSketchEntityId;
    if (!m_modelingController->addSketchEntity(m_sketchFeatureId, geometry, createdId)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }
    logInfo(tr("Sketch entity %1 added").arg(createdId));
}

void MainWindow::deleteSelectedSketchEntity()
{
    if (!m_sketchMode) {
        return;
    }
    // The keyboard acts on the same selection the Delete Entity button uses, so
    // the two paths can never disagree about what is being removed.
    const modeling::SketchEntityId id = m_sketchPanel->selectedEntityId();
    if (id == modeling::kInvalidSketchEntityId) {
        m_sketchPanel->setStatusText(tr("Select an entity to delete"));
        return;
    }
    onSketchEntityRemoved(id);
}

void MainWindow::onSketchEntityRemoved(modeling::SketchEntityId entityId)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }
    if (!m_modelingController->removeSketchEntity(m_sketchFeatureId, entityId)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }
    logInfo(tr("Sketch entity %1 deleted").arg(entityId));
}

void MainWindow::onSketchEntitySelected(modeling::SketchEntityId entityId)
{
    const modeling::SketchEntity* entity = m_sketchPanel->findEntity(entityId);
    m_propertyPanel->sketchEntityPanel()->setEntity(entity);
    m_propertyPanel->showSketchEntityPanel(entity != nullptr);
}

void MainWindow::onSketchEntitySelectionCleared()
{
    m_propertyPanel->sketchEntityPanel()->setEntity(nullptr);
    m_propertyPanel->showSketchEntityPanel(false);
}

void MainWindow::onSketchEntityEdited()
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }
    const modeling::SketchEntityId entityId = m_sketchPanel->selectedEntityId();
    if (entityId == modeling::kInvalidSketchEntityId) {
        return;
    }
    if (!m_modelingController->editSketchEntity(
            m_sketchFeatureId, entityId,
            m_propertyPanel->sketchEntityPanel()->geometry())) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }
    logInfo(tr("Sketch entity %1 updated").arg(entityId));
}

void MainWindow::extrudeSketch(double depth, bool reverse, modeling::ExtrudeOperation operation)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        const QString message = tr("Start a sketch before extruding.");
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    // An unfinished sketch is a normal editing state, so the core's own verdict -
    // not the button - decides whether an extrude can be issued, and its error is
    // what the user is shown.
    const modeling::SketchFeatureParams* params =
        m_modelingController->sketchParams(m_sketchFeatureId);
    std::string error;
    if (params == nullptr || !modeling::validateSketch(*params, error)) {
        const QString message = params == nullptr
            ? tr("The sketch is no longer part of the document.")
            : QString::fromStdString(error);
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    modeling::FeatureId extrudeId = modeling::kInvalidFeatureId;
    if (!m_modelingController->extrude(m_sketchFeatureId, depth, reverse, operation,
                                       extrudeId)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    logInfo(tr("Sketch extruded: feature %1, depth %2 mm").arg(extrudeId).arg(depth));
    m_sketchPanel->setStatusText(tr("Extruded %1 mm (feature %2); its end face is now "
                                    "available as a sketch plane").arg(depth).arg(extrudeId));
}

void MainWindow::cutSketch(double depth, bool throughAll, bool reverse)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        const QString message = tr("Start a sketch before cutting.");
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    const modeling::SketchFeatureParams* params =
        m_modelingController->sketchParams(m_sketchFeatureId);
    std::string error;
    if (params == nullptr || !modeling::validateSketch(*params, error)) {
        const QString message = params == nullptr
            ? tr("The sketch is no longer part of the document.")
            : QString::fromStdString(error);
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    // A cut on a datum plane still removes material wherever the sketch crosses
    // the body, so the core decides the outcome and its error is surfaced here.
    modeling::FeatureId cutId = modeling::kInvalidFeatureId;
    if (!m_modelingController->cut(m_sketchFeatureId, depth, throughAll, reverse, cutId)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        m_sketchPanel->setStatusText(message);
        return;
    }

    logInfo(tr("Sketch cut: feature %1").arg(cutId));
    m_sketchPanel->setStatusText(tr("Cut applied (feature %1)").arg(cutId));
}

void MainWindow::deleteFeature(modeling::FeatureId featureId)
{
    // Deleting a sketch that an extrusion still references would leave a document
    // that cannot rebuild, so the dependents are named before the user confirms
    // and the removal then cascades (core request: cascade delete).
    const std::vector<modeling::FeatureId> dependents =
        m_modelingController->dependentsOf(featureId);

    QString message = tr("Delete feature %1?").arg(featureId);
    if (!dependents.empty()) {
        QStringList ids;
        for (const modeling::FeatureId id : dependents) {
            ids << QString::number(id);
        }
        message = tr("Feature %1 is used by feature(s) %2.\n\nDeleting it deletes them too.")
                      .arg(featureId)
                      .arg(ids.join(QStringLiteral(", ")));
    }

    if (QMessageBox::question(this, tr("Delete Feature"), message,
                              QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
        != QMessageBox::Yes) {
        return;
    }

    if (!m_modelingController->removeFeature(featureId, true)) {
        const QString error = m_modelingController->lastError();
        logError(error);
        m_modelingPanel->setStatusText(error);
        return;
    }
    logInfo(tr("Feature %1 deleted").arg(featureId));
}

void MainWindow::logInfo(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[INFO] %1").arg(message));
}

void MainWindow::logWarning(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[WARNING] %1").arg(message));
}

void MainWindow::logError(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[ERROR] %1").arg(message));
}

void MainWindow::createActions()
{
    m_openAction = new QAction(style()->standardIcon(QStyle::SP_DialogOpenButton), tr("Open Model"), this);
    m_openAction->setShortcut(QKeySequence::Open);
    // A cloud produced elsewhere (or by an earlier reconstruction run) can be
    // inspected without going through the pipeline again.
    m_openPointCloudAction = new QAction(tr("Open Point Cloud..."), this);
    m_openPointCloudAction->setToolTip(tr("Load a .ply point cloud into the viewport"));
    // Turning observations into a model is the second half of the pipeline, so
    // it sits next to the loader and stays inert until a cloud exists.
    m_reconstructCadAction = new QAction(tr("Reconstruct CAD"), this);
    m_reconstructCadAction->setToolTip(
        tr("Recognize a box in the loaded point cloud and add it to the model"));
    m_reconstructCadAction->setEnabled(false);
    // Texturing is a presentation step rather than a modeling one, so it sits
    // next to the loading actions and appears as soon as a body exists.
    m_textureAction = new QAction(tr("Apply Texture..."), this);
    m_textureAction->setToolTip(
        tr("Project an image onto the model without unwrapping it"));
    // Capturing the current viewport is the data-generation step of the
    // pipeline, so it sits right next to Open Model in the toolbar.
    m_captureAction = new QAction(tr("Capture Image"), this);
    m_captureAction->setToolTip(tr("Save the current viewport as a PNG"));
    // Starting a sketch is the entry point of manual modeling, and the plane it
    // will use is picked by clicking a datum plane or a planar face in the
    // viewport, so the action stays inert until such a selection exists.
    m_newSketchAction = new QAction(tr("New Sketch"), this);
    m_newSketchAction->setToolTip(
        tr("Create a sketch on the selected datum plane or planar face"));
    m_newSketchAction->setEnabled(false);
    m_closeAction = new QAction(tr("Close Model"), this);
    m_saveAction = new QAction(style()->standardIcon(QStyle::SP_DialogSaveButton), tr("Save"), this);
    m_saveAction->setShortcut(QKeySequence::Save);
    m_exitAction = new QAction(tr("Exit"), this);
    m_exitAction->setShortcut(QKeySequence::Quit);

    m_resetCameraAction = new QAction(style()->standardIcon(QStyle::SP_BrowserReload), tr("Reset Camera"), this);
    m_frontViewAction = new QAction(tr("Front View"), this);
    m_backViewAction = new QAction(tr("Back View"), this);
    m_leftViewAction = new QAction(tr("Left View"), this);
    m_rightViewAction = new QAction(tr("Right View"), this);
    m_topViewAction = new QAction(tr("Top View"), this);
    m_bottomViewAction = new QAction(tr("Bottom View"), this);

    m_captureToolAction = new QAction(tr("Capture"), this);
    m_captureToolAction->setCheckable(true);
    m_captureToolAction->setToolTip(tr("Show the capture panel in Properties"));

    m_reconstructToolAction = new QAction(tr("点云重建"), this);
    m_reconstructToolAction->setCheckable(true);
    m_reconstructToolAction->setToolTip(tr("Reconstruct a point cloud from multi-view images"));
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(m_openAction);
    fileMenu->addAction(m_openPointCloudAction);
    fileMenu->addAction(m_reconstructCadAction);
    fileMenu->addAction(m_textureAction);
    fileMenu->addAction(m_closeAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_saveAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAction);

    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(m_resetCameraAction);
    viewMenu->addSeparator();
    viewMenu->addActions({m_frontViewAction, m_backViewAction, m_leftViewAction,
                          m_rightViewAction, m_topViewAction, m_bottomViewAction});

    QMenu* toolsMenu = menuBar()->addMenu(tr("&Tools"));
    toolsMenu->addAction(m_captureToolAction);
    toolsMenu->addAction(m_reconstructToolAction);

    QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));
    QAction* aboutAction = helpMenu->addAction(tr("About"));
    connect(aboutAction, &QAction::triggered, this, &MainWindow::showAbout);
}

void MainWindow::createToolBar()
{
    QToolBar* toolBar = addToolBar(tr("Main Tools"));
    toolBar->setObjectName(QStringLiteral("mainToolBar"));
    toolBar->setMovable(true);
    toolBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    // The toolbar carries the whole pipeline: load a model, generate the data
    // by capturing images, then the stages reserved for later versions.
    toolBar->addAction(m_openAction);
    toolBar->addSeparator();
    // The toolbar button is created explicitly so it keeps a stable object name
    // for the UI automation checks, which a QAction alone does not provide.
    auto* newSketchButton = new QToolButton(toolBar);
    newSketchButton->setObjectName(QStringLiteral("newSketchButton"));
    newSketchButton->setDefaultAction(m_newSketchAction);
    newSketchButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    toolBar->addWidget(newSketchButton);
    toolBar->addSeparator();
    toolBar->addAction(m_captureAction);
    toolBar->addAction(m_reconstructToolAction);

    const QStringList reservedStages{tr("几何拟合"), tr("AI 修改")};
    for (const QString& stage : reservedStages) {
        auto* button = new QToolButton(toolBar);
        button->setText(stage);
        button->setEnabled(false);
        button->setToolTip(tr("Reserved for a future version"));
        button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        toolBar->addWidget(button);
    }
}

void MainWindow::createCentralArea()
{
    m_viewer = new VTKViewer(this);
    m_toolPalette = new SketchToolPalette(this);

    // The tool strip sits against the left edge of the viewport rather than
    // floating over it: the VTK surface is a native window, so an overlay widget
    // would be painted behind it. It only carries the sketch tools, so it stays
    // hidden until a sketch is open - the model mode picks its plane on the
    // datum planes drawn in the viewport itself.
    m_toolPalette->setVisible(false);

    auto* central = new QWidget(this);
    auto* layout = new QHBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(m_toolPalette);
    layout->addWidget(m_viewer, 1);
    setCentralWidget(central);
}

void MainWindow::createDockPanels()
{
    m_scenePanel = new ScenePanel(this);
    auto* sceneDock = new QDockWidget(tr("Scene"), this);
    sceneDock->setObjectName(QStringLiteral("sceneDock"));
    sceneDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    sceneDock->setWidget(m_scenePanel);
    sceneDock->setMinimumWidth(210);
    addDockWidget(Qt::LeftDockWidgetArea, sceneDock);

    m_propertyPanel = new PropertyPanel(this);
    auto* propertyDock = new QDockWidget(tr("Properties"), this);
    propertyDock->setObjectName(QStringLiteral("propertyDock"));
    propertyDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    propertyDock->setWidget(m_propertyPanel);
    propertyDock->setMinimumWidth(400);
    addDockWidget(Qt::RightDockWidgetArea, propertyDock);

    m_console = new QPlainTextEdit(this);
    m_console->setReadOnly(true);
    m_console->setMaximumBlockCount(2000);
    m_console->setObjectName(QStringLiteral("console"));
    auto* consoleDock = new QDockWidget(tr("Console / Log"), this);
    consoleDock->setObjectName(QStringLiteral("consoleDock"));
    consoleDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    consoleDock->setWidget(m_console);
    consoleDock->setMinimumHeight(130);
    addDockWidget(Qt::BottomDockWidgetArea, consoleDock);

    resizeDocks({sceneDock, propertyDock}, {235, 380}, Qt::Horizontal);
    resizeDocks({consoleDock}, {150}, Qt::Vertical);

    m_modelingPanel = new ModelingPanel(this);
    auto* modelingDock = new QDockWidget(tr("Modeling"), this);
    modelingDock->setObjectName(QStringLiteral("modelingDock"));
    modelingDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    modelingDock->setWidget(m_modelingPanel);
    modelingDock->setMinimumWidth(250);
    addDockWidget(Qt::LeftDockWidgetArea, modelingDock);
    splitDockWidget(sceneDock, modelingDock, Qt::Vertical);

    // The sketch panel shares the bottom row with the console: the entity list and
    // the extrude/cut parameters need a wide slot, and the right-hand column is
    // already taken by Properties at full height. The sketch itself is drawn in
    // the 3D viewport, not here.
    m_sketchPanel = new SketchPanel(this);
    auto* sketchDock = new QDockWidget(tr("Sketch"), this);
    sketchDock->setObjectName(QStringLiteral("sketchDock"));
    sketchDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    sketchDock->setWidget(m_sketchPanel);
    sketchDock->setMinimumWidth(360);
    addDockWidget(Qt::BottomDockWidgetArea, sketchDock);
    splitDockWidget(consoleDock, sketchDock, Qt::Horizontal);
}

void MainWindow::createStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready | Model: None | Camera: Perspective"), this);
    statusBar()->addWidget(m_statusLabel, 1);
}

void MainWindow::connectUi()
{
    connect(m_openAction, &QAction::triggered, this, &MainWindow::openModel);
    connect(m_openPointCloudAction, &QAction::triggered, this, &MainWindow::openPointCloud);
    connect(m_reconstructCadAction, &QAction::triggered, this, &MainWindow::reconstructCad);
    connect(m_textureAction, &QAction::triggered, this, &MainWindow::chooseTexture);
    connect(m_captureAction, &QAction::triggered, this, &MainWindow::captureImage);
    connect(m_closeAction, &QAction::triggered, this, &MainWindow::closeModel);
    connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProject);
    connect(m_exitAction, &QAction::triggered, qApp, &QApplication::closeAllWindows);

    connect(m_resetCameraAction, &QAction::triggered, this, [this]() {
        m_viewer->resetCamera();
        // Reset Camera is the framing the user has just chosen, so the orbit
        // distance follows it instead of keeping whatever it held before.
        syncCaptureDistance();
        logInfo(tr("Camera reset"));
    });
    connect(m_frontViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Front);
        logInfo(tr("Front view selected"));
    });
    connect(m_backViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Back);
        logInfo(tr("Back view selected"));
    });
    connect(m_leftViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Left);
        logInfo(tr("Left view selected"));
    });
    connect(m_rightViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Right);
        logInfo(tr("Right view selected"));
    });
    connect(m_topViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Top);
        logInfo(tr("Top view selected"));
    });
    connect(m_bottomViewAction, &QAction::triggered, this, [this]() {
        m_viewer->setStandardView(CameraController::StandardView::Bottom);
        logInfo(tr("Bottom view selected"));
    });
    connect(m_sketchPanel, &SketchPanel::entityRemoveRequested,
            this, &MainWindow::onSketchEntityRemoved);
    connect(m_sketchPanel, &SketchPanel::extrudeRequested, this, &MainWindow::extrudeSketch);
    connect(m_sketchPanel, &SketchPanel::cutRequested, this, &MainWindow::cutSketch);
    connect(m_sketchPanel, &SketchPanel::entitySelected, this, &MainWindow::onSketchEntitySelected);
    connect(m_sketchPanel, &SketchPanel::entitySelectionCleared,
            this, &MainWindow::onSketchEntitySelectionCleared);
    connect(m_propertyPanel->sketchEntityPanel(), &SketchEntityPanel::entityEdited,
            this, &MainWindow::onSketchEntityEdited);
    connect(m_modelingPanel, &ModelingPanel::deleteFeatureRequested,
            this, &MainWindow::deleteFeature);

    // Viewport sketch workflow: a click picks the sketch plane and the toolbar
    // action turns that selection into a sketch.
    connect(m_newSketchAction, &QAction::triggered, this, &MainWindow::onNewSketchRequested);
    connect(m_toolPalette, &SketchToolPalette::toolChanged,
            this, &MainWindow::onViewportToolChanged);
    connect(m_toolPalette, &SketchToolPalette::exitSketchRequested,
            this, &MainWindow::onExitSketch);

    connect(m_viewer, &VTKViewer::facePicked, this, &MainWindow::onFacePicked);
    connect(m_viewer, &VTKViewer::datumPlanePicked, this, &MainWindow::onDatumPlaneSelected);
    connect(m_viewer, &VTKViewer::emptyPicked, this, &MainWindow::onEmptyPicked);
    connect(m_viewer, &VTKViewer::sketchPointRequested,
            this, &MainWindow::onSketchPointRequested);
    connect(m_viewer, &VTKViewer::sketchEntityPicked,
            this, &MainWindow::onSketchEntityPicked);
    connect(m_viewer, &VTKViewer::sketchSelectionCleared,
            this, &MainWindow::onSketchSelectionCleared);
    connect(m_viewer, &VTKViewer::sketchCancelled, this, &MainWindow::onSketchCancelled);
    connect(m_viewer, &VTKViewer::sketchDeleteRequested,
            this, &MainWindow::deleteSelectedSketchEntity);

    connect(m_scenePanel, &ScenePanel::nodeSelected, this, [this](SceneNodeType type) {
        if (type == SceneNodeType::Model) {
            m_propertyPanel->showObjectProperties();
        } else if (type == SceneNodeType::Camera) {
            m_propertyPanel->showCameraProperties();
        } else if (type == SceneNodeType::PointCloud) {
            // The point cloud has no properties of its own yet, so selecting it
            // surfaces the panel that produces one.
            m_propertyPanel->showReconstructPanel(true);
            m_reconstructToolAction->setChecked(true);
        }
    });

    connect(m_propertyPanel, &PropertyPanel::objectTransformChanged,
            m_viewer, &VTKViewer::setObjectTransform);
    connect(m_propertyPanel, &PropertyPanel::objectVisibilityChanged,
            m_viewer, &VTKViewer::setObjectVisible);
    connect(m_propertyPanel, &PropertyPanel::objectRepresentationChanged,
            m_viewer, &VTKViewer::setObjectRepresentation);
    connect(m_propertyPanel, &PropertyPanel::objectOpacityChanged,
            m_viewer, &VTKViewer::setObjectOpacity);
    connect(m_propertyPanel, &PropertyPanel::objectColorChanged,
            m_viewer, &VTKViewer::setObjectColor);

    CameraPanel* cameraPanel = m_propertyPanel->cameraPanel();
    connect(cameraPanel, &CameraPanel::applyRequested, m_viewer, &VTKViewer::applyCamera);
    connect(cameraPanel, &CameraPanel::resetRequested, m_resetCameraAction, &QAction::trigger);
    connect(m_viewer, &VTKViewer::cameraChanged, this, [this, cameraPanel](const CameraParameters& parameters) {
        cameraPanel->setParameters(parameters);
        updateCameraStatus(parameters.parallelProjection);
    });

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    connect(renderPanel, &RenderPanel::backgroundColorChanged,
            m_viewer, &VTKViewer::setBackgroundColor);
    connect(renderPanel, &RenderPanel::lightingChanged,
            m_viewer, &VTKViewer::setLightingEnabled);
    connect(renderPanel, &RenderPanel::captureRequested, this, &MainWindow::captureImage);
    // The texture controls only ask for a change; the dialog and the texture
    // itself stay with the window and the viewport.
    connect(renderPanel, &RenderPanel::textureBrowseRequested,
            this, &MainWindow::chooseTexture);
    connect(renderPanel, &RenderPanel::textureCleared, this, &MainWindow::clearTexture);
    connect(renderPanel, &RenderPanel::textureEnabledChanged, this, [this](bool enabled) {
        m_viewer->setBodyTextureEnabled(enabled);
        logInfo(enabled ? tr("Texture display on") : tr("Texture display off"));
    });
    connect(renderPanel, &RenderPanel::textureProjectionChanged, this, [this](int axis) {
        m_viewer->setTextureProjection(axis);
        static const char* const labels[] = {"Auto", "X", "Y", "Z", "Per Face"};
        logInfo(tr("Texture projection set to %1")
                    .arg(QString::fromLatin1(labels[std::clamp(axis, 0, 4)])));
    });

    CapturePanel* capturePanel = m_propertyPanel->capturePanel();
    connect(m_captureToolAction, &QAction::toggled, this, [this](bool visible) {
        m_propertyPanel->showCapturePanel(visible);
    });
    connect(capturePanel, &CapturePanel::baseDirectoryChanged,
            m_captureController, &CaptureController::setBaseDirectory);
    connect(capturePanel, &CapturePanel::capturePhotoRequested,
            this, &MainWindow::capturePhoto);
    connect(capturePanel, &CapturePanel::captureOrbitRequested,
            this, &MainWindow::captureOrbit);

    connect(m_captureController, &CaptureController::shotsChanged, this, [this, capturePanel]() {
        capturePanel->setShots(m_captureController->shots());
    });
    connect(m_captureController, &CaptureController::progressChanged,
            this, [this, capturePanel](const QString& message) {
                capturePanel->setStatusText(message);
                statusBar()->showMessage(message, 4000);
            });
    connect(m_captureController, &CaptureController::batchStateChanged,
            this, [this, capturePanel](bool running) {
                capturePanel->setBusy(running);
                // The orbit sweep drives the camera directly, so the panel must
                // stop echoing it back while the batch owns the pose.
                m_propertyPanel->cameraPanel()->setFeedbackSuppressed(running);
                if (running) {
                    capturePanel->setStatusText(tr("Capturing orbit..."));
                } else {
                    // The pose was restored while feedback was still suppressed,
                    // so re-read the live camera instead of trusting the stale
                    // readout to happen to match.
                    m_propertyPanel->cameraPanel()->setParameters(m_viewer->cameraParameters());
                }
            });
    connect(m_captureController, &CaptureController::captureFailed,
            this, [this](const QString& message) {
                logError(message);
                QMessageBox::warning(this, tr("Capture"), message);
            });

    ReconstructPanel* reconstructPanel = m_propertyPanel->reconstructPanel();
    connect(m_reconstructToolAction, &QAction::toggled, this, [this](bool visible) {
        m_propertyPanel->showReconstructPanel(visible);
    });
    connect(capturePanel, &CapturePanel::baseDirectoryChanged,
            reconstructPanel, &ReconstructPanel::setCaptureBaseDirectory);
    connect(reconstructPanel, &ReconstructPanel::reconstructRequested,
            this, &MainWindow::startReconstruction);
    connect(reconstructPanel, &ReconstructPanel::cancelRequested,
            m_reconstructionController, &ReconstructionController::cancel);

    connect(m_reconstructionController, &ReconstructionController::started,
            this, [this, reconstructPanel](const QString& runDirectory, const QString& cameraModeHint) {
                reconstructPanel->setStatusText(cameraModeHint);
                logInfo(tr("Reconstruction started in %1").arg(runDirectory));
            });
    connect(m_reconstructionController, &ReconstructionController::progressChanged,
            this, [this, reconstructPanel](int stage, int totalStages, const QString& label) {
                reconstructPanel->setProgress(stage, totalStages);
                reconstructPanel->setStatusText(
                    tr("Stage %1/%2: %3").arg(stage).arg(totalStages).arg(label));
                statusBar()->showMessage(tr("Reconstruction %1/%2: %3")
                                             .arg(stage).arg(totalStages).arg(label), 4000);
            });
    connect(m_reconstructionController, &ReconstructionController::logMessage,
            this, &MainWindow::logInfo);
    connect(m_reconstructionController, &ReconstructionController::failed,
            this, [this, reconstructPanel](const QString& message) {
                reconstructPanel->setBusy(false);
                reconstructPanel->setStatusText(message);
                if (m_reconstructionController->wasCancelled()) {
                    // The user asked for this, so it is a state change rather
                    // than a fault worth interrupting them with a dialog.
                    logWarning(message);
                    return;
                }
                logError(message);
                QMessageBox::warning(this, tr("Point Cloud Reconstruction"), message);
            });
    connect(m_reconstructionController, &ReconstructionController::finished,
            this, &MainWindow::onReconstructionFinished);
}

void MainWindow::openModel()
{
    const QString filePath = QFileDialog::getOpenFileName(
        this, tr("Open Model"), QString(),
        tr("Supported Models (*.step *.stp *.obj *.ply);;STEP Files (*.step *.stp);;Mesh Files (*.obj *.ply)"));
    if (filePath.isEmpty()) {
        return;
    }

    const QFileInfo info(filePath);
    m_viewer->setObjectVisible(true);
    m_statusLabel->setText(tr("Ready | Model: %1 | Camera: %2")
                               .arg(info.fileName(),
                                    m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
    logWarning(tr("Loader interface reserved; '%1' is not parsed in this skeleton.").arg(info.fileName()));
}

void MainWindow::openPointCloud()
{
    // A cloud on disk carries no capture metadata, so no scale warning applies;
    // the file is read straight into the same actor the pipeline feeds.
    const QString filePath = QFileDialog::getOpenFileName(
        this, tr("Open Point Cloud"), ProjectPaths::outputsRoot(),
        tr("Point Cloud Files (*.ply)"));
    if (filePath.isEmpty()) {
        return;
    }

    QString error;
    if (!m_viewer->loadPointCloud(filePath, error)) {
        logError(error);
        QMessageBox::warning(this, tr("Open Point Cloud"), error);
        return;
    }

    const QFileInfo info(filePath);
    const int pointCount = m_viewer->pointCloudPointCount();
    // A cloud is now in the viewport, so the recognized-model entry becomes
    // usable and works on this file.
    m_pointCloudPath = filePath;
    m_reconstructCadAction->setEnabled(true);
    m_viewer->resetCameraToPointCloud();
    syncCaptureDistance();
    m_scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(info.fileName()).arg(pointCount));
    m_statusLabel->setText(tr("Ready | Point Cloud: %1 | Camera: %2")
                               .arg(info.fileName(),
                                    m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
    logInfo(tr("Point cloud loaded from %1: %2 points").arg(info.fileName()).arg(pointCount));
}

void MainWindow::reconstructCad()
{
    if (m_pointCloudPath.isEmpty()) {
        logWarning(tr("Load a point cloud before reconstructing a model."));
        statusBar()->showMessage(tr("No point cloud is loaded"), 3500);
        return;
    }

    PlyCloud cloud;
    QString error;
    if (!PlyReader::read(m_pointCloudPath, cloud, error)) {
        logError(error);
        QMessageBox::warning(this, tr("Reconstruct CAD"), error);
        return;
    }

    // The file carries no capture metadata, so the cloud's unit is unknown and
    // is recorded as such; the tolerances below follow its size instead.
    const reconstruction::PointStore points = PointCloudAdapter::toPointStore(
        cloud, reconstruction::LengthUnit::Arbitrary);

    reconstruction::BoxCandidate candidate;
    std::string failure;
    if (!reconstruction::reconstructBox(points, PointCloudAdapter::planeOptionsFor(cloud),
                                        reconstruction::BoxRecognitionOptions{}, candidate,
                                        failure)) {
        // No box in the cloud is an ordinary outcome for an arbitrary scan, not
        // an error, so it is reported in the log and the panel rather than in a
        // dialog the user has to dismiss.
        const QString message = tr("No box could be recognized in %1: %2")
                                    .arg(QFileInfo(m_pointCloudPath).fileName(),
                                         QString::fromStdString(failure));
        logWarning(message);
        m_modelingPanel->setStatusText(message);
        statusBar()->showMessage(message, 5000);
        return;
    }

    if (!m_modelingController->commitReconstructedBox(candidate)) {
        const QString message = m_modelingController->lastError();
        logError(message);
        QMessageBox::warning(this, tr("Reconstruct CAD"), message);
        return;
    }

    const modeling::BoxPrimitiveParams& box = candidate.primitive;
    logInfo(tr("Reconstructed box: %1 x %2 x %3 at (%4, %5, %6), confidence %7")
                .arg(box.sizeX, 0, 'f', 3)
                .arg(box.sizeY, 0, 'f', 3)
                .arg(box.sizeZ, 0, 'f', 3)
                .arg(box.pose.origin.x, 0, 'f', 3)
                .arg(box.pose.origin.y, 0, 'f', 3)
                .arg(box.pose.origin.z, 0, 'f', 3)
                .arg(candidate.confidence, 0, 'f', 3));
    m_statusLabel->setText(tr("Ready | Model: Reconstructed Box | Camera: %1")
                               .arg(m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
}

void MainWindow::chooseTexture()
{
    if (!m_viewer->hasBody()) {
        logWarning(tr("Model a body before projecting a texture onto it."));
        statusBar()->showMessage(tr("Nothing to texture yet"), 3500);
        return;
    }

    // Any picture will do: it is projected onto the body rather than unwrapped
    // onto it, so no texture coordinates have to be authored first.
    const QString filePath = QFileDialog::getOpenFileName(
        this, tr("Apply Texture"), ProjectPaths::outputsRoot(),
        tr("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
    if (filePath.isEmpty()) {
        return;
    }

    QString error;
    if (!m_viewer->loadBodyTexture(filePath, error)) {
        logError(error);
        QMessageBox::warning(this, tr("Apply Texture"), error);
        return;
    }

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    renderPanel->setTexturePath(filePath);
    renderPanel->setTextureEnabled(true);
    const QString name = QFileInfo(filePath).fileName();
    statusBar()->showMessage(tr("Texture projected from %1").arg(name), 4000);
    logInfo(tr("Texture projected from %1").arg(name));
}

void MainWindow::clearTexture()
{
    m_viewer->clearBodyTexture();
    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    renderPanel->setTexturePath(QString());
    renderPanel->setTextureEnabled(false);
    logInfo(tr("Texture removed"));
}

void MainWindow::closeModel()
{
    m_viewer->setObjectVisible(false);
    m_statusLabel->setText(tr("Ready | Model: None | Camera: %1")
                               .arg(m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
    logInfo(tr("Model closed"));
}

void MainWindow::saveProject()
{
    logWarning(tr("Save is reserved for the future project format."));
    statusBar()->showMessage(tr("Save is not implemented in the UI skeleton."), 3500);
}

void MainWindow::captureImage()
{
    // A single-frame capture is named after the moment it was taken and offered
    // inside the project's outputs folder; a relative default would resolve
    // against the process working directory and could land outside the project.
    // The folder is created up front because the save dialog cannot start in a
    // directory that does not exist yet.
    const QString screenshotsRoot = ProjectPaths::screenshotsRoot();
    QDir().mkpath(screenshotsRoot);
    const QString defaultPath = QDir(screenshotsRoot).filePath(
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))
        + QStringLiteral(".png"));

    QString filePath = QFileDialog::getSaveFileName(
        this, tr("Capture Image"), defaultPath, tr("PNG Image (*.png)"));
    if (filePath.isEmpty()) {
        return;
    }
    if (!filePath.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)) {
        filePath += QStringLiteral(".png");
    }

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    if (m_viewer->captureImage(filePath, renderPanel->outputWidth(), renderPanel->outputHeight())) {
        logInfo(tr("Image captured: %1").arg(QFileInfo(filePath).fileName()));
        statusBar()->showMessage(tr("Image saved to %1").arg(filePath), 5000);
    } else {
        logError(tr("Failed to capture image: %1").arg(filePath));
        QMessageBox::warning(this, tr("Capture Image"), tr("The image could not be saved."));
    }
}

void MainWindow::capturePhoto()
{
    const RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    if (m_captureController->captureSingle(renderPanel->outputWidth(),
                                           renderPanel->outputHeight())) {
        const CaptureShot& shot = m_captureController->shots().back();
        logInfo(tr("Photo captured: %1/%2").arg(shot.groupName, shot.imageName));
    }
}

void MainWindow::captureOrbit(int count, double elevationDeg, double distance,
                              bool includeLower, double lowerElevationDeg)
{
    // Both rings share the requested count and distance; only the height they
    // are shot from differs, so one group covers the sides and the under-side.
    std::vector<OrbitRing> rings;
    rings.push_back({count, elevationDeg});
    if (includeLower) {
        rings.push_back({count, lowerElevationDeg});
    }

    const RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    const std::size_t before = m_captureController->shots().size();
    if (m_captureController->captureOrbit(rings, distance,
                                          renderPanel->outputWidth(),
                                          renderPanel->outputHeight())) {
        const std::size_t captured = m_captureController->shots().size() - before;
        logInfo(tr("Orbit capture finished: %1 photos at %2 degrees elevation")
                    .arg(captured)
                    .arg(elevationDeg, 0, 'f', 1));
    }
}

void MainWindow::syncCaptureDistance()
{
    m_propertyPanel->capturePanel()->setDefaultDistance(
        m_captureController->currentDistance());
}

void MainWindow::showAbout()
{
    QMessageBox::about(this, tr("About"),
                       tr("CAD & Point Cloud Studio\n\n"
                          "C++17 / Qt 6 Widgets / VTK UI skeleton\n"
                          "Version 0.1.0"));
}

void MainWindow::updateCameraStatus(bool parallelProjection)
{
    const QString current = m_statusLabel->text();
    const int cameraSeparator = current.lastIndexOf(QStringLiteral(" | Camera:"));
    const QString prefix = cameraSeparator >= 0 ? current.left(cameraSeparator) : tr("Ready | Model: None");
    m_statusLabel->setText(prefix + QStringLiteral(" | Camera: ")
                           + (parallelProjection ? tr("Orthographic") : tr("Perspective")));
}

void MainWindow::applyDarkTheme()
{
    qApp->setStyle(QStringLiteral("Fusion"));
    qApp->setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget { background: #25272b; color: #d7d9dc; }
        QMenuBar { background: #202226; border-bottom: 1px solid #3b3e44; }
        QMenuBar::item:selected, QMenu::item:selected { background: #3b5878; }
        QMenu { background: #292c31; border: 1px solid #454951; }
        QToolBar { background: #2b2e33; border-bottom: 1px solid #454951; spacing: 3px; }
        QToolButton, QPushButton { background: #34373d; border: 1px solid #50545c; padding: 5px 8px; }
        QToolButton:hover, QPushButton:hover { background: #41454c; }
        QToolButton:pressed, QPushButton:pressed { background: #2d638f; }
        QToolButton:checked { background: #315f86; border-color: #4f91c7; }
        QToolButton:disabled { color: #777b82; background: #292b2f; }
        QDockWidget { color: #e2e3e5; }
        QDockWidget::title { background: #303339; padding: 6px; border-bottom: 1px solid #474b52; }
        QTreeWidget, QPlainTextEdit, QTabWidget::pane { background: #202226; border: 1px solid #41444a; }
        QTreeWidget::item:selected { background: #315f86; }
        QTabBar::tab { background: #303339; border: 1px solid #44484f; padding: 6px 10px; }
        QTabBar::tab:selected { background: #3a3e45; border-bottom-color: #4f91c7; }
        QGroupBox { border: 1px solid #44484f; margin-top: 10px; padding-top: 8px; }
        QGroupBox::title { subcontrol-origin: margin; left: 7px; padding: 0 4px; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            background: #1f2125; border: 1px solid #494d54; padding: 3px; selection-background-color: #315f86;
        }
        QStatusBar { background: #202226; border-top: 1px solid #41444a; }
        #console { color: #b8c7d9; font-family: Consolas, monospace; }
        #vtkViewport { border: 0; }
    )"));
}
