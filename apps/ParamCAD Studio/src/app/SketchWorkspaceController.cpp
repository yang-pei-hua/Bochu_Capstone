#include "app/SketchWorkspaceController.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchEntityPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <modeling/SketchValidation.h>

#include <QAction>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStringList>
#include <QTabWidget>
#include <algorithm>
#include <cmath>
#include <string>

SketchWorkspaceController::SketchWorkspaceController(
    ModelingController* modelingController, const SketchWorkspaceUi& ui,
    QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent),
      m_modelingController(modelingController), m_ui(ui)
{
    connect(m_modelingController, &ModelingController::modelRebuilt,
            this, &SketchWorkspaceController::onModelRebuilt);
    connect(m_modelingController, &ModelingController::modelError,
            this, &SketchWorkspaceController::onModelError);
}

void SketchWorkspaceController::connectUi()
{
    connect(m_ui.sketchPanel, &SketchPanel::entityRemoveRequested,
            this, &SketchWorkspaceController::onSketchEntityRemoved);
    connect(m_ui.sketchPanel, &SketchPanel::extrudeRequested, this, &SketchWorkspaceController::extrudeSketch);
    connect(m_ui.sketchPanel, &SketchPanel::cutRequested, this, &SketchWorkspaceController::cutSketch);
    connect(m_ui.sketchPanel, &SketchPanel::entitySelected, this, &SketchWorkspaceController::onSketchEntitySelected);
    connect(m_ui.sketchPanel, &SketchPanel::entitySelectionCleared,
            this, &SketchWorkspaceController::onSketchEntitySelectionCleared);
    connect(m_ui.propertyPanel->sketchEntityPanel(), &SketchEntityPanel::entityEdited,
            this, &SketchWorkspaceController::onSketchEntityEdited);
    connect(m_ui.modelingPanel, &ModelingPanel::deleteFeatureRequested,
            this, &SketchWorkspaceController::deleteFeature);

    // Viewport sketch workflow: a click picks the sketch plane and the toolbar
    // action turns that selection into a sketch.
    connect(m_ui.newSketchAction, &QAction::triggered, this, &SketchWorkspaceController::onNewSketchRequested);
    connect(m_ui.toolPalette, &SketchToolPalette::toolChanged,
            this, &SketchWorkspaceController::onViewportToolChanged);
    connect(m_ui.toolPalette, &SketchToolPalette::exitSketchRequested,
            this, &SketchWorkspaceController::onExitSketch);

    connect(m_ui.viewer, &VTKViewer::facePicked, this, &SketchWorkspaceController::onFacePicked);
    connect(m_ui.viewer, &VTKViewer::datumPlanePicked, this, &SketchWorkspaceController::onDatumPlaneSelected);
    connect(m_ui.viewer, &VTKViewer::emptyPicked, this, &SketchWorkspaceController::onEmptyPicked);
    connect(m_ui.viewer, &VTKViewer::sketchPointRequested,
            this, &SketchWorkspaceController::onSketchPointRequested);
    connect(m_ui.viewer, &VTKViewer::sketchEntityPicked,
            this, &SketchWorkspaceController::onSketchEntityPicked);
    connect(m_ui.viewer, &VTKViewer::sketchSelectionCleared,
            this, &SketchWorkspaceController::onSketchSelectionCleared);
    connect(m_ui.viewer, &VTKViewer::sketchCancelled, this, &SketchWorkspaceController::onSketchCancelled);
    connect(m_ui.viewer, &VTKViewer::sketchDeleteRequested,
            this, &SketchWorkspaceController::deleteSelectedSketchEntity);

}

void SketchWorkspaceController::onModelRebuilt()
{
    m_ui.viewer->setBodyShape(m_modelingController->bodyShape());
    m_ui.viewer->renderNow();

    m_ui.modelingPanel->setFeatures(m_modelingController->features());

    const QString error = m_modelingController->lastError();
    m_ui.modelingPanel->setStatusText(
        error.isEmpty()
            ? workspaceTr("Rebuild succeeded | %1 features").arg(m_modelingController->features().size())
            : error);

    // The feature graph is the only sketch the client has, so whatever the core
    // now holds is what both the in-viewport preview and the panel show.
    pushSketchState();
    updateSketchEntryState();
}

void SketchWorkspaceController::pushSketchState()
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
        m_ui.viewer->clearSketch();
        m_ui.sketchPanel->clearSketch();
        m_ui.propertyPanel->sketchEntityPanel()->setEntity(nullptr);
        m_ui.propertyPanel->showSketchEntityPanel(false);
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
    m_ui.sketchPanel->setSketch(*params, frame);
    m_ui.viewer->setSketch(*params, frame);
    if (m_sketchMode) {
        // A sketch on a face follows the body when an upstream feature changes, so
        // the plane the tools project onto is refreshed from the rebuilt frame.
        m_ui.viewer->setSketchPlane(frame);
    }
    // The property editor keeps describing the selected entity, whose values the
    // core may have just re-validated.
    m_ui.propertyPanel->sketchEntityPanel()->setEntity(
        m_ui.sketchPanel->findEntity(m_ui.sketchPanel->selectedEntityId()));
}

void SketchWorkspaceController::onModelError(const QString& message)
{
    emit logError(message);
    // A rejected command or a failed rebuild must be visible in the panel, not
    // only in the console; the previous body stays on screen untouched.
    m_ui.modelingPanel->setFeatures(m_modelingController->features());
    m_ui.modelingPanel->setStatusText(message);
    emit statusTextChanged(workspaceTr("Ready | Model: None | Camera: %1")
                               .arg(m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("Orthographic") : workspaceTr("Perspective")));
}

void SketchWorkspaceController::clearPendingSelection()
{
    m_pendingSketchPlane.reset();
    m_pendingFaceId = -1;
    m_ui.viewer->clearFaceHighlight();
    m_ui.viewer->clearDatumPlaneHighlight();
    updateSketchEntryState();
}

void SketchWorkspaceController::updateSketchEntryState()
{
    // "New Sketch" is inert until a plane has been picked, which is what makes
    // the entry point meaningful on an empty scene and on a modeled body alike.
    m_ui.newSketchAction->setEnabled(m_pendingSketchPlane.has_value() && !m_sketchMode);
}

void SketchWorkspaceController::onDatumPlaneSelected(modeling::DatumPlane plane)
{
    if (m_sketchMode) {
        return;
    }
    m_pendingSketchPlane = modeling::datumPlane(plane);
    m_pendingFaceId = -1;
    m_ui.viewer->clearFaceHighlight();
    m_ui.viewer->highlightDatumPlane(plane);
    updateSketchEntryState();
    const QString label = sketchapp::planeLabel(*m_pendingSketchPlane);
    emit statusMessage(workspaceTr("Sketch plane: %1 - press New Sketch to draw on it").arg(label),
                             4000);
    emit logInfo(workspaceTr("Sketch plane %1 selected").arg(label));
}

void SketchWorkspaceController::onFacePicked(int faceId, bool planar, const sketchapp::PlaneFrame& plane)
{
    if (m_sketchMode) {
        return;
    }
    if (!planar) {
        // Only planar faces can host a sketch: a cylindrical face has no plane to
        // lay a profile out on, so the pick is refused with an explanation rather
        // than silently ignored.
        clearPendingSelection();
        emit statusMessage(workspaceTr("Face %1 is curved and cannot host a sketch").arg(faceId),
                                 4000);
        emit logWarning(workspaceTr("Face %1 is curved and cannot host a sketch; pick a planar face.")
                       .arg(faceId));
        return;
    }

    // The face is captured as an explicit world-space plane, so the sketch works
    // on this face whatever its orientation is - not only on XY/YZ/XZ.
    m_pendingSketchPlane = sketchapp::plane3dOf(plane);
    m_pendingFaceId = faceId;
    m_ui.viewer->clearDatumPlaneHighlight();
    m_ui.viewer->highlightFace(faceId);
    updateSketchEntryState();
    emit statusMessage(
        workspaceTr("Face %1 (planar) selected - press New Sketch to draw on it").arg(faceId), 4000);
    emit logInfo(workspaceTr("Face %1 selected as sketch plane").arg(faceId));
}

void SketchWorkspaceController::onEmptyPicked()
{
    if (m_sketchMode) {
        return;
    }
    clearPendingSelection();
}

void SketchWorkspaceController::onNewSketchRequested()
{
    if (!m_pendingSketchPlane.has_value()) {
        emit logWarning(workspaceTr("Select a planar face or a datum plane before starting a sketch."));
        return;
    }
    const modeling::SketchPlaneReference plane = *m_pendingSketchPlane;
    clearPendingSelection();
    beginSketch(plane);
}

void SketchWorkspaceController::beginSketch(const modeling::SketchPlaneReference& plane)
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
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    // The rebuild this command triggered ran before the id was known, so the new
    // sketch is pushed here; from now on onModelRebuilt() keeps it in sync.
    m_sketchFeatureId = createdId;
    pushSketchState();
    enterSketchMode();
    emit logInfo(workspaceTr("Sketch started (feature %1) on %2")
                .arg(createdId)
                .arg(sketchapp::planeLabel(plane)));
}

void SketchWorkspaceController::enterSketchMode()
{
    m_sketchMode = true;
    m_hasDraftAnchor = false;
    m_ui.toolPalette->setActiveTool(ViewportTool::Select);
    m_ui.toolPalette->setVisible(true);
    // Editing a sketch inserts the sketch page into the bottom tab group and
    // brings it forward; the console keeps its tab, so the log is one click away.
    if (m_ui.bottomTabs->indexOf(m_ui.sketchPanel) < 0) {
        m_ui.bottomTabs->addTab(m_ui.sketchPanel, workspaceTr("Sketch"));
    }
    m_ui.bottomTabs->setCurrentWidget(m_ui.sketchPanel);
    m_ui.viewer->setDatumPlanesVisible(false);
    m_ui.viewer->setSketchInteractionEnabled(true);
    m_ui.viewer->setViewportTool(ViewportTool::Select);
    if (m_hasSketchFrame) {
        m_ui.viewer->setSketchPlane(m_sketchFrame);
        // A sketch is drawn flat, so the view turns onto its plane in an
        // orthographic projection; the 3D view it replaced is restored on exit.
        m_cameraBeforeSketch = m_ui.viewer->cameraParameters();
        m_hasCameraBeforeSketch = true;
        m_ui.viewer->alignToSketchPlane(m_sketchFrame);
    }
    updateSketchEntryState();
    updateSketchHint();
}

void SketchWorkspaceController::exitSketchMode()
{
    m_sketchMode = false;
    m_hasDraftAnchor = false;
    m_ui.viewer->clearSketchDraft();
    m_ui.viewer->setSketchInteractionEnabled(false);
    m_ui.viewer->clearSketchPlane();
    // The flat sketch view is left behind and the 3D camera the user came from
    // (with its projection mode) is put back.
    if (m_hasCameraBeforeSketch) {
        m_ui.viewer->applyCamera(m_cameraBeforeSketch);
        m_hasCameraBeforeSketch = false;
    }
    // The drawing tools and the reference planes swap back: the model mode picks
    // its next sketch plane on the datum planes. The bottom panel returns to the
    // console and the sketch page leaves the tab group with it.
    m_ui.bottomTabs->setCurrentWidget(m_ui.console);
    const int sketchTab = m_ui.bottomTabs->indexOf(m_ui.sketchPanel);
    if (sketchTab >= 0) {
        m_ui.bottomTabs->removeTab(sketchTab);
    }
    m_ui.toolPalette->setVisible(false);
    m_ui.viewer->clearDatumPlaneHighlight();
    // Back to the 3D view, so the planes return to whatever the View menu says
    // rather than to visible, which would silently undo that choice.
    m_ui.viewer->setDatumPlanesVisible(m_ui.datumPlanesAction->isChecked());
    updateSketchEntryState();
    m_ui.viewer->renderNow();
    emit logInfo(workspaceTr("Sketch closed; the drawing tools are hidden until a sketch is started again"));
}

void SketchWorkspaceController::onExitSketch()
{
    if (!m_sketchMode) {
        return;
    }
    exitSketchMode();
}

void SketchWorkspaceController::onViewportToolChanged(ViewportTool tool)
{
    m_ui.viewer->setViewportTool(tool);
    m_hasDraftAnchor = false;
    m_ui.viewer->clearSketchDraft();
    m_ui.viewer->renderNow();

    if (!m_sketchMode) {
        return;
    }
    updateSketchHint();
}

void SketchWorkspaceController::updateSketchHint()
{
    if (!m_sketchMode) {
        return;
    }
    switch (m_ui.viewer->viewportTool()) {
    case ViewportTool::Select:
        m_ui.toolPalette->setSelectionText(workspaceTr("Select: click a sketch entity"));
        break;
    case ViewportTool::Point:
        m_ui.toolPalette->setSelectionText(workspaceTr("Point: click the sketch plane"));
        break;
    case ViewportTool::Line:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor ? workspaceTr("Line: click the end point")
                                                         : workspaceTr("Line: click the start point"));
        break;
    case ViewportTool::Rectangle:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? workspaceTr("Rectangle: click the opposite corner")
                                            : workspaceTr("Rectangle: click one corner"));
        break;
    case ViewportTool::Circle:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? workspaceTr("Circle: click a point on the radius")
                                            : workspaceTr("Circle: click the center"));
        break;
    }
}

void SketchWorkspaceController::ensureSketchPoint(const modeling::Point2D& point)
{
    // The viewer snaps a click onto an existing vertex, so a point that is not
    // already there came from a free click and has to become a point entity
    // before the line that uses it. A snapped point is a vertex's exact
    // coordinates, so a sub-micron tolerance is all this needs.
    constexpr double kExistingVertexTolerance = 1.0e-6;
    if (m_ui.viewer->hasSketchVertexAt(point, kExistingVertexTolerance)) {
        return;
    }
    onSketchEntityAdded(modeling::Point2D{point.x, point.y});
}

void SketchWorkspaceController::onSketchPointRequested(const modeling::Point2D& point)
{
    if (!m_sketchMode || m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }

    const ViewportTool tool = m_ui.viewer->viewportTool();
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
        m_ui.viewer->setSketchDraftAnchor(point);
        updateSketchHint();
        return;
    }

    const modeling::Point2D anchor = m_draftAnchor;
    m_hasDraftAnchor = false;
    m_ui.viewer->clearSketchDraft();

    switch (tool) {
    case ViewportTool::Line:
        ensureSketchPoint(point);
        onSketchEntityAdded(modeling::Line2D{anchor.x, anchor.y, point.x, point.y});
        // SolidWorks keeps drawing from the last point, so the chain continues
        // until the tool is cancelled or changed.
        m_draftAnchor = point;
        m_hasDraftAnchor = true;
        m_ui.viewer->setSketchDraftAnchor(point);
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
    m_ui.viewer->renderNow();
}

void SketchWorkspaceController::onSketchEntityPicked(modeling::SketchEntityId entityId)
{
    // The entity list is the one place a selection is kept, so the viewport pick
    // is expressed by selecting the matching row.
    m_ui.sketchPanel->selectEntity(entityId);
}

void SketchWorkspaceController::onSketchSelectionCleared()
{
    m_ui.sketchPanel->selectEntity(modeling::kInvalidSketchEntityId);
}

void SketchWorkspaceController::onSketchCancelled()
{
    m_hasDraftAnchor = false;
    m_ui.viewer->clearSketchDraft();
    m_ui.viewer->renderNow();
    if (m_sketchMode) {
        updateSketchHint();
    }
}

void SketchWorkspaceController::onSketchEntityAdded(const modeling::SketchGeometry& geometry)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        m_ui.sketchPanel->setStatusText(workspaceTr("Start a new sketch before drawing"));
        return;
    }

    // The core allocates the entity id and validates the geometry, so the entity
    // only reaches the canvas once the command has been accepted.
    modeling::SketchEntityId createdId = modeling::kInvalidSketchEntityId;
    if (!m_modelingController->addSketchEntity(m_sketchFeatureId, geometry, createdId)) {
        const QString message = m_modelingController->lastError();
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }
    emit logInfo(workspaceTr("Sketch entity %1 added").arg(createdId));
}

void SketchWorkspaceController::deleteSelectedSketchEntity()
{
    if (!m_sketchMode) {
        return;
    }
    // The keyboard acts on the same selection the Delete Entity button uses, so
    // the two paths can never disagree about what is being removed.
    const modeling::SketchEntityId id = m_ui.sketchPanel->selectedEntityId();
    if (id == modeling::kInvalidSketchEntityId) {
        m_ui.sketchPanel->setStatusText(workspaceTr("Select an entity to delete"));
        return;
    }
    onSketchEntityRemoved(id);
}

void SketchWorkspaceController::onSketchEntityRemoved(modeling::SketchEntityId entityId)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }
    if (!m_modelingController->removeSketchEntity(m_sketchFeatureId, entityId)) {
        const QString message = m_modelingController->lastError();
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }
    emit logInfo(workspaceTr("Sketch entity %1 deleted").arg(entityId));
}

void SketchWorkspaceController::onSketchEntitySelected(modeling::SketchEntityId entityId)
{
    const modeling::SketchEntity* entity = m_ui.sketchPanel->findEntity(entityId);
    m_ui.propertyPanel->sketchEntityPanel()->setEntity(entity);
    m_ui.propertyPanel->showSketchEntityPanel(entity != nullptr);
}

void SketchWorkspaceController::onSketchEntitySelectionCleared()
{
    m_ui.propertyPanel->sketchEntityPanel()->setEntity(nullptr);
    m_ui.propertyPanel->showSketchEntityPanel(false);
}

void SketchWorkspaceController::onSketchEntityEdited()
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        return;
    }
    const modeling::SketchEntityId entityId = m_ui.sketchPanel->selectedEntityId();
    if (entityId == modeling::kInvalidSketchEntityId) {
        return;
    }
    if (!m_modelingController->editSketchEntity(
            m_sketchFeatureId, entityId,
            m_ui.propertyPanel->sketchEntityPanel()->geometry())) {
        const QString message = m_modelingController->lastError();
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }
    emit logInfo(workspaceTr("Sketch entity %1 updated").arg(entityId));
}

void SketchWorkspaceController::extrudeSketch(double depth, bool reverse, modeling::ExtrudeOperation operation)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        const QString message = workspaceTr("Start a sketch before extruding.");
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
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
            ? workspaceTr("The sketch is no longer part of the document.")
            : QString::fromStdString(error);
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    modeling::FeatureId extrudeId = modeling::kInvalidFeatureId;
    if (!m_modelingController->extrude(m_sketchFeatureId, depth, reverse, operation,
                                       extrudeId)) {
        const QString message = m_modelingController->lastError();
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    emit logInfo(workspaceTr("Sketch extruded: feature %1, depth %2 mm").arg(extrudeId).arg(depth));
    m_ui.sketchPanel->setStatusText(workspaceTr("Extruded %1 mm (feature %2); its end face is now "
                                    "available as a sketch plane").arg(depth).arg(extrudeId));
}

void SketchWorkspaceController::cutSketch(double depth, bool throughAll, bool reverse)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        const QString message = workspaceTr("Start a sketch before cutting.");
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    const modeling::SketchFeatureParams* params =
        m_modelingController->sketchParams(m_sketchFeatureId);
    std::string error;
    if (params == nullptr || !modeling::validateSketch(*params, error)) {
        const QString message = params == nullptr
            ? workspaceTr("The sketch is no longer part of the document.")
            : QString::fromStdString(error);
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    // A cut on a datum plane still removes material wherever the sketch crosses
    // the body, so the core decides the outcome and its error is surfaced here.
    modeling::FeatureId cutId = modeling::kInvalidFeatureId;
    if (!m_modelingController->cut(m_sketchFeatureId, depth, throughAll, reverse, cutId)) {
        const QString message = m_modelingController->lastError();
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    emit logInfo(workspaceTr("Sketch cut: feature %1").arg(cutId));
    m_ui.sketchPanel->setStatusText(workspaceTr("Cut applied (feature %1)").arg(cutId));
}

void SketchWorkspaceController::deleteFeature(modeling::FeatureId featureId)
{
    // Deleting a sketch that an extrusion still references would leave a document
    // that cannot rebuild, so the dependents are named before the user confirms
    // and the removal then cascades (core request: cascade delete).
    const std::vector<modeling::FeatureId> dependents =
        m_modelingController->dependentsOf(featureId);

    QString message = workspaceTr("Delete feature %1?").arg(featureId);
    if (!dependents.empty()) {
        QStringList ids;
        for (const modeling::FeatureId id : dependents) {
            ids << QString::number(id);
        }
        message = workspaceTr("Feature %1 is used by feature(s) %2.\n\nDeleting it deletes them too.")
                      .arg(featureId)
                      .arg(ids.join(QStringLiteral(", ")));
    }

    if (QMessageBox::question(m_dialogParent, workspaceTr("Delete Feature"), message,
                              QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
        != QMessageBox::Yes) {
        return;
    }

    if (!m_modelingController->removeFeature(featureId, true)) {
        const QString error = m_modelingController->lastError();
        emit logError(error);
        m_ui.modelingPanel->setStatusText(error);
        return;
    }
    emit logInfo(workspaceTr("Feature %1 deleted").arg(featureId));
}

