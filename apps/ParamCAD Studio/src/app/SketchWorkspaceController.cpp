#include "app/SketchWorkspaceController.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchEntityPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <modeling/SketchValidation.h>
#include <modeling/SemanticFaces.h>

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
    connect(m_ui.modelingPanel, &ModelingPanel::featureSelected, this, [this](modeling::FeatureId id) {
        m_ui.propertyPanel->featureDetailsPanel()->setFeature(m_modelingController->features(), id);
        if (id != modeling::kInvalidFeatureId) m_ui.propertyPanel->showFeatureDetails();
    });
    connect(m_ui.propertyPanel->featureDetailsPanel(), &FeatureDetailsPanel::featureRequested,
            m_ui.modelingPanel, &ModelingPanel::selectFeature);

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
    // A selected transient face belongs to the previous body revision.
    clearPendingSelection();
    m_ui.viewer->setBodyShape(m_modelingController->bodyShape());
    m_ui.viewer->renderNow();

    m_ui.modelingPanel->setFeatures(m_modelingController->features());
    m_ui.propertyPanel->featureDetailsPanel()->setFeature(m_modelingController->features(),
                                                        m_ui.modelingPanel->selectedFeatureId());

    const QString error = m_modelingController->lastError();
    m_ui.modelingPanel->setStatusText(
        error.isEmpty()
            ? workspaceTr("重建成功 | %1 个特征").arg(m_modelingController->features().size())
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
    m_ui.propertyPanel->featureDetailsPanel()->setFeature(m_modelingController->features(),
                                                        m_ui.modelingPanel->selectedFeatureId());
    m_ui.modelingPanel->setStatusText(message);
    emit statusTextChanged(workspaceTr("就绪 | 模型未变 | 相机：%1")
                               .arg(m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("正交投影") : workspaceTr("透视投影")));
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
    emit statusMessage(workspaceTr("草图平面：%1，按「新建草图」在其上绘制").arg(label),
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
        emit statusMessage(workspaceTr("面 %1 为曲面，无法作为草图平面").arg(faceId),
                                 4000);
        emit logWarning(workspaceTr("Face %1 is curved and cannot host a sketch; pick a planar face.")
                       .arg(faceId));
        return;
    }

    // Preserve a semantic owner when the displayed body is the current model.
    // Imported and ambiguous faces remain explicit planes, with a visible note.
    m_pendingSketchPlane = sketchapp::plane3dOf(plane);
    modeling::Plane3d pickedPlane;
    modeling::Vec3 pickedPoint;
    std::string referenceError;
    const auto& body = m_modelingController->bodyShape();
    if (!body.IsNull() && m_ui.viewer->displayedBody().IsSame(body)
        && m_ui.viewer->pickedBodyPlane(faceId,pickedPlane,pickedPoint)) {
        if (const auto reference = modeling::identifyPrimitiveFace(
                m_modelingController->features(),pickedPlane,pickedPoint,referenceError))
            m_pendingSketchPlane = *reference;
    }
    m_pendingFaceId = faceId;
    m_ui.viewer->clearDatumPlaneHighlight();
    m_ui.viewer->highlightFace(faceId);
    updateSketchEntryState();
    emit statusMessage(
        workspaceTr("已选中面 %1（平面），按「新建草图」在其上绘制").arg(faceId), 4000);
    emit logInfo(workspaceTr("Face %1 selected as sketch plane").arg(faceId));
    if (const auto* reference = std::get_if<modeling::FaceReference>(&*m_pendingSketchPlane))
        emit statusMessage(workspaceTr("已关联特征 #%1 的 %2；草图将随上游尺寸更新。")
            .arg(reference->ownerFeature).arg(QString::fromLatin1(modeling::faceRoleName(reference->role))),4000);
    else
        emit statusMessage(workspaceTr("已选择固定平面；此面未关联参数特征，不会随上游尺寸移动。"),4000);
}

void SketchWorkspaceController::onEmptyPicked()
{
    if (m_sketchMode) {
        return;
    }
    clearPendingSelection();
}

void SketchWorkspaceController::resetForProject()
{
    m_sketchFeatureId = modeling::kInvalidFeatureId;
    if (m_sketchMode) {
        exitSketchMode();
    }
    m_hasSketchFrame = false;
    m_hasDraftAnchor = false;
    clearPendingSelection();
    pushSketchState();
    m_ui.viewer->clearSketch();
    m_ui.sketchPanel->clearSketch();
    m_ui.propertyPanel->sketchEntityPanel()->setEntity(nullptr);
    m_ui.propertyPanel->showSketchEntityPanel(false);
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
        m_ui.bottomTabs->addTab(m_ui.sketchPanel, workspaceTr("草图"));
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
        m_ui.toolPalette->setSelectionText(workspaceTr("选择：点击草图图元"));
        break;
    case ViewportTool::Point:
        m_ui.toolPalette->setSelectionText(workspaceTr("点：点击草图平面"));
        break;
    case ViewportTool::Line:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor ? workspaceTr("直线：点击终点")
                                                         : workspaceTr("直线：点击起点"));
        break;
    case ViewportTool::Rectangle:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? workspaceTr("矩形：点击对角点")
                                            : workspaceTr("矩形：点击一个角点"));
        break;
    case ViewportTool::Circle:
        m_ui.toolPalette->setSelectionText(m_hasDraftAnchor
                                            ? workspaceTr("圆：点击半径上的一点")
                                            : workspaceTr("圆：点击圆心"));
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
        m_ui.sketchPanel->setStatusText(workspaceTr("请先新建草图再绘制"));
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
        m_ui.sketchPanel->setStatusText(workspaceTr("请选择要删除的图元"));
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
        const QString message = workspaceTr("请先新建草图再拉伸。");
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
            ? workspaceTr("该草图已不在文档中。")
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
    m_ui.sketchPanel->setStatusText(workspaceTr("已拉伸 %1 mm（特征 %2）；其端面现在可作为草图平面").arg(depth).arg(extrudeId));
}

void SketchWorkspaceController::cutSketch(double depth, bool throughAll, bool reverse)
{
    if (m_sketchFeatureId == modeling::kInvalidFeatureId) {
        const QString message = workspaceTr("请先新建草图再切除。");
        emit logError(message);
        m_ui.sketchPanel->setStatusText(message);
        return;
    }

    const modeling::SketchFeatureParams* params =
        m_modelingController->sketchParams(m_sketchFeatureId);
    std::string error;
    if (params == nullptr || !modeling::validateSketch(*params, error)) {
        const QString message = params == nullptr
            ? workspaceTr("该草图已不在文档中。")
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
    m_ui.sketchPanel->setStatusText(workspaceTr("已应用切除（特征 %1）").arg(cutId));
}

void SketchWorkspaceController::deleteFeature(modeling::FeatureId featureId)
{
    // Deleting a sketch that an extrusion still references would leave a document
    // that cannot rebuild, so the dependents are named before the user confirms
    // and the removal then cascades (core request: cascade delete).
    const std::vector<modeling::FeatureId> dependents =
        m_modelingController->dependentsOf(featureId);

    QString message = workspaceTr("删除特征 %1？").arg(featureId);
    if (!dependents.empty()) {
        QStringList ids;
        for (const modeling::FeatureId id : dependents) {
            ids << QString::number(id);
        }
        message = workspaceTr("特征 %1 被特征 %2 引用。\n\n删除它也会删除这些特征。")
                      .arg(featureId)
                      .arg(ids.join(QStringLiteral(", ")));
    }

    if (QMessageBox::question(m_dialogParent, workspaceTr("删除特征"), message,
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

