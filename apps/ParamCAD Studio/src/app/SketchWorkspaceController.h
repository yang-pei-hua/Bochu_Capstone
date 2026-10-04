#pragma once

#include "app/WorkspaceController.h"
#include "app/ModelingController.h"
#include "core/CameraController.h"
#include "core/SketchFrame.h"
#include "ui/viewport/ViewportTool.h"

#include <optional>

class QAction;
class ModelingPanel;
class PropertyPanel;
class SketchPanel;
class SketchToolPalette;
class QTabWidget;
class QPlainTextEdit;
class VTKViewer;

struct SketchWorkspaceUi
{
    VTKViewer* viewer;
    SketchToolPalette* toolPalette;
    ModelingPanel* modelingPanel;
    PropertyPanel* propertyPanel;
    SketchPanel* sketchPanel;
    QTabWidget* bottomTabs;
    QPlainTextEdit* console;
    QAction* newSketchAction;
    QAction* datumPlanesAction;
};

// Owns the sketch editing session and translates UI intents into core commands.
class SketchWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    SketchWorkspaceController(ModelingController* modelingController,
                              const SketchWorkspaceUi& ui,
                              QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
    void resetForProject();

private:
    void onModelRebuilt();
    void onModelError(const QString& message);
    // Sketch workflow. Every handler turns a viewport or panel intent into a core
    // command and the core's feature graph is then pushed back into the UI, so
    // the client never keeps a sketch of its own.
    //
    // A sketch is created on whatever plane is selected in the viewport, so the
    // workspace remembers the pending selection between a click and the
    // moment "New Sketch" is pressed.
    void onDatumPlaneSelected(modeling::DatumPlane plane);
    void onFacePicked(int faceId, bool planar, const sketchapp::PlaneFrame& plane);
    void onEmptyPicked();
    void onNewSketchRequested();
    void onExitSketch();
    void onViewportToolChanged(ViewportTool tool);
    void onSketchPointRequested(const modeling::Point2D& point);
    void ensureSketchPoint(const modeling::Point2D& point);
    void onSketchEntityPicked(modeling::SketchEntityId entityId);
    void onSketchSelectionCleared();
    void onSketchCancelled();

    void onSketchEntityRemoved(modeling::SketchEntityId entityId);
    void deleteSelectedSketchEntity();
    void onSketchEntityAdded(const modeling::SketchGeometry& geometry);
    void onSketchEntitySelected(modeling::SketchEntityId entityId);
    void onSketchEntitySelectionCleared();
    void onSketchEntityEdited();
    void extrudeSketch(double depth, bool reverse, modeling::ExtrudeOperation operation);
    void cutSketch(double depth, bool throughAll, bool reverse);
    void deleteFeature(modeling::FeatureId featureId);

    // Viewport sketch workflow: a click in the viewport picks the sketch plane (a
    // datum plane or a planar body face) and the New Sketch action turns that
    // selection into a feature.
    void beginSketch(const modeling::SketchPlaneReference& plane);
    void enterSketchMode();
    void exitSketchMode();
    void clearPendingSelection();
    void updateSketchEntryState();
    void updateSketchHint();
    void pushSketchState();

    ModelingController* const m_modelingController;
    const SketchWorkspaceUi m_ui;
    // The sketch currently open in the panel. It enters the feature graph as soon
    // as it is created, so the id is valid from the first click onwards.
    modeling::FeatureId m_sketchFeatureId = modeling::kInvalidFeatureId;

    // Viewport sketch mode: while it is on, the left mouse button draws on the
    // sketch plane instead of picking body faces, the datum planes are hidden and
    // the tool palette appears.
    bool m_sketchMode = false;
    sketchapp::PlaneFrame m_sketchFrame;
    bool m_hasSketchFrame = false;

    // Sketch mode shows a 2D orthographic view straight at the sketch plane, so
    // the 3D view the user was in is remembered and restored on exit.
    CameraParameters m_cameraBeforeSketch;
    bool m_hasCameraBeforeSketch = false;

    // The plane the next "New Sketch" will use, chosen by clicking a body face or
    // a datum plane in the viewport.
    std::optional<modeling::SketchPlaneReference> m_pendingSketchPlane;
    int m_pendingFaceId = -1;

    // First click of a two-click tool (line, rectangle, circle).
    modeling::Point2D m_draftAnchor;
    bool m_hasDraftAnchor = false;
};
