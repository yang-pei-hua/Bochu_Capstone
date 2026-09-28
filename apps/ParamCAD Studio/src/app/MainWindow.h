#pragma once

#include "app/ModelingController.h"
#include "app/ReconstructionController.h"
#include "core/CameraController.h"
#include "core/Scene.h"
#include "core/SketchFrame.h"
#include "io/ModelLoader.h"
#include "ui/viewport/ViewportTool.h"

#include <QMainWindow>

#include <optional>

class QAction;
class CaptureController;
class QDockWidget;
class QLabel;
class ModelingPanel;
class QPlainTextEdit;
class PropertyPanel;
class ScenePanel;
class SketchPanel;
class SketchToolPalette;
class QTabWidget;
class VTKViewer;

class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

public slots:
    void logInfo(const QString& message);
    void logWarning(const QString& message);
    void logError(const QString& message);

private:
    void createActions();
    void createMenus();
    void createToolBar();
    void createCentralArea();
    void createDockPanels();
    void createStatusBar();
    void connectUi();
    void applyDarkTheme();

    void createModelingController();
    void createCaptureController();
    void createReconstructionController();
    void onModelRebuilt();
    void onModelError(const QString& message);

    // Sketch workflow. Every handler turns a viewport or panel intent into a core
    // command and the core's feature graph is then pushed back into the UI, so
    // the client never keeps a sketch of its own.
    //
    // A sketch is created on whatever plane is selected in the viewport, so the
    // window is what remembers the pending selection between a click and the
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

    void startReconstruction(const ReconstructRequest& request);
    void onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                  const QString& cameraMode, bool dense);
    void openModel();
    void openPointCloud();
    // Recognizes a box in the loaded cloud and commits it to the model.
    void reconstructCad();
    void chooseTexture();
    void clearTexture();
    void closeModel();
    void saveProject();
    void captureImage();
    void capturePhoto();
    void captureOrbit(int count, double elevationDeg, double distance,
                      bool includeLower, double lowerElevationDeg);
    // Re-seeds the orbit distance from the live camera so an orbit reframes the
    // model exactly the way the last camera reset did.
    void syncCaptureDistance();
    void showAbout();
    void updateCameraStatus(bool parallelProjection);

    Scene m_scene;
    VTKViewer* m_viewer = nullptr;
    SketchToolPalette* m_toolPalette = nullptr;
    ModelingController* m_modelingController = nullptr;
    CaptureController* m_captureController = nullptr;
    ReconstructionController* m_reconstructionController = nullptr;
    ModelingPanel* m_modelingPanel = nullptr;
    ScenePanel* m_scenePanel = nullptr;
    PropertyPanel* m_propertyPanel = nullptr;
    SketchPanel* m_sketchPanel = nullptr;
    QPlainTextEdit* m_console = nullptr;
    // The bottom dock is one tabbed panel, the way Properties groups Camera and
    // Render: the console has a permanent tab and the sketch page only joins the
    // group while a sketch is being edited.
    QDockWidget* m_bottomDock = nullptr;
    QTabWidget* m_bottomTabs = nullptr;
    QLabel* m_statusLabel = nullptr;

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

    QAction* m_openAction = nullptr;
    QAction* m_openPointCloudAction = nullptr;
    QAction* m_textureAction = nullptr;
    QAction* m_closeAction = nullptr;
    QAction* m_saveAction = nullptr;
    QAction* m_exitAction = nullptr;
    QAction* m_resetCameraAction = nullptr;
    QAction* m_frontViewAction = nullptr;
    QAction* m_backViewAction = nullptr;
    QAction* m_leftViewAction = nullptr;
    QAction* m_rightViewAction = nullptr;
    QAction* m_topViewAction = nullptr;
    QAction* m_bottomViewAction = nullptr;
    // Toggles the three reference planes. Sketch mode still hides them while a
    // sketch is open; this entry is the user's preference for the 3D view.
    QAction* m_datumPlanesAction = nullptr;
    QAction* m_captureAction = nullptr;
    QAction* m_newSketchAction = nullptr;
    QAction* m_captureToolAction = nullptr;
    QAction* m_reconstructToolAction = nullptr;
    QAction* m_reconstructCadAction = nullptr;

    // Cloud the "Reconstruct CAD" entry works on. It is the file last loaded
    // into the viewport, whether it was opened directly or produced here.
    QString m_pointCloudPath;

    // The imported CAD file, kept as native B-Rep for the lifetime of the
    // window. The viewer only tessellates it for display, so the shape has to
    // outlive every actor built from it - the window is what owns it, and an
    // empty shape means nothing has been imported.
    LoadedModel m_loadedModel;
};
