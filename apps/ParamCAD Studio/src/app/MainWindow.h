#pragma once

#include "core/Scene.h"

#include <QMainWindow>

class QAction;
class QDockWidget;
class QLabel;
class QPlainTextEdit;
class QTabWidget;
class ModelingPanel;
class PointCloudRelationsPanel;
class PropertyPanel;
class ScenePanel;
class SketchPanel;
class SketchToolPalette;
class VTKViewer;
class CaptureWorkspaceController;
class ModelViewController;
class ProjectController;
class PointCloudDocument;
class PointCloudWorkspaceController;
class SketchWorkspaceController;
class FeatureEditingController;

// Composition root: window layout, navigation and application feedback.
// Each workspace owns its editing state and connects its own panel intents.
class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    // CLI validation entry point; writes images and camera/capture metadata
    // below the caller's case directory via the model view controller.
    bool runPerformanceCapture(const QString& modelPath,
                               const QString& outputDirectory,
                               const QString& texturePath,
                               QString& errorMessage);

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
    void createWorkspaceControllers();
    void connectUi();
    void applyDarkTheme();
    void showAbout();
    void updateCameraStatus(bool parallelProjection);

    Scene m_scene;
    PointCloudDocument* m_pointCloudDocument = nullptr;
    VTKViewer* m_viewer = nullptr;
    SketchToolPalette* m_toolPalette = nullptr;
    ModelingPanel* m_modelingPanel = nullptr;
    PointCloudRelationsPanel* m_relationsPanel = nullptr;
    ScenePanel* m_scenePanel = nullptr;
    PropertyPanel* m_propertyPanel = nullptr;
    SketchPanel* m_sketchPanel = nullptr;
    QPlainTextEdit* m_console = nullptr;
    QDockWidget* m_bottomDock = nullptr;
    QTabWidget* m_bottomTabs = nullptr;
    QLabel* m_statusLabel = nullptr;

    SketchWorkspaceController* m_sketchWorkspace = nullptr;
    FeatureEditingController* m_featureEditing = nullptr;
    QAction* m_undoCadAction = nullptr;
    QAction* m_redoCadAction = nullptr;
    QAction* m_textEditAction = nullptr;
    PointCloudWorkspaceController* m_pointCloudWorkspace = nullptr;
    CaptureWorkspaceController* m_captureWorkspace = nullptr;
    ModelViewController* m_modelViewController = nullptr;
    ProjectController* m_projectController = nullptr;

    QAction* m_openAction = nullptr;
    QAction* m_openPointCloudAction = nullptr;
    QAction* m_textureAction = nullptr;
    QAction* m_closeAction = nullptr;
    QAction* m_saveAction = nullptr;
    QAction* m_openProjectAction = nullptr;
    QAction* m_saveAsAction = nullptr;
    QAction* m_exportStepAction = nullptr;
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
    QAction* m_cancelCadAction = nullptr;

};
