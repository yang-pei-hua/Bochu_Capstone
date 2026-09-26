#pragma once

#include "app/ModelingController.h"
#include "app/ReconstructionController.h"
#include "core/Scene.h"

#include <QMainWindow>

class QAction;
class CaptureController;
class QLabel;
class ModelingPanel;
class QPlainTextEdit;
class PropertyPanel;
class ScenePanel;
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
    void generateDemoModel(const DemoModelParameters& parameters);
    void onModelRebuilt();
    void onModelError(const QString& message);
    void startReconstruction(const ReconstructRequest& request);
    void onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                  const QString& cameraMode, bool dense);
    void openModel();
    void openPointCloud();
    void closeModel();
    void saveProject();
    void captureImage();
    void capturePhoto();
    void captureOrbit(int count, double elevationDeg, double distance);
    void showAbout();
    void updateCameraStatus(bool parallelProjection);

    Scene m_scene;
    VTKViewer* m_viewer = nullptr;
    ModelingController* m_modelingController = nullptr;
    CaptureController* m_captureController = nullptr;
    ReconstructionController* m_reconstructionController = nullptr;
    ModelingPanel* m_modelingPanel = nullptr;
    bool m_fitViewOnNextRebuild = false;
    ScenePanel* m_scenePanel = nullptr;
    PropertyPanel* m_propertyPanel = nullptr;
    QPlainTextEdit* m_console = nullptr;
    QLabel* m_statusLabel = nullptr;

    QAction* m_openAction = nullptr;
    QAction* m_openPointCloudAction = nullptr;
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
    QAction* m_captureAction = nullptr;
    QAction* m_captureToolAction = nullptr;
    QAction* m_reconstructToolAction = nullptr;
};
