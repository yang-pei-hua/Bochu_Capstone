#pragma once

#include "core/Scene.h"

#include <QMainWindow>

class QAction;
class QLabel;
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

    QWidget* createPipelineBar();
    void openModel();
    void closeModel();
    void saveProject();
    void captureImage();
    void showAbout();
    void updateCameraStatus(bool parallelProjection);

    Scene m_scene;
    VTKViewer* m_viewer = nullptr;
    ScenePanel* m_scenePanel = nullptr;
    PropertyPanel* m_propertyPanel = nullptr;
    QPlainTextEdit* m_console = nullptr;
    QLabel* m_statusLabel = nullptr;

    QAction* m_openAction = nullptr;
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
    QAction* m_projectionAction = nullptr;
    QAction* m_captureAction = nullptr;
};
