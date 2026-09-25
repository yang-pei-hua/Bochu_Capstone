#include "MainWindow.h"

#include "core/CameraController.h"
#include "io/ModelLoader.h"
#include "widgets/CameraPanel.h"
#include "widgets/ModelingPanel.h"
#include "widgets/PropertyPanel.h"
#include "widgets/RenderPanel.h"
#include "widgets/ScenePanel.h"
#include "widgets/VTKViewer.h"

#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStyle>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

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
    connectUi();
    applyDarkTheme();

    m_scenePanel->setScene(m_scene);
    m_propertyPanel->cameraPanel()->setParameters(m_viewer->cameraParameters());

    logInfo(tr("Application started"));
    logInfo(tr("VTK renderer initialized"));

    createModelingController();
    generateDemoModel(m_modelingPanel->parameters());
}

void MainWindow::createModelingController()
{
    m_modelingController = new ModelingController(this);
    connect(m_modelingController, &ModelingController::modelRebuilt,
            this, &MainWindow::onModelRebuilt);
    connect(m_modelingController, &ModelingController::modelError,
            this, &MainWindow::onModelError);
}

void MainWindow::generateDemoModel(const DemoModelParameters& parameters)
{
    // A brand new document has no meaningful previous view, so fit the camera
    // once; later edits keep the camera untouched.
    m_fitViewOnNextRebuild = true;

    if (m_modelingController->createDemoModel(parameters)) {
        logInfo(tr("Demo model generated: %1 features")
                    .arg(m_modelingController->features().size()));
    } else {
        logError(tr("Demo model generation failed: %1")
                     .arg(m_modelingController->lastError()));
    }
}

void MainWindow::onModelRebuilt()
{
    m_viewer->setBodyShape(m_modelingController->bodyShape());
    if (m_fitViewOnNextRebuild) {
        m_fitViewOnNextRebuild = false;
        m_viewer->resetCamera();
    }
    m_viewer->renderNow();

    m_modelingPanel->setFeatures(m_modelingController->features());

    const QString error = m_modelingController->lastError();
    m_modelingPanel->setStatusText(
        error.isEmpty()
            ? tr("Rebuild succeeded | %1 features").arg(m_modelingController->features().size())
            : error);
}

void MainWindow::onModelError(const QString& message)
{
    logError(message);
    // A rejected command or a failed rebuild must be visible in the panel, not
    // only in the console; the previous body stays on screen untouched.
    m_modelingPanel->setFeatures(m_modelingController->features());
    m_modelingPanel->setStatusText(message);
    m_statusLabel->setText(tr("Ready | Model: Demo | Camera: %1")
                               .arg(m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
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

    m_projectionAction = new QAction(tr("Orthographic"), this);
    m_projectionAction->setCheckable(true);
    m_projectionAction->setToolTip(tr("Toggle Perspective / Orthographic"));
    m_captureAction = new QAction(style()->standardIcon(QStyle::SP_DialogSaveButton), tr("Capture Image"), this);
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(m_openAction);
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
    QAction* placeholder = toolsMenu->addAction(tr("Reserved for future tools"));
    placeholder->setEnabled(false);

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
    toolBar->addAction(m_openAction);
    toolBar->addSeparator();
    toolBar->addAction(m_resetCameraAction);
    toolBar->addAction(m_frontViewAction);
    toolBar->addAction(m_topViewAction);
    QAction* sideViewAction = toolBar->addAction(tr("Side View"));
    connect(sideViewAction, &QAction::triggered, m_rightViewAction, &QAction::trigger);
    toolBar->addSeparator();
    toolBar->addAction(m_projectionAction);
    toolBar->addSeparator();
    toolBar->addAction(m_captureAction);
}

void MainWindow::createCentralArea()
{
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(createPipelineBar());

    m_viewer = new VTKViewer(central);
    layout->addWidget(m_viewer, 1);
    setCentralWidget(central);
}

QWidget* MainWindow::createPipelineBar()
{
    auto* bar = new QFrame(this);
    bar->setObjectName(QStringLiteral("pipelineBar"));
    bar->setFixedHeight(54);
    auto* layout = new QHBoxLayout(bar);
    layout->setContentsMargins(14, 7, 14, 7);
    layout->setSpacing(2);

    const QStringList stages{
        tr("1  数据生成"), tr("2  点云重建"), tr("3  几何拟合"), tr("4  AI 修改")};

    for (int index = 0; index < stages.size(); ++index) {
        auto* button = new QToolButton(bar);
        button->setText(stages[index]);
        button->setCheckable(true);
        button->setChecked(index == 0);
        button->setEnabled(index == 0);
        button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        button->setMinimumWidth(135);
        if (index > 0) {
            button->setToolTip(tr("Reserved for a future version"));
        }
        layout->addWidget(button);
        if (index < stages.size() - 1) {
            auto* arrow = new QLabel(QStringLiteral("›"), bar);
            arrow->setObjectName(QStringLiteral("pipelineArrow"));
            arrow->setAlignment(Qt::AlignCenter);
            arrow->setFixedWidth(18);
            layout->addWidget(arrow);
        }
    }
    layout->addStretch();
    return bar;
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
    propertyDock->setMinimumWidth(350);
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
}

void MainWindow::createStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready | Model: Demo | Camera: Perspective"), this);
    statusBar()->addWidget(m_statusLabel, 1);
}

void MainWindow::connectUi()
{
    connect(m_openAction, &QAction::triggered, this, &MainWindow::openModel);
    connect(m_closeAction, &QAction::triggered, this, &MainWindow::closeModel);
    connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProject);
    connect(m_exitAction, &QAction::triggered, qApp, &QApplication::closeAllWindows);
    connect(m_captureAction, &QAction::triggered, this, &MainWindow::captureImage);

    connect(m_resetCameraAction, &QAction::triggered, this, [this]() {
        m_viewer->resetCamera();
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
    connect(m_projectionAction, &QAction::toggled, this, [this](bool orthographic) {
        m_viewer->setParallelProjection(orthographic);
        m_projectionAction->setText(orthographic ? tr("Perspective") : tr("Orthographic"));
        updateCameraStatus(orthographic);
    });

    connect(m_modelingPanel, &ModelingPanel::generateRequested,
            this, &MainWindow::generateDemoModel);
    connect(m_modelingPanel, &ModelingPanel::applyRequested, this, [this](const DemoModelParameters& parameters) {
        if (m_modelingController->updateDemoModel(parameters)) {
            logInfo(tr("Parameters applied: %1 features")
                        .arg(m_modelingController->features().size()));
        } else {
            logError(tr("Parameter update failed: %1")
                         .arg(m_modelingController->lastError()));
        }
    });
    connect(m_modelingPanel, &ModelingPanel::fitViewRequested, this, [this]() {
        m_viewer->resetCamera();
    });

    connect(m_scenePanel, &ScenePanel::nodeSelected, this, [this](SceneNodeType type) {
        if (type == SceneNodeType::Model) {
            m_propertyPanel->showObjectProperties();
        } else if (type == SceneNodeType::Camera) {
            m_propertyPanel->showCameraProperties();
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
        const QSignalBlocker blocker(m_projectionAction);
        m_projectionAction->setChecked(parameters.parallelProjection);
        m_projectionAction->setText(parameters.parallelProjection ? tr("Perspective") : tr("Orthographic"));
        updateCameraStatus(parameters.parallelProjection);
    });

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    connect(renderPanel, &RenderPanel::backgroundColorChanged,
            m_viewer, &VTKViewer::setBackgroundColor);
    connect(renderPanel, &RenderPanel::lightingChanged,
            m_viewer, &VTKViewer::setLightingEnabled);
    connect(renderPanel, &RenderPanel::captureRequested, this, &MainWindow::captureImage);
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
    QString filePath = QFileDialog::getSaveFileName(
        this, tr("Capture Image"), QStringLiteral("viewport.png"), tr("PNG Image (*.png)"));
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
    const QString prefix = cameraSeparator >= 0 ? current.left(cameraSeparator) : tr("Ready | Model: Demo");
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
        #pipelineBar { background: #202226; border-bottom: 1px solid #454951; }
        #pipelineArrow { color: #777c84; font-size: 20px; }
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
