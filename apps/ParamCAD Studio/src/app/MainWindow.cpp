#include "app/MainWindow.h"
#include "app/CaptureController.h"
#include "app/CaptureWorkspaceController.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/ProjectController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/ReconstructionController.h"
#include "app/SketchWorkspaceController.h"
#include "core/CameraController.h"
#include "core/PointCloudDocument.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QFileInfo>
#include <QLabel>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>

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
    createWorkspaceControllers();
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

MainWindow::~MainWindow()
{
    // Workspaces borrow widgets: destroy them (and join analysis) before Qt tears
    // down the window's child widgets and command controllers.
    delete m_pointCloudWorkspace;
    delete m_sketchWorkspace;
    delete m_captureWorkspace;
    delete m_modelViewController;
    // The view borrows the document; release it after workers and before data.
    delete m_viewer;
    delete m_pointCloudDocument;
    delete m_projectController;
}

bool MainWindow::runPerformanceCapture(const QString& modelPath,
                                      const QString& outputDirectory,
                                      const QString& texturePath,
                                      QString& errorMessage)
{
    return m_modelViewController->runPerformanceCapture(
        modelPath, outputDirectory, texturePath, errorMessage);
}

void MainWindow::createWorkspaceControllers()
{
    auto* modeling = new ModelingController(this);
    auto* capture = new CaptureController(m_viewer, this);
    auto* reconstruction = new ReconstructionController(this);

    m_sketchWorkspace = new SketchWorkspaceController(modeling,
        {m_viewer, m_toolPalette, m_modelingPanel, m_propertyPanel, m_sketchPanel,
         m_bottomTabs, m_console, m_newSketchAction, m_datumPlanesAction}, this, this);
    m_captureWorkspace = new CaptureWorkspaceController(capture,
        {m_viewer, m_propertyPanel, m_captureToolAction}, this, this);
    m_propertyPanel->reconstructPanel()->setCaptureBaseDirectory(capture->baseDirectory());
    m_pointCloudWorkspace = new PointCloudWorkspaceController(m_pointCloudDocument, modeling, reconstruction,
        {m_viewer, m_modelingPanel, m_relationsPanel, m_scenePanel,
         m_propertyPanel, m_reconstructCadAction, m_cancelCadAction, m_reconstructToolAction}, this, this);
    m_modelViewController = new ModelViewController(capture,
        {m_viewer, m_propertyPanel, m_scenePanel, m_exportStepAction}, this, this);

    m_projectController = new ProjectController(modeling, m_pointCloudDocument, m_modelViewController,
        reconstruction, {m_viewer, m_propertyPanel, m_scenePanel, m_openProjectAction, m_saveAction,
        m_saveAsAction, m_datumPlanesAction}, this, this);
    connect(m_projectController, &ProjectController::projectAboutToReplace,
            m_sketchWorkspace, &SketchWorkspaceController::resetForProject);
    connect(m_projectController, &ProjectController::projectAboutToReplace,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::prepareProjectReplacement);
    connect(m_projectController, &ProjectController::projectPathChanged, this, [this](const QString& path) {
        setWindowTitle(tr("CAD & Point Cloud Studio — %1").arg(QFileInfo(path).fileName()));
    });
    connect(capture, &CaptureController::batchStateChanged, m_projectController, &ProjectController::setCaptureBusy);

    WorkspaceController* workspaces[] = {
        m_sketchWorkspace, m_captureWorkspace, m_pointCloudWorkspace, m_modelViewController, m_projectController};
    for (auto* workspace : workspaces) {
        connect(workspace, &WorkspaceController::logInfo, this, &MainWindow::logInfo);
        connect(workspace, &WorkspaceController::logWarning, this, &MainWindow::logWarning);
        connect(workspace, &WorkspaceController::logError, this, &MainWindow::logError);
        connect(workspace, &WorkspaceController::statusTextChanged, m_statusLabel, &QLabel::setText);
        connect(workspace, &WorkspaceController::statusMessage, statusBar(), &QStatusBar::showMessage);
        connect(workspace, &WorkspaceController::sceneFramed,
                m_captureWorkspace, &CaptureWorkspaceController::syncCaptureDistance);
    }
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
    m_cancelCadAction = new QAction(tr("Cancel CAD Reconstruction"), this);
    m_cancelCadAction->setObjectName(QStringLiteral("cancelCadReconstructionAction"));
    m_cancelCadAction->setEnabled(false);
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
    m_saveAction->setObjectName(QStringLiteral("saveProjectAction"));
    m_openProjectAction = new QAction(tr("Open Project..."), this);
    m_openProjectAction->setObjectName(QStringLiteral("openProjectAction"));
    m_openProjectAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+O")));
    m_saveAsAction = new QAction(tr("Save Project As..."), this);
    m_saveAsAction->setObjectName(QStringLiteral("saveProjectAsAction"));
    m_saveAsAction->setShortcut(QKeySequence::SaveAs);
    m_exportStepAction = new QAction(tr("Export STEP..."), this);
    m_exportStepAction->setObjectName(QStringLiteral("exportStepAction"));
    m_exportStepAction->setToolTip(tr("Export the current CAD geometry in millimetres"));
    m_exportStepAction->setEnabled(false);
    m_exitAction = new QAction(tr("Exit"), this);
    m_exitAction->setShortcut(QKeySequence::Quit);

    m_resetCameraAction = new QAction(style()->standardIcon(QStyle::SP_BrowserReload), tr("Reset Camera"), this);
    m_frontViewAction = new QAction(tr("Front View"), this);
    m_backViewAction = new QAction(tr("Back View"), this);
    m_leftViewAction = new QAction(tr("Left View"), this);
    m_rightViewAction = new QAction(tr("Right View"), this);
    m_topViewAction = new QAction(tr("Top View"), this);
    m_bottomViewAction = new QAction(tr("Bottom View"), this);

    m_datumPlanesAction = new QAction(tr("Datum Planes"), this);
    m_datumPlanesAction->setCheckable(true);
    // The planes are drawn from the first frame of the 3D view, so the entry
    // starts checked and always reflects what the viewport shows.
    m_datumPlanesAction->setChecked(true);
    m_datumPlanesAction->setToolTip(tr("Show the XY, YZ and XZ datum planes"));

    m_captureToolAction = new QAction(tr("Capture"), this);
    m_captureToolAction->setCheckable(true);
    m_captureToolAction->setToolTip(tr("Show the capture panel in Properties"));

    m_reconstructToolAction = new QAction(tr("Reconstruct Point Cloud"), this);
    m_reconstructToolAction->setCheckable(true);
    m_reconstructToolAction->setToolTip(tr("Reconstruct a point cloud from multi-view images"));
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(m_openAction);
    fileMenu->addAction(m_openPointCloudAction);
    fileMenu->addAction(m_reconstructCadAction);
    fileMenu->addAction(m_cancelCadAction);
    fileMenu->addAction(m_textureAction);
    fileMenu->addAction(m_closeAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_openProjectAction);
    fileMenu->addAction(m_saveAction);
    fileMenu->addAction(m_saveAsAction);
    fileMenu->addAction(m_exportStepAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAction);

    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(m_resetCameraAction);
    viewMenu->addSeparator();
    viewMenu->addActions({m_frontViewAction, m_backViewAction, m_leftViewAction,
                          m_rightViewAction, m_topViewAction, m_bottomViewAction});
    viewMenu->addSeparator();
    viewMenu->addAction(m_datumPlanesAction);

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

    const QStringList reservedStages{tr("Geometry Fitting"), tr("AI Edit")};
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
    m_pointCloudDocument = new PointCloudDocument(this);
    m_viewer = new VTKViewer(this, m_pointCloudDocument);
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

    m_sketchPanel = new SketchPanel(this);
    // Built detached from the tab widget, so it has to stay hidden until a sketch
    // is edited; a child that is never laid out would float over the tab bar.
    m_sketchPanel->hide();

    // The console and the sketch workspace share one panel as two pages, exactly
    // like Properties groups Camera and Render, so the tabs sit on top and no
    // title is repeated above them.
    m_bottomTabs = new QTabWidget(this);
    m_bottomTabs->addTab(m_console, tr("Console / Log"));

    m_bottomDock = new QDockWidget(tr("Output"), this);
    m_bottomDock->setObjectName(QStringLiteral("outputDock"));
    m_bottomDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    m_bottomDock->setWidget(m_bottomTabs);
    m_bottomDock->setMinimumHeight(150);
    // An empty title bar collapses the dock's own header, so the tab row is the
    // only label above the panel and "Output" never appears.
    m_bottomDock->setTitleBarWidget(new QWidget(m_bottomDock));
    addDockWidget(Qt::BottomDockWidgetArea, m_bottomDock);

    resizeDocks({sceneDock, propertyDock}, {235, 380}, Qt::Horizontal);
    resizeDocks({m_bottomDock}, {170}, Qt::Vertical);

    m_modelingPanel = new ModelingPanel(this);
    m_relationsPanel = m_propertyPanel->pointCloudRelationsPanel();
    auto* modelingDock = new QDockWidget(tr("CAD 特征"), this);
    modelingDock->setObjectName(QStringLiteral("modelingDock"));
    modelingDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    modelingDock->setWidget(m_modelingPanel);
    modelingDock->setMinimumWidth(235);
    addDockWidget(Qt::LeftDockWidgetArea, modelingDock);
    splitDockWidget(sceneDock, modelingDock, Qt::Vertical);
}

void MainWindow::createStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready | Model: None | Camera: Perspective"), this);
    statusBar()->addWidget(m_statusLabel, 1);
}

void MainWindow::connectUi()
{
    connect(m_openAction, &QAction::triggered, m_modelViewController, &ModelViewController::openModel);
    connect(m_openPointCloudAction, &QAction::triggered, m_pointCloudWorkspace, &PointCloudWorkspaceController::openPointCloud);
    connect(m_reconstructCadAction, &QAction::triggered, m_pointCloudWorkspace, &PointCloudWorkspaceController::reconstructCad);
    connect(m_cancelCadAction, &QAction::triggered,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::cancelCadReconstruction);
    connect(m_textureAction, &QAction::triggered, m_modelViewController, &ModelViewController::chooseTexture);
    connect(m_captureAction, &QAction::triggered, m_captureWorkspace, &CaptureWorkspaceController::captureImage);
    connect(m_closeAction, &QAction::triggered, m_modelViewController, &ModelViewController::closeModel);
    connect(m_closeAction, &QAction::triggered,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::cancelCadReconstruction);
    connect(m_exportStepAction, &QAction::triggered, m_modelViewController, &ModelViewController::exportModel);
    connect(m_exitAction, &QAction::triggered, qApp, &QApplication::closeAllWindows);

    connect(m_resetCameraAction, &QAction::triggered, this, [this]() {
        m_viewer->resetCamera();
        // Reset Camera is the framing the user has just chosen, so the orbit
        // distance follows it instead of keeping whatever it held before.
        m_captureWorkspace->syncCaptureDistance();
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
    connect(m_datumPlanesAction, &QAction::toggled, this, [this](bool visible) {
        m_viewer->setDatumPlanesVisible(visible);
        m_viewer->renderNow();
        logInfo(visible ? tr("Datum planes shown") : tr("Datum planes hidden"));
    });
    m_sketchWorkspace->connectUi();

    connect(m_scenePanel, &ScenePanel::nodeSelected, this, [this](SceneNodeType type) {
        if (type == SceneNodeType::Model) {
            m_viewer->setPointCloudSelectionMode(0);
            m_propertyPanel->showObjectProperties();
        } else if (type == SceneNodeType::Camera) {
            m_propertyPanel->showCameraProperties();
        } else if (type == SceneNodeType::PointCloud) {
            m_propertyPanel->showPointCloudProperties();
            m_viewer->setPointCloudSelectionMode(1);
        }
    });

    m_pointCloudWorkspace->connectUi();

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
    // A pan has to be allowed to leave the origin behind, so the panel drops its
    // "look at origin" pin rather than pulling the camera back onto it.
    connect(m_viewer, &VTKViewer::cameraPanned, cameraPanel, &CameraPanel::releaseOriginLock);

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    connect(renderPanel, &RenderPanel::backgroundColorChanged,
            m_viewer, &VTKViewer::setBackgroundColor);
    connect(renderPanel, &RenderPanel::lightingChanged,
            m_viewer, &VTKViewer::setLightingEnabled);
    connect(renderPanel, &RenderPanel::captureRequested, m_captureWorkspace, &CaptureWorkspaceController::captureImage);
    m_modelViewController->connectUi();

    m_captureWorkspace->connectUi();
    connect(m_propertyPanel->capturePanel(), &CapturePanel::baseDirectoryChanged,
            m_propertyPanel->reconstructPanel(), &ReconstructPanel::setCaptureBaseDirectory);
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
