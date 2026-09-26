#include "app/MainWindow.h"

#include "app/CaptureController.h"
#include "app/ProjectPaths.h"
#include "app/ReconstructionController.h"
#include "core/CameraController.h"
#include "io/ModelLoader.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStatusBar>
#include <QStyle>
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
    createModelingController();
    createCaptureController();
    createReconstructionController();
    connectUi();
    applyDarkTheme();

    m_scenePanel->setScene(m_scene);
    m_propertyPanel->cameraPanel()->setParameters(m_viewer->cameraParameters());

    logInfo(tr("Application started"));
    logInfo(tr("VTK renderer initialized"));

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

    m_viewer->resetCameraToPointCloud();
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
    // A cloud produced elsewhere (or by an earlier reconstruction run) can be
    // inspected without going through the pipeline again.
    m_openPointCloudAction = new QAction(tr("Open Point Cloud..."), this);
    m_openPointCloudAction->setToolTip(tr("Load a .ply point cloud into the viewport"));
    // Capturing the current viewport is the data-generation step of the
    // pipeline, so it sits right next to Open Model in the toolbar.
    m_captureAction = new QAction(tr("Capture Image"), this);
    m_captureAction->setToolTip(tr("Save the current viewport as a PNG"));
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
    setCentralWidget(m_viewer);
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
}

void MainWindow::createStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready | Model: Demo | Camera: Perspective"), this);
    statusBar()->addWidget(m_statusLabel, 1);
}

void MainWindow::connectUi()
{
    connect(m_openAction, &QAction::triggered, this, &MainWindow::openModel);
    connect(m_openPointCloudAction, &QAction::triggered, this, &MainWindow::openPointCloud);
    connect(m_captureAction, &QAction::triggered, this, &MainWindow::captureImage);
    connect(m_closeAction, &QAction::triggered, this, &MainWindow::closeModel);
    connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProject);
    connect(m_exitAction, &QAction::triggered, qApp, &QApplication::closeAllWindows);

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
    m_viewer->resetCameraToPointCloud();
    m_scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(info.fileName()).arg(pointCount));
    m_statusLabel->setText(tr("Ready | Point Cloud: %1 | Camera: %2")
                               .arg(info.fileName(),
                                    m_viewer->cameraParameters().parallelProjection
                                        ? tr("Orthographic") : tr("Perspective")));
    logInfo(tr("Point cloud loaded from %1: %2 points").arg(info.fileName()).arg(pointCount));
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

void MainWindow::captureOrbit(int count, double elevationDeg, double distance)
{
    const RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    const std::size_t before = m_captureController->shots().size();
    if (m_captureController->captureOrbit(count, elevationDeg, distance,
                                          renderPanel->outputWidth(),
                                          renderPanel->outputHeight())) {
        const std::size_t captured = m_captureController->shots().size() - before;
        logInfo(tr("Orbit capture finished: %1 photos at %2 degrees elevation")
                    .arg(captured)
                    .arg(elevationDeg, 0, 'f', 1));
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
