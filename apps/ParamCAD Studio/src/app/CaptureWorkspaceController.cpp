#include "app/CaptureWorkspaceController.h"
#include "app/CapturePlan.h"
#include "app/ProjectPaths.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>

CaptureWorkspaceController::CaptureWorkspaceController(
    CaptureController* captureController, const CaptureWorkspaceUi& ui,
    QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_captureController(captureController), m_ui(ui)
{
    auto* panel = m_ui.propertyPanel->capturePanel();
    panel->setBaseDirectory(m_captureController->baseDirectory());
    panel->setDefaultDistance(m_captureController->currentDistance());
}

void CaptureWorkspaceController::connectUi()
{
    CapturePanel* capturePanel = m_ui.propertyPanel->capturePanel();
    connect(m_ui.captureToolAction, &QAction::toggled, this, [this](bool visible) {
        m_ui.propertyPanel->showCapturePanel(visible);
    });
    connect(capturePanel, &CapturePanel::baseDirectoryChanged,
            m_captureController, &CaptureController::setBaseDirectory);
    connect(capturePanel, &CapturePanel::capturePhotoRequested,
            this, &CaptureWorkspaceController::capturePhoto);
    connect(capturePanel, &CapturePanel::captureOrbitRequested,
            this, &CaptureWorkspaceController::captureOrbit);

    connect(m_captureController, &CaptureController::shotsChanged, this, [this, capturePanel]() {
        capturePanel->setShots(m_captureController->shots());
    });
    connect(m_captureController, &CaptureController::progressChanged,
            this, [this, capturePanel](const QString& message) {
                capturePanel->setStatusText(message);
                emit statusMessage(message, 4000);
            });
    connect(m_captureController, &CaptureController::batchStateChanged,
            this, [this, capturePanel](bool running) {
                capturePanel->setBusy(running);
                // The orbit sweep drives the camera directly, so the panel must
                // stop echoing it back while the batch owns the pose.
                m_ui.propertyPanel->cameraPanel()->setFeedbackSuppressed(running);
                if (running) {
                    capturePanel->setStatusText(workspaceTr("Capturing orbit..."));
                } else {
                    // The pose was restored while feedback was still suppressed,
                    // so re-read the live camera instead of trusting the stale
                    // readout to happen to match.
                    m_ui.propertyPanel->cameraPanel()->setParameters(m_ui.viewer->cameraParameters());
                }
            });
    connect(m_captureController, &CaptureController::captureFailed,
            this, [this](const QString& message) {
                emit logError(message);
                QMessageBox::warning(m_dialogParent, workspaceTr("Capture"), message);
            });
}

void CaptureWorkspaceController::captureImage()
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

    QString filePath = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("Capture Image"), defaultPath, workspaceTr("PNG Image (*.png)"));
    if (filePath.isEmpty()) {
        return;
    }
    if (!filePath.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)) {
        filePath += QStringLiteral(".png");
    }

    RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    if (m_ui.viewer->captureImage(filePath, renderPanel->outputWidth(), renderPanel->outputHeight())) {
        emit logInfo(workspaceTr("Image captured: %1").arg(QFileInfo(filePath).fileName()));
        emit statusMessage(workspaceTr("Image saved to %1").arg(filePath), 5000);
    } else {
        emit logError(workspaceTr("Failed to capture image: %1").arg(filePath));
        QMessageBox::warning(m_dialogParent, workspaceTr("Capture Image"), workspaceTr("The image could not be saved."));
    }
}

void CaptureWorkspaceController::capturePhoto()
{
    const RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    if (m_captureController->captureSingle(renderPanel->outputWidth(),
                                           renderPanel->outputHeight())) {
        const CaptureShot& shot = m_captureController->shots().back();
        emit logInfo(workspaceTr("Photo captured: %1/%2").arg(shot.groupName, shot.imageName));
    }
}

void CaptureWorkspaceController::captureOrbit(double distance)
{
    // Five latitude rings at 30-degree longitude steps cover the body, while
    // one canonical pose at each pole avoids twelve duplicate images there.
    const std::vector<OrbitRing> rings = sphericalCaptureRings();

    const RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    const std::size_t before = m_captureController->shots().size();
    if (m_captureController->captureOrbit(rings, distance,
                                          renderPanel->outputWidth(),
                                          renderPanel->outputHeight())) {
        const std::size_t captured = m_captureController->shots().size() - before;
        emit logInfo(workspaceTr("Spherical capture finished: %1 photos on a 30-degree grid")
                    .arg(captured));
    }
}

void CaptureWorkspaceController::syncCaptureDistance()
{
    m_ui.propertyPanel->capturePanel()->setDefaultDistance(
        m_captureController->currentDistance());
}

