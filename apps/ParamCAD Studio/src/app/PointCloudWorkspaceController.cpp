#include "app/PointCloudWorkspaceController.h"
#include "core/PointCloudDocument.h"
#include "app/CadReconstructionTask.h"
#include "app/ModelingController.h"
#include "app/ProjectPaths.h"
#include "app/ReconstructionController.h"
#include "io/PointCloudAdapter.h"
#include "io/PointCloudAnalysis.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QSaveFile>
#include <QScopeGuard>
#include <QTabWidget>
#include <QThread>
#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <utility>

PointCloudWorkspaceController::PointCloudWorkspaceController(
    PointCloudDocument* document, ModelingController* modelingController,
    ReconstructionController* reconstructionController,
    const PointCloudWorkspaceUi& ui, QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_document(document),
      m_modelingController(modelingController),
      m_reconstructionController(reconstructionController), m_ui(ui),
      m_cadTask(new CadReconstructionTask(this))
{
    connect(m_cadTask, &CadReconstructionTask::runningChanged, this, [this](bool running) {
        updateCadActions();
        if (!running && std::exchange(m_analyseAfterCad, false)) startPointCloudAnalysis();
    });
    connect(m_cadTask, &CadReconstructionTask::stageChanged, this,
        [this](CadReconstructionTask::Stage stage) {
            QString message;
            switch (stage) {
            case CadReconstructionTask::Stage::Preprocessing:
                message = workspaceTr("正在后台预处理点云…"); break;
            case CadReconstructionTask::Stage::Recognizing:
                message = workspaceTr("正在后台识别 CAD 实体…"); break;
            case CadReconstructionTask::Stage::Cancelling:
                message = workspaceTr("已请求取消 CAD 重建，等待当前计算步骤结束…"); break;
            }
            m_ui.relationsPanel->setStatus(message);
            m_ui.modelingPanel->setStatusText(message);
            emit statusMessage(message, 0);
            updateCadActions();
        });
    connect(m_cadTask, &CadReconstructionTask::finished,
            this, &PointCloudWorkspaceController::onCadReconstructionFinished);
}

PointCloudWorkspaceController::~PointCloudWorkspaceController()
{
    // Join both workers before the window destroys borrowed views.
    disconnect(m_cadTask, nullptr, this, nullptr);
    delete m_cadTask;
    if (m_analysisThread) {
        disconnect(m_analysisThread, nullptr, this, nullptr);
        m_analysisThread->requestInterruption();
        m_analysisThread->wait();
    }
}

void PointCloudWorkspaceController::connectUi()
{
    PointCloudPanel* cloudPanel = m_ui.propertyPanel->pointCloudPanel();
    const auto refreshCloudPanel = [this, cloudPanel] {
        cloudPanel->refresh(m_document->selection(), m_document->hasCloud(),
                            m_ui.viewer->pointCloudSelectionMode());
    };
    connect(m_ui.viewer, &VTKViewer::pointCloudSelectionChanged, this, refreshCloudPanel);
    connect(m_ui.viewer, &VTKViewer::pointCloudLoaded, this, [this, cloudPanel, refreshCloudPanel] {
        if (m_analysisThread) m_analysisThread->requestInterruption();
        m_ui.relationsPanel->setDefaultTolerances(m_ui.viewer->suggestedPointCloudRadius() / 0.02);
        m_ui.relationsPanel->setStatus(workspaceTr("可识别未归属点，或先在三维视图中手动建立集合。"));
        m_ui.structureTabs->setCurrentWidget(m_ui.relationsPanel);
        cloudPanel->setDefaultRadius(m_ui.viewer->suggestedPointCloudRadius());
        refreshCloudPanel();
        m_ui.propertyPanel->showPointCloudProperties();
    });
    connect(cloudPanel, &PointCloudPanel::modeChanged, m_ui.viewer, &VTKViewer::setPointCloudSelectionMode);
    connect(cloudPanel, &PointCloudPanel::boxRequested, this, [this, cloudPanel] {
        m_ui.viewer->selectPointCloudBox(cloudPanel->addBoxSelection());
    });
    connect(cloudPanel, &PointCloudPanel::resetBoxRequested, m_ui.viewer, &VTKViewer::resetPointCloudBox);
    connect(cloudPanel, &PointCloudPanel::expandRequested, this, [this, cloudPanel] {
        m_document->expandSelection(cloudPanel->radius());
    });
    connect(cloudPanel, &PointCloudPanel::clearRequested, m_document, &PointCloudDocument::clearSelection);
    connect(cloudPanel, &PointCloudPanel::assignRequested, this, [this, cloudPanel] {
        m_document->assignRegion(cloudPanel->geometry(), cloudPanel->regionName(), cloudPanel->regionId());
    });
    connect(cloudPanel, &PointCloudPanel::recallRequested, this, [this, cloudPanel] {
        m_document->selectRegion(cloudPanel->regionId());
    });
    connect(cloudPanel, &PointCloudPanel::removeRegionRequested, this, [this, cloudPanel] {
        m_document->removeRegion(cloudPanel->regionId());
    });
    connect(cloudPanel, &PointCloudPanel::unassignRequested, m_document, &PointCloudDocument::unassignSelection);
    connect(cloudPanel, &PointCloudPanel::saveRequested, this, [this] {
        QString path = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("保存点云标注"),
            m_document->sourcePath() + QStringLiteral(".regions.json"), workspaceTr("点云标注 (*.json)"));
        if (path.isEmpty()) return;
        if (!path.endsWith(".json", Qt::CaseInsensitive)) path += ".json";
        QString error;
        if (!m_document->saveAnnotations(path, error)) {
            QMessageBox::warning(m_dialogParent, workspaceTr("保存点云标注"), error);
            return;
        }
        emit logInfo(workspaceTr("点云标注已保存：%1").arg(path));
    });
    connect(cloudPanel, &PointCloudPanel::loadRequested, this, [this] {
        const QString path = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("载入点云标注"),
            QFileInfo(m_document->sourcePath()).absolutePath(), workspaceTr("点云标注 (*.json)"));
        if (path.isEmpty()) return;
        QString error;
        if (!m_document->loadAnnotations(path, error)) {
            QMessageBox::warning(m_dialogParent, workspaceTr("载入点云标注"), error);
            return;
        }
        emit logInfo(workspaceTr("点云标注已载入：%1").arg(path));
    });
    refreshCloudPanel();
    const auto refreshRelationships = [this] {
        m_ui.relationsPanel->setDocument(m_document->selection(), m_document->hasCloud());
        updateCadActions();
        if (m_document->hasCloud()) m_ui.scenePanel->updateNodeLabel(SceneNodeType::PointCloud,
            workspaceTr("%1 (%2 有效点)").arg(QFileInfo(m_document->sourcePath()).fileName()).arg(m_document->pointCount()));
    };
    connect(m_document, &PointCloudDocument::documentChanged, this, refreshRelationships);
    connect(m_document, &PointCloudDocument::selectionChanged, m_ui.relationsPanel, &PointCloudRelationsPanel::updateSelection);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::regionSelected, this, [this, cloudPanel](int id) {
        if (id == 0) m_document->selectUnassigned();
        else m_document->selectRegion(id);
        cloudPanel->setTargetRegion(id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::relationSelected, m_document, &PointCloudDocument::selectRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::pointSelected, m_document, &PointCloudDocument::selectPoint);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::editRegionRequested, m_document, &PointCloudDocument::editRegion);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::appendSelectionRequested, this, [this](int id) {
        const auto* region = m_document->selection().region(id);
        if (region) m_document->assignRegion(region->geometry, region->name, id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::replaceSelectionRequested, m_document, &PointCloudDocument::replaceRegionPoints);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRegionRequested, m_document, &PointCloudDocument::removeRegion);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteRegionPointsRequested, m_document, &PointCloudDocument::deleteRegionPoints);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteSelectedPointsRequested, m_document, &PointCloudDocument::deleteSelectedPoints);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::setRelationRequested, this,
        [this](int id, int first, int second, const QString& type) {
            if (!m_document->setRelation(id, first, second, type))
                m_ui.relationsPanel->setStatus(workspaceTr("关系未保存：端点须为两个非空集合，且不能与现有关系重复。"));
        });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRelationRequested, m_document, &PointCloudDocument::removeRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::undoRequested, m_document, &PointCloudDocument::undo);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::detectRequested, this, [this] { startPointCloudAnalysis(); });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::cancelCadRequested,
            m_ui.cancelCadAction, &QAction::trigger);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::refitRequested, this, &PointCloudWorkspaceController::startPointCloudAnalysis);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::inferRequested, this, [this] {
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(), m_ui.relationsPanel->angleTolerance());
        m_ui.relationsPanel->setStatus(workspaceTr("已按当前容差更新自动关系。手动关系与已删除的关系记录保持有效。"));
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::saveRequested, cloudPanel, &PointCloudPanel::saveRequested);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::loadRequested, cloudPanel, &PointCloudPanel::loadRequested);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::exportRequested, this, [this] {
        const QFileInfo source(m_document->sourcePath());
        QString path = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("导出当前工作点云"),
            source.dir().filePath(source.completeBaseName() + ".edited.ply"), workspaceTr("点云 (*.ply)"));
        if (path.isEmpty()) return;
        if (!path.endsWith(".ply", Qt::CaseInsensitive)) path += ".ply";
        if (QFileInfo(path).absoluteFilePath().compare(source.absoluteFilePath(), Qt::CaseInsensitive) == 0) {
            m_ui.relationsPanel->setStatus(workspaceTr("请选择新文件名，保留原始点云。")); return;
        }
        QString error;
        if (!m_document->exportCloud(path, error)) { m_ui.relationsPanel->setStatus(error); return; }
        const auto unit = PointCloudAdapter::lengthUnitFor(source.absoluteFilePath());
        QSaveFile metadata(QFileInfo(path).dir().filePath(QFileInfo(path).completeBaseName() + ".json"));
        const auto bytes = QJsonDocument(QJsonObject{{"unit", unit == reconstruction::LengthUnit::Meter ? "m"
            : unit == reconstruction::LengthUnit::Millimeter ? "mm" : "arbitrary"}}).toJson();
        if (!metadata.open(QIODevice::WriteOnly) || metadata.write(bytes) != bytes.size() || !metadata.commit()) {
            m_ui.relationsPanel->setStatus(workspaceTr("点云已导出，但单位信息保存失败：%1").arg(metadata.errorString())); return;
        }
        m_ui.relationsPanel->setStatus(workspaceTr("已导出 %1 个有效点：%2").arg(m_document->pointCount()).arg(path));
    });
    refreshRelationships();

    ReconstructPanel* reconstructPanel = m_ui.propertyPanel->reconstructPanel();
    connect(m_ui.reconstructToolAction, &QAction::toggled, this, [this](bool visible) {
        m_ui.propertyPanel->showReconstructPanel(visible);
    });
    connect(reconstructPanel, &ReconstructPanel::reconstructRequested,
            this, &PointCloudWorkspaceController::startReconstruction);
    connect(reconstructPanel, &ReconstructPanel::cancelRequested,
            m_reconstructionController, &ReconstructionController::cancel);

    connect(m_reconstructionController, &ReconstructionController::started,
            this, [this, reconstructPanel](const QString& runDirectory, const QString& cameraModeHint) {
                reconstructPanel->setStatusText(cameraModeHint);
                emit logInfo(workspaceTr("Reconstruction started in %1").arg(runDirectory));
            });
    connect(m_reconstructionController, &ReconstructionController::progressChanged,
            this, [this, reconstructPanel](int stage, int totalStages, const QString& label) {
                reconstructPanel->setProgress(stage, totalStages);
                reconstructPanel->setStatusText(
                    workspaceTr("Stage %1/%2: %3").arg(stage).arg(totalStages).arg(label));
                emit statusMessage(workspaceTr("Reconstruction %1/%2: %3")
                                             .arg(stage).arg(totalStages).arg(label), 4000);
            });
    connect(m_reconstructionController, &ReconstructionController::logMessage,
            this, &PointCloudWorkspaceController::logInfo);
    connect(m_reconstructionController, &ReconstructionController::failed,
            this, [this, reconstructPanel](const QString& message) {
                reconstructPanel->setBusy(false);
                reconstructPanel->setStatusText(message);
                if (m_reconstructionController->wasCancelled()) {
                    // The user asked for this, so it is a state change rather
                    // than a fault worth interrupting them with a dialog.
                    emit logWarning(message);
                    return;
                }
                emit logError(message);
                QMessageBox::warning(m_dialogParent, workspaceTr("Point Cloud Reconstruction"), message);
            });
    connect(m_reconstructionController, &ReconstructionController::finished,
            this, &PointCloudWorkspaceController::onReconstructionFinished);
}

void PointCloudWorkspaceController::startReconstruction(const ReconstructRequest& request)
{
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    panel->setBusy(true);
    if (!m_reconstructionController->start(request)) {
        panel->setBusy(false);
        const QString error = m_reconstructionController->lastError();
        emit logError(error);
        QMessageBox::warning(m_dialogParent, workspaceTr("Point Cloud Reconstruction"), error);
    }
}

void PointCloudWorkspaceController::onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                          const QString& cameraMode, bool dense)
{
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    panel->setBusy(false);

    QString error;
    if (!m_document->load(pointCloudPath, error)) {
        emit logError(error);
        panel->setStatusText(error);
        QMessageBox::warning(m_dialogParent, workspaceTr("Point Cloud Reconstruction"), error);
        return;
    }

    // The freshly built cloud is now what the recognized-model entry works on.
    updateCadActions();
    m_ui.viewer->resetCameraToPointCloud();
    emit sceneFramed();
    m_ui.scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(QFileInfo(pointCloudPath).fileName()).arg(pointCount));

    panel->setStatusText(workspaceTr("Point cloud loaded: %1 points (%2)")
                             .arg(pointCount)
                             .arg(dense ? workspaceTr("dense") : workspaceTr("sparse")));
    // Fixed wording: the UI automation script asserts on this exact line.
    emit logInfo(workspaceTr("Point Cloud loaded: %1 points").arg(pointCount));

    if (cameraMode == QStringLiteral("estimated")) {
        emit logWarning(workspaceTr("Camera poses were estimated by COLMAP; the point cloud "
                      "scale is arbitrary until a reconstruction with capture "
                      "metadata is available."));
    }
    if (m_cadTask->isRunning() || m_startingCad) m_analyseAfterCad = true;
    else startPointCloudAnalysis();
}

void PointCloudWorkspaceController::openPointCloud()
{
    // A cloud on disk carries no capture metadata, so no scale warning applies;
    // the file is read straight into the same actor the pipeline feeds.
    const QString filePath = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("Open Point Cloud"), ProjectPaths::outputsRoot(),
        workspaceTr("Point Cloud Files (*.ply)"));
    if (filePath.isEmpty()) {
        return;
    }

    QString error;
    if (!m_document->load(filePath, error)) {
        emit logError(error);
        QMessageBox::warning(m_dialogParent, workspaceTr("Open Point Cloud"), error);
        return;
    }

    const QFileInfo info(filePath);
    const int pointCount = m_document->pointCount();
    // A cloud is now in the viewport, so the recognized-model entry becomes
    // usable and works on this file.
    updateCadActions();
    m_ui.viewer->resetCameraToPointCloud();
    emit sceneFramed();
    m_ui.scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(info.fileName()).arg(pointCount));
    emit statusTextChanged(workspaceTr("Ready | Point Cloud: %1 | Camera: %2")
                               .arg(info.fileName(),
                                    m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("Orthographic") : workspaceTr("Perspective")));
    emit logInfo(workspaceTr("Point cloud loaded from %1: %2 points").arg(info.fileName()).arg(pointCount));
}

void PointCloudWorkspaceController::reconstructCad()
{
    if (m_cadTask->isRunning() || m_startingCad || m_analysisThread) return;
    if (m_document->sourcePath().isEmpty()) {
        emit logWarning(workspaceTr("Load a point cloud before reconstructing a model."));
        emit statusMessage(workspaceTr("No point cloud is loaded"), 3500);
        return;
    }

    m_startingCad = true;
    const auto resetStarting = qScopeGuard([this] {
        m_startingCad = false;
        updateCadActions();
        if (!m_cadTask->isRunning() && std::exchange(m_analyseAfterCad, false))
            startPointCloudAnalysis();
    });
    updateCadActions();
    m_cadCloudRevision = m_document->revision();
    m_cadModelRevision = m_modelingController->revision();
    m_cadBodyRevision = m_ui.viewer->bodyRevision();
    m_cadSourcePath = m_document->sourcePath();
    PlyCloud cloud = m_document->workingCloud();
    if (cloud.positions.empty()) {
        m_ui.relationsPanel->setStatus(workspaceTr("当前工作点云为空，无法重建 CAD。"));
        return;
    }

    reconstruction::LengthUnit sourceUnit =
        PointCloudAdapter::lengthUnitFor(m_cadSourcePath);
    if (sourceUnit == reconstruction::LengthUnit::Arbitrary) {
        const QMessageBox::StandardButton choice = QMessageBox::warning(m_dialogParent,
            workspaceTr("Unknown point-cloud scale"),
            workspaceTr("This point cloud has no metric scale metadata. Continuing will "
               "interpret one source unit as one millimetre in the CAD model. "
               "Continue with that explicit assumption?"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (choice != QMessageBox::Yes) {
            return;
        }
        emit logWarning(workspaceTr("Point-cloud scale is unknown; using the user-approved "
                      "assumption 1 source unit = 1 mm"));
        sourceUnit = reconstruction::LengthUnit::Millimeter;
    }

    // A scale confirmation runs a nested event loop; verify the snapshot again
    // before launching in case an import/edit occurred while it was open.
    if (m_document->revision() != m_cadCloudRevision
        || m_modelingController->revision() != m_cadModelRevision
        || m_ui.viewer->bodyRevision() != m_cadBodyRevision) {
        m_ui.relationsPanel->setStatus(workspaceTr("点云或模型已修改，请重新启动 CAD 重建。"));
        return;
    }
    m_analyseAfterCad = false;
    m_cadTask->start(std::move(cloud), sourceUnit);
}

void PointCloudWorkspaceController::cancelCadReconstruction()
{
    m_cadTask->cancel();
}

void PointCloudWorkspaceController::prepareProjectReplacement()
{
    m_analyseAfterCad = false;
    cancelCadReconstruction();
    if (m_analysisThread) m_analysisThread->requestInterruption();
}

void PointCloudWorkspaceController::updateCadActions()
{
    const bool running = m_cadTask->isRunning();
    const bool busy = running || m_startingCad || m_analysisThread;
    m_ui.reconstructCadAction->setEnabled(!busy && m_document->hasCloud()
                                         && m_document->pointCount() > 0);
    m_ui.cancelCadAction->setEnabled(running && !m_cadTask->isCancellationRequested());
    m_ui.relationsPanel->setBusy(busy);
    m_ui.relationsPanel->setCadReconstructionBusy(running, m_ui.cancelCadAction->isEnabled());
}

void PointCloudWorkspaceController::onCadReconstructionFinished(const CadReconstructionResult& result)
{
    const auto reportStatus = [this](const QString& message) {
        m_ui.relationsPanel->setStatus(message);
        m_ui.modelingPanel->setStatusText(message);
        emit statusMessage(message, 5000);
    };
    if (m_document->revision() != m_cadCloudRevision
        || m_modelingController->revision() != m_cadModelRevision
        || m_ui.viewer->bodyRevision() != m_cadBodyRevision) {
        const QString message = workspaceTr("点云或模型已修改，本轮 CAD 重建结果未应用；可重新重建。");
        reportStatus(message);
        emit logWarning(message);
        return;
    }
    if (result.outcome == CadReconstructionResult::Outcome::Cancelled) {
        reportStatus(workspaceTr("CAD 重建已取消，模型保持原状。"));
        emit logInfo(workspaceTr("CAD reconstruction cancelled"));
        return;
    }
    if (result.preprocessingComplete) {
        const auto& preprocessingReport = result.preprocessing;
        emit logInfo(workspaceTr("Point-cloud preprocessing: %1 → %2 points, removed %3 voxel "
                   "duplicates and %4 outliers, estimated %5 normals")
                .arg(preprocessingReport.inputPointCount)
                .arg(preprocessingReport.outputPointCount)
                .arg(preprocessingReport.removedByVoxel)
                .arg(preprocessingReport.removedAsOutliers)
                .arg(preprocessingReport.normalsEstimated));
    }
    if (result.outcome == CadReconstructionResult::Outcome::Unsupported) {
        const QString message = workspaceTr("No supported solid could be reconstructed in %1: %2")
                                    .arg(QFileInfo(m_cadSourcePath).fileName(), result.error);
        reportStatus(message);
        emit logWarning(message);
        return;
    }
    if (result.outcome != CadReconstructionResult::Outcome::Success) {
        const QString message = result.outcome == CadReconstructionResult::Outcome::PreprocessingFailed
            ? workspaceTr("Point-cloud preprocessing failed: %1").arg(result.error)
            : workspaceTr("CAD reconstruction failed: %1").arg(result.error);
        reportStatus(message);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("Reconstruct CAD"), message);
        return;
    }
    const auto& candidate = result.candidate;
    if (!m_modelingController->commitReconstructedCompound(candidate)) {
        const QString message = m_modelingController->lastError();
        reportStatus(message);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("Reconstruct CAD"), message);
        return;
    }

    emit logInfo(workspaceTr("Reconstructed solid: %1 boxes, %2 polygonal prisms, %3 through holes, surface coverage %4, %5 curved solids")
                .arg(candidate.boxes.size()).arg(candidate.prisms.size())
                .arg(candidate.throughHoles.size()).arg(candidate.surfaceCoverage, 0, 'f', 3).arg(candidate.curvedSolids.size()));
    QString curvedName;
    if (!candidate.curvedSolids.empty()) {
        curvedName=std::visit([&](const auto& params)->QString {
            using P=std::decay_t<decltype(params)>;
            if constexpr(std::is_same_v<P,modeling::SpherePrimitiveParams>)return workspaceTr("Sphere");
            else if constexpr(std::is_same_v<P,modeling::CylinderPrimitiveParams>)return workspaceTr("Cylinder");
            else if constexpr(std::is_same_v<P,modeling::ConePrimitiveParams>)return workspaceTr("Cone / Frustum");
            else return workspaceTr("Torus");
        },candidate.curvedSolids.front().primitive);
    }
    for (const reconstruction::ThroughHoleCandidate& hole : candidate.throughHoles) {
        emit logInfo(workspaceTr("Recognized through hole: diameter %1, thickness %2, confidence %3")
                    .arg(hole.primitive.radius * 2.0, 0, 'f', 3)
                    .arg(hole.bodyThickness, 0, 'f', 3)
                    .arg(hole.confidence, 0, 'f', 3));
    }
    emit statusTextChanged(curvedName.isEmpty() ? workspaceTr("Ready | Model: Reconstructed Solid (%1 boxes, %2 prisms) | Camera: %3")
                               .arg(candidate.boxes.size()).arg(candidate.prisms.size())
                               .arg(m_ui.viewer->cameraParameters().parallelProjection
                                         ? workspaceTr("Orthographic") : workspaceTr("Perspective")) :
                          workspaceTr("Ready | Model: %1 | Camera: %2").arg(curvedName).arg(m_ui.viewer->cameraParameters().parallelProjection ? workspaceTr("Orthographic") : workspaceTr("Perspective")));
    reportStatus(workspaceTr("CAD 重建完成，已更新参数化模型。"));
    m_analyseAfterCad = m_document->selection().regionPointCount(0) >= 20;
}

void PointCloudWorkspaceController::startPointCloudAnalysis(int regionId)
{
    if (m_analysisThread || m_startingCad || m_cadTask->isRunning() || !m_document->hasCloud()) return;
    std::vector<vtkIdType> ids;
    const auto cloud = m_document->workingCloud(&ids, regionId == 0, regionId == 0 ? -1 : regionId);
    const auto* region = m_document->selection().region(regionId);
    const QString geometry = region ? region->geometry : QString();
    if (regionId && (!region || geometry == "cuboid")) return;
    if (cloud.positions.size() < 20) { m_ui.relationsPanel->setStatus(workspaceTr("至少需要 20 个有效点进行识别。")); return; }
    const quint64 revision = m_document->revision();
    const double tolerance = m_ui.relationsPanel->surfaceTolerance();
    auto result = std::make_shared<PointCloudAnalysisResult>();
    m_ui.relationsPanel->setStatus(regionId ? workspaceTr("正在后台重拟合集合…") : workspaceTr("正在后台识别表面，并映射回原始点…"));
    m_analysisThread = QThread::create([cloud, ids, geometry, tolerance, result] {
        try { *result = analysePointCloud(cloud, ids, tolerance, geometry); }
        catch (const std::exception& error) { result->error = QString::fromUtf8(error.what()); }
    });
    auto* thread = m_analysisThread;
    thread->setParent(this);
    updateCadActions();
    connect(thread, &QThread::finished, this, [this, result, revision, regionId, count = cloud.positions.size()] {
        m_analysisThread = nullptr;
        updateCadActions();
        if (m_document->revision() != revision) {
            m_ui.relationsPanel->setStatus(workspaceTr("工作点云或关系已被修改，本轮识别结果未应用；可重新识别。")); return;
        }
        if (!result->error.isEmpty()) { m_ui.relationsPanel->setStatus(result->error); emit logWarning(result->error); return; }
        if (result->surfaces.empty()) {
            m_ui.relationsPanel->setStatus(workspaceTr("没有识别到受支持的表面；点保持原有归属，可调整容差或手动分组。")); return;
        }
        if (regionId) {
            const auto best = std::max_element(result->surfaces.begin(), result->surfaces.end(),
                [](const auto& a, const auto& b) { return a.points.size() < b.points.size(); });
            if (best->points.size() < count * 0.7) {
                m_ui.relationsPanel->setStatus(workspaceTr("拟合表面覆盖不足 70%，保留原集合；可修正点归属后重试。")); return;
            }
            m_document->updateRegionFit(regionId, best->surface);
        } else m_document->addSurfaces(result->surfaces);
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(), m_ui.relationsPanel->angleTolerance());
        m_ui.relationsPanel->setStatus(workspaceTr("识别完成：%1 个表面，归属 %2 / %3 个原始点。未归属点已保留。")
            .arg(result->surfaces.size()).arg(result->assignedPoints).arg(count));
        emit logInfo(workspaceTr("Surface analysis completed: %1 surfaces, %2 source points assigned")
            .arg(result->surfaces.size()).arg(result->assignedPoints));
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    thread->start();
}

