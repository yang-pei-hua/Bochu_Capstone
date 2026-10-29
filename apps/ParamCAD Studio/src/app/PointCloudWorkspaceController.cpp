#include "app/PointCloudWorkspaceController.h"
#include "core/PointCloudDocument.h"
#include "app/CadReconstructionTask.h"
#include "app/ModelingController.h"
#include "app/ProjectPaths.h"
#include "app/ReconstructionController.h"
#include "io/PointCloudAdapter.h"
#include "io/PointCloudAnalysis.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/CameraPanel.h"
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
#include <QJsonArray>
#include <QMessageBox>
#include <QSaveFile>
#include <QScopeGuard>
#include <QThread>
#include <QTimer>
#include <map>
#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <utility>

namespace {
reconstruction::LengthUnit cloudUnit(const PointCloudDocument& document)
{
    if (QFileInfo::exists(document.sourcePath()))
        return PointCloudAdapter::lengthUnitFor(document.sourcePath());
    const auto& unit = document.sourceUnit();
    return unit == "mm" ? reconstruction::LengthUnit::Millimeter
        : unit == "m" ? reconstruction::LengthUnit::Meter : reconstruction::LengthUnit::Arbitrary;
}
}

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
        if (!running && m_guidedQueued) {
            m_analyseAfterCad = false;
            QTimer::singleShot(0,this,&PointCloudWorkspaceController::startQueuedReconstruction);
            return;
        }
        if (!running && std::exchange(m_analyseAfterCad, false)) startPointCloudAnalysis(0, false);
    });
    connect(m_cadTask, &CadReconstructionTask::stageChanged, this,
        [this](CadReconstructionTask::Stage stage) {
            QString message;
            switch (stage) {
            case CadReconstructionTask::Stage::Preprocessing:
                message = workspaceTr("正在后台预处理点云…"); break;
            case CadReconstructionTask::Stage::FittingTemplates:
                message = workspaceTr("正在拟合手动模板，并据此继续整体 CAD 重建…"); break;
            case CadReconstructionTask::Stage::Recognizing:
                message = workspaceTr("正在后台识别 CAD 实体…"); break;
            case CadReconstructionTask::Stage::SeparatingParts:
                message = workspaceTr("正在分离部件、恢复有限边界并检查曲面连接与闭合…");break;
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
    connect(m_modelingController, &ModelingController::modelRebuilt, this, [this] {
        if (m_modelingController->hasProtectedReconstruction()) {
            m_guidedQueued = false;
            if (m_cadTask->isRunning()) m_cadTask->cancel();
        }
    });
    connect(m_ui.scenePanel, &ScenePanel::nodeVisibilityRequested, this,
            [this](SceneNodeType type, bool visible) {
        if (type == SceneNodeType::PointCloud) m_ui.viewer->setPointCloudVisible(visible);
    });
    connect(m_ui.scenePanel, &ScenePanel::nodeDeleteRequested, this, [this](SceneNodeType type) {
        if (type != SceneNodeType::PointCloud) return;
        prepareProjectReplacement();
        m_document->clear();
        emit logInfo(workspaceTr("Point cloud deleted"));
    });
    connect(m_ui.viewer, &VTKViewer::pointCloudVisibilityChanged, this, [this](bool visible) {
        m_ui.scenePanel->setNodeState(SceneNodeType::PointCloud, m_document->hasCloud(), visible);
    });
    connect(m_document, &PointCloudDocument::cloudChanged, this, [this] {
        m_guidedQueued = false;
    });
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
    connect(cloudPanel, &PointCloudPanel::pointRequested, this, [this](qint64 id) {
        if (m_document->selection().active(id)) m_document->selectPoint(id);
        else m_ui.relationsPanel->setStatus(workspaceTr("该点 ID 不存在或已删除。"));
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::assignRequested, this,
        [this](const QString& geometry, const QString& name, int target) {
            if (m_document->assignRegion(geometry, name, target)) requestGuidedReconstruction();
        });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::reconstructRequested,
        this, &PointCloudWorkspaceController::reconstructCad);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::unassignRequested, this, [this] {
        m_document->unassignSelection(); requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::saveRequested, this, [this] {
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
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::loadRequested, this, [this] {
        const QString path = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("载入点云标注"),
            QFileInfo(m_document->sourcePath()).absolutePath(), workspaceTr("点云标注 (*.json)"));
        if (path.isEmpty()) return;
        QString error;
        if (!m_document->loadAnnotations(path, error)) {
            QMessageBox::warning(m_dialogParent, workspaceTr("载入点云标注"), error);
            return;
        }
        emit logInfo(workspaceTr("点云标注已载入：%1").arg(path));
        requestGuidedReconstruction();
    });
    refreshCloudPanel();
    const auto refreshRelationships = [this] {
        m_ui.relationsPanel->setSourceInfo(m_document->sourcePath(), m_document->sourceUnit());
        m_ui.relationsPanel->setDocument(m_document->selection(), m_document->hasCloud());
        updateCadActions();
        if (m_document->hasCloud()) m_ui.scenePanel->updateNodeLabel(SceneNodeType::PointCloud,
            workspaceTr("%1 (%2 有效点)").arg(QFileInfo(m_document->sourcePath()).fileName()).arg(m_document->pointCount()));
        else m_ui.scenePanel->updateNodeLabel(SceneNodeType::PointCloud, workspaceTr("PointCloud"));
        m_ui.scenePanel->setNodeState(SceneNodeType::PointCloud, m_document->hasCloud(),
                                     m_ui.viewer->pointCloudVisible());
    };
    connect(m_document, &PointCloudDocument::documentChanged, this, refreshRelationships);
    connect(m_document, &PointCloudDocument::selectionChanged, m_ui.relationsPanel, &PointCloudRelationsPanel::updateSelection);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::regionSelected, this, [this](int id) {
        if (id == 0) m_document->selectUnassigned();
        else m_document->selectRegion(id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::relationSelected, m_document, &PointCloudDocument::selectRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::pointSelected, m_document, &PointCloudDocument::selectPoint);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::editRegionRequested,this,[this](int id,const QString& name,const QString& geometry) {
        if(m_document->editRegion(id,name,geometry)) requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::appendSelectionRequested, this, [this](int id) {
        const auto* region = m_document->selection().region(id);
        if (region && m_document->assignRegion(region->geometry, region->name, id)) requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::replaceSelectionRequested,this,[this](int id) {
        m_document->replaceRegionPoints(id); m_document->lockTemplate(id); requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRegionRequested,this,[this](int id) {
        m_document->removeRegion(id); requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteRegionPointsRequested, this, [this](int id) {
        m_document->deleteRegionPoints(id);
        requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteSelectedPointsRequested, this, [this] {
        m_document->deleteSelectedPoints();
        requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::setRelationRequested, this,
        [this](int id, int first, int second, const QString& type) {
            if (!m_document->setRelation(id, first, second, type))
                m_ui.relationsPanel->setStatus(workspaceTr("关系未保存：端点须为两个非空集合，且不能与现有关系重复。"));
        });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRelationRequested, m_document, &PointCloudDocument::removeRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::undoRequested, this, [this] {
        if (m_document->undo()) requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::detectRequested, this, [this] { startPointCloudAnalysis(); });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::cancelCadRequested,
            m_ui.cancelCadAction, &QAction::trigger);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::refitRequested,this,[this](int id) {
        if(!m_document->selection().region(id)) return;
        m_document->lockTemplate(id); requestGuidedReconstruction();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::inferRequested, this, [this] {
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(), m_ui.relationsPanel->angleTolerance());
        m_ui.relationsPanel->setStatus(workspaceTr("已按当前容差更新自动关系。手动关系与已删除的关系记录保持有效。"));
    });
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
    m_ui.propertyPanel->cameraPanel()->releaseOriginLock();
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

    loadPointCloudFromFile(filePath);
}

void PointCloudWorkspaceController::loadPointCloudFromFile(const QString& filePath)
{

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
    m_ui.propertyPanel->cameraPanel()->releaseOriginLock();
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
    m_ui.viewer->setFocus(Qt::OtherFocusReason);
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
    m_cadOverwriteManual = false;
    if (m_modelingController->hasProtectedReconstruction()) {
        const auto answer = QMessageBox::question(m_dialogParent, workspaceTr("替换手动修改"),
            workspaceTr("当前重建特征包含手动修改。重新拟合会替换这些特征及其尺寸调整。是否继续？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
        m_cadOverwriteManual = true;
    }
    std::vector<vtkIdType> ids;
    PlyCloud cloud = m_document->workingCloud(&ids);
    m_cadOriginalPointIds.assign(ids.begin(),ids.end());
    std::map<vtkIdType, reconstruction::PointId> pointIds;
    for (size_t i = 0; i < ids.size(); ++i) pointIds[ids[i]] = i + 1;
    std::vector<reconstruction::TemplateHint> hints;
    for (const auto& region : m_document->selection().regions()) {
        if (!region.templateLocked || m_document->selection().regionPointCount(region.id) == 0)
            continue;
        reconstruction::TemplateHint hint;
        hint.regionId = region.id;
        const auto& type = region.geometry;
        using Kind = reconstruction::TemplateKind;
        hint.kind = type == "cuboid" ? Kind::Box : type == "sphere" ? Kind::Sphere
            : type == "cylinder" ? Kind::Cylinder : type == "cone" ? Kind::Cone
            : type == "ring" ? Kind::Torus : Kind::Plane;
        for (auto id : m_document->selection().regionPoints(region.id))
            hint.points.push_back(pointIds.at(id));
        hints.push_back(std::move(hint));
    }
    if (cloud.positions.empty()) {
        m_ui.relationsPanel->setStatus(workspaceTr("当前工作点云为空，无法重建 CAD。"));
        return;
    }

    auto sourceUnit = cloudUnit(*m_document);
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
    std::vector<reconstruction::SurfaceRelationHint> relations;
    for(const auto& relation:m_document->selection().relations()) {
        if(relation.automatic) continue;
        reconstruction::SurfaceRelationHint hint;
        bool known=false;
        for(int i=0;i<=static_cast<int>(reconstruction::SurfaceRelationKind::AxisParallelToPlane);++i) {
            const auto kind=static_cast<reconstruction::SurfaceRelationKind>(i);
            if(relation.type==QString::fromLatin1(reconstruction::surfaceRelationName(kind))) {hint.kind=kind;known=true;break;}
        }
        if(!known) continue;
        for(auto id:m_document->selection().regionPoints(relation.first)) if(pointIds.count(id)) hint.firstPoints.push_back(pointIds.at(id));
        for(auto id:m_document->selection().regionPoints(relation.second)) if(pointIds.count(id)) hint.secondPoints.push_back(pointIds.at(id));
        relations.push_back(std::move(hint));
    }
    m_cadTask->start(std::move(cloud), sourceUnit, std::move(hints),
                     m_ui.relationsPanel->surfaceTolerance(), m_surfaceAnalysis,std::move(relations));
    emit logInfo(workspaceTr("仅点云重建：使用观测坐标、法向、单位和用户模板，不读取源模型或面对应表。"));
}

void PointCloudWorkspaceController::requestGuidedReconstruction()
{
    m_analyseAfterCad = false;
    if (m_cadTask->isRunning()) m_cadTask->cancel();
    if (m_analysisThread) m_analysisThread->requestInterruption();
    if (m_modelingController->hasProtectedReconstruction()) {
        m_guidedQueued = false;
        m_ui.relationsPanel->setStatus(workspaceTr("模板已保存；CAD 包含手动修改，自动重建已暂停。点击「重建 CAD」可明确替换。"));
        return;
    }
    if (cloudUnit(*m_document) == reconstruction::LengthUnit::Arbitrary) {
        m_guidedQueued = false;
        m_ui.relationsPanel->setStatus(workspaceTr("模板已保存。点云尺度未知，请点击「重建 CAD」确认尺度后继续。"));
        return;
    }
    m_guidedQueued = true;
    startQueuedReconstruction();
}

void PointCloudWorkspaceController::startQueuedReconstruction()
{
    if (!m_guidedQueued || m_cadTask->isRunning() || m_analysisThread || m_startingCad) return;
    m_guidedQueued = false;
    if (m_modelingController->hasProtectedReconstruction()) return;
    if (m_document->hasCloud()) reconstructCad();
}

void PointCloudWorkspaceController::cancelCadReconstruction()
{
    m_guidedQueued = false;
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
    m_ui.reconstructCadAction->setText(workspaceTr("Reconstruct CAD"));
    m_ui.reconstructCadAction->setToolTip(workspaceTr("仅使用点云的位置、法向和用户模板拟合 CAD，不读取源模型或拓扑对应表。"));
    m_ui.reconstructCadAction->setEnabled(!busy && m_document->hasCloud()
                                         && m_document->pointCount() > 0);
    m_ui.cancelCadAction->setEnabled(running && !m_cadTask->isCancellationRequested());
    m_ui.relationsPanel->setBusy(busy);
    m_ui.relationsPanel->setCadReconstructionBusy(running, m_ui.cancelCadAction->isEnabled());
}

void PointCloudWorkspaceController::onCadReconstructionFinished(const CadReconstructionResult& result)
{
    // Consumers validate fingerprint, units and settings before reusing this.
    if (result.analysis.pointFingerprint) m_surfaceAnalysis = result.analysis;
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
        if (preprocessingReport.normalsEstimated) {
            emit logInfo(workspaceTr("补算法向：已确定方向 %1 个，方向未确定 %2 个。")
                .arg(preprocessingReport.estimatedNormalsOriented)
                .arg(preprocessingReport.normalsWithoutOrientation));
        }
    }
    if(result.assembly && !result.assembly->parts.empty()) {
        std::vector<DetectedPointCloudSurface> groups;size_t closed=0;
        const double scale=cloudUnit(*m_document)==reconstruction::LengthUnit::Meter?.001:1.0;
        const auto vector=[&](modeling::Vec3 p){return QJsonArray{p.x*scale,p.y*scale,p.z*scale};};
        for(const auto& part:result.assembly->parts){DetectedPointCloudSurface group;group.surface.geometry="part";
            group.surface.name=workspaceTr("部件候选 %1 · %2").arg(part.id).arg(part.closed?workspaceTr("已闭合"):workspaceTr("未闭合"));
            group.surface.confidence=part.confidence;group.surface.templateLocked=false;
            group.surface.parameters={{"status",QString::fromStdString(part.status)},{"closed",part.closed},{"lower",vector(part.lower)},
                {"upper",vector(part.upper)},{"open_edges",double(part.openEdges)},{"reason",QString::fromStdString(part.reason)}};
            for(auto id:part.pointIds)if(id>0&&id<=m_cadOriginalPointIds.size())group.points.push_back(vtkIdType(m_cadOriginalPointIds[id-1]));
            if(!group.points.empty())groups.push_back(std::move(group));if(part.closed)++closed;
        }
        m_document->replacePartCandidates(groups,false);
        std::vector<std::array<double,6>> lines;std::vector<bool> closedLines;
        for(const auto& part:result.assembly->parts)for(auto id:part.patchIds)for(const auto& loop:result.assembly->patches[id-1].loops)
            for(size_t i=0;i<loop.size();++i){const auto a=loop[i],b=loop[(i+1)%loop.size()];lines.push_back({a.x*scale,a.y*scale,a.z*scale,b.x*scale,b.y*scale,b.z*scale});closedLines.push_back(part.closed);}
        m_ui.viewer->setAssemblyBoundaries(lines,closedLines);
        emit logInfo(workspaceTr("仅点云分离：%1 个候选，%2 个验证闭合，%3 个保留开放边界；候选不等同于原装配实例。")
            .arg(result.assembly->parts.size()).arg(closed).arg(result.assembly->parts.size()-closed));
    }
    if (result.outcome == CadReconstructionResult::Outcome::Unsupported) {
        QString message = workspaceTr("No supported solid could be reconstructed in %1: %2")
                                    .arg(QFileInfo(m_cadSourcePath).fileName(), result.error);
        if(const auto* region=m_document->selection().region(result.failedRegionId))
            message=workspaceTr("模板「%1」重建失败，模型保持原状：%2").arg(region->name,result.error);
        message += result.assembly?workspaceTr(" 分离结果已显示在点云集合中；未生成新的 CAD，已有模型保持原状。"):
            workspaceTr(" 未生成新的 CAD，已有模型保持原状。");
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
    if (!m_modelingController->commitReconstructedCompound(candidate, m_cadOverwriteManual)) {
        const QString message = m_modelingController->lastError();
        reportStatus(message);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("Reconstruct CAD"), message);
        return;
    }
    // Derived results share the user's edit history instead of adding undo
    // steps that an automatic reconstruction would immediately recreate.
    for (const auto& fit : result.templateFits)
        m_document->updateRegionFit(fit.regionId,
            describeTemplateFit(fit, result.preprocessing.appliedScaleToMillimeters), false);
    if (!result.templateFits.empty()) {
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(),
                                  m_ui.relationsPanel->angleTolerance(), false);
        emit logInfo(workspaceTr("已按 %1 个手动模板约束完成整体重建。原始点归属保持不变。").arg(result.templateFits.size()));
    }

    emit logInfo((result.assembly?workspaceTr("重建结果：%1 个长方体，%2 个棱柱，%3 个通孔，闭合候选所属观测点比例 %4，%5 个解析曲面实体，%6 个有限边界实体"):
                workspaceTr("Reconstructed solid: %1 boxes, %2 polygonal prisms, %3 through holes, surface coverage %4, %5 curved solids, %6 finite shells"))
                .arg(candidate.boxes.size()).arg(candidate.prisms.size())
                .arg(candidate.throughHoles.size()).arg(candidate.surfaceCoverage, 0, 'f', 3).arg(candidate.curvedSolids.size()).arg(candidate.profileBodies.size()));
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
    reportStatus(result.assembly?workspaceTr("已显示部件候选并更新可闭合部分的 CAD；未闭合部分保留在点云集合中。"):
        workspaceTr("CAD 重建完成，已更新参数化模型。"));
    m_analyseAfterCad = !result.assembly && m_document->selection().regionPointCount(0) >= 20;
}

void PointCloudWorkspaceController::startPointCloudAnalysis(int regionId, bool recordUndo)
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
    const auto sourceUnit=cloudUnit(*m_document);
    m_analysisThread = QThread::create([cloud, ids, geometry, tolerance, result, sourceUnit] {
        try { *result = analysePointCloud(cloud, ids, tolerance, geometry,sourceUnit); }
        catch (const std::exception& error) { result->error = QString::fromUtf8(error.what()); }
    });
    auto* thread = m_analysisThread;
    thread->setParent(this);
    updateCadActions();
    connect(thread, &QThread::finished, this, [this, result, revision, regionId, recordUndo, count = cloud.positions.size()] {
        m_analysisThread = nullptr;
        updateCadActions();
        if(m_guidedQueued) { startQueuedReconstruction(); return; }
        if (m_document->revision() != revision) {
            m_ui.relationsPanel->setStatus(workspaceTr("工作点云或关系已被修改，本轮识别结果未应用；可重新识别。")); return;
        }
        if (!result->error.isEmpty()) { m_ui.relationsPanel->setStatus(result->error); emit logWarning(result->error); return; }
        if (!regionId) m_surfaceAnalysis = result->analysis;
        if (result->surfaces.empty()) {
            m_ui.relationsPanel->setStatus(workspaceTr("没有识别到受支持的表面；点保持原有归属，可调整容差或手动分组。")); return;
        }
        if (regionId) {
            const auto best = std::max_element(result->surfaces.begin(), result->surfaces.end(),
                [](const auto& a, const auto& b) { return a.points.size() < b.points.size(); });
            if (best->points.size() < count * 0.7) {
                m_ui.relationsPanel->setStatus(workspaceTr("拟合表面覆盖不足 70%，保留原集合；可修正点归属后重试。")); return;
            }
            m_document->updateRegionFit(regionId, best->surface, recordUndo);
        } else m_document->addSurfaces(result->surfaces, recordUndo);
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(), m_ui.relationsPanel->angleTolerance(), recordUndo);
        m_ui.relationsPanel->setStatus(workspaceTr("识别完成：%1 个表面，归属 %2 / %3 个原始点。未归属点已保留。")
            .arg(result->surfaces.size()).arg(result->assignedPoints).arg(count));
        emit logInfo(workspaceTr("Surface analysis completed: %1 surfaces, %2 source points assigned")
            .arg(result->surfaces.size()).arg(result->assignedPoints));
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    thread->start();
}

