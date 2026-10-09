#include "app/ProjectController.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/ProjectPaths.h"
#include "app/ReconstructionController.h"
#include "io/ProjectStore.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"
#include <QAction>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QScopeGuard>
#include <vtkImageData.h>

ProjectController::ProjectController(ModelingController* model, PointCloudDocument* cloud,
                                     ModelViewController* modelView,
                                     ReconstructionController* reconstruction, const ProjectUi& ui,
                                     QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_model(model), m_cloud(cloud),
      m_modelView(modelView), m_reconstruction(reconstruction), m_ui(ui)
{
    connect(ui.openAction, &QAction::triggered, this, &ProjectController::openProject);
    connect(ui.saveAction, &QAction::triggered, this, &ProjectController::saveProject);
    connect(ui.saveAsAction, &QAction::triggered, this, &ProjectController::saveProjectAs);
}
void ProjectController::updateActions()
{
    for (auto* action : {m_ui.openAction, m_ui.saveAction, m_ui.saveAsAction})
        action->setEnabled(!m_processing && !m_captureBusy);
}
void ProjectController::setCaptureBusy(bool busy)
{
    m_captureBusy = busy;
    updateActions();
}
void ProjectController::reportFailure(const QString& message)
{
    emit logError(message);
    emit statusMessage(message, 5000);
    QMessageBox::warning(m_dialogParent, workspaceTr("Project"), message);
}
void ProjectController::saveProject()
{
    save(false);
}
void ProjectController::saveProjectAs()
{
    save(true);
}
void ProjectController::save(bool choosePath)
{
    if (m_processing || m_captureBusy)
        return;
    m_processing = true;
    updateActions();
    const auto guard = qScopeGuard([this] {
        m_processing = false;
        updateActions();
    });
    QString path = m_projectPath;
    if (choosePath || path.isEmpty()) {
        path = QFileDialog::getSaveFileName(
            m_dialogParent, workspaceTr("Save Project"),
            path.isEmpty() ? QDir(ProjectPaths::outputsRoot()).filePath("project.pcad") : path,
            workspaceTr("ParamCAD Projects (*.pcad)"));
        if (path.isEmpty())
            return;
        if (!path.endsWith(".pcad", Qt::CaseInsensitive))
            path += ".pcad";
    }
    QString error;
    if (!saveTo(path, error))
        reportFailure(workspaceTr("Could not save project: %1").arg(error));
}
void ProjectController::openProject()
{
    if (m_processing || m_captureBusy)
        return;
    m_processing = true;
    updateActions();
    const auto guard = qScopeGuard([this] {
        m_processing = false;
        updateActions();
    });
    const QString path = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("Open Project"),
                                                      m_projectPath.isEmpty()
                                                          ? ProjectPaths::outputsRoot()
                                                          : QFileInfo(m_projectPath).absolutePath(),
                                                      workspaceTr("ParamCAD Projects (*.pcad)"));
    if (path.isEmpty())
        return;
    QString error;
    if (!loadFrom(path, error))
        reportFailure(workspaceTr("Could not open project: %1").arg(error));
}
bool ProjectController::saveTo(const QString& path, QString& error)
{
    if (m_captureBusy) {
        error = workspaceTr("Wait for capture to finish before saving a project.");
        return false;
    }
    ProjectSnapshot snapshot;
    snapshot.features = m_model->features();
    snapshot.currentBody = m_modelView->currentBody();
    snapshot.parametricBody = !snapshot.currentBody.IsNull() && !m_model->bodyShape().IsNull()
        && snapshot.currentBody.IsSame(m_model->bodyShape());
    snapshot.cloud = m_cloud;
    auto& view = snapshot.view;
    view.camera = m_ui.viewer->cameraParameters();
    view.parallelScale = m_ui.viewer->parallelScale();
    view.cloudVisible = m_ui.viewer->pointCloudVisible();
    view.pointSize = m_ui.viewer->pointCloudPointSize();
    view.selectionMode = m_ui.viewer->pointCloudSelectionMode();
    view.datumPlanes = m_ui.datumPlanesAction->isChecked();
    view.originLock = m_ui.properties->cameraPanel()->originLocked();
    view.object = m_ui.viewer->objectPresentation();
    view.render = m_ui.properties->renderPanel()->presentation();
    view.render.textureProjection = m_ui.viewer->textureProjection();
    view.render.textureEnabled = m_ui.viewer->bodyTextureEnabled();
    view.render.texturePath = m_ui.viewer->bodyTexturePath();
    view.render.background = m_ui.viewer->backgroundColor();
    view.render.background2 = m_ui.viewer->backgroundColor2();
    view.render.gradientBackground = m_ui.viewer->gradientBackground();
    if (!ProjectStore::save(path, snapshot, error))
        return false;
    m_projectPath = QFileInfo(path).absoluteFilePath();
    emit projectPathChanged(m_projectPath);
    emit logInfo(workspaceTr("Project saved to %1").arg(m_projectPath));
    emit statusMessage(workspaceTr("Project saved"), 4000);
    return true;
}
bool ProjectController::loadFrom(const QString& path, QString& error)
{
    if (m_captureBusy || m_reconstruction->isRunning()) {
        error =
            workspaceTr("Finish or cancel capture/photo reconstruction before opening a project.");
        return false;
    }
    auto prepared = ProjectStore::load(path, error);
    if (!prepared)
        return false;
    const auto texture = BodyActor::prepareTexture(prepared->texture);
    // Decoding, checksums, geometry rebuild and annotation validation have all
    // succeeded. Commit consists of no-fail in-memory state replacements.
    emit projectAboutToReplace();
    const auto oldAssets = m_assets; // Keep old source files alive throughout publication.
    m_assets = prepared->assets;
    m_model->adoptProject(std::move(prepared->model));
    m_cloud->adoptProject(std::move(*prepared->cloud));
    if (prepared->currentBody.IsNull())
        m_ui.viewer->clearBody();
    else
        m_ui.viewer->setBodyShape(prepared->currentBody);
    auto& v = prepared->view;
    m_ui.properties->setPresentation(v.object);
    m_ui.properties->renderPanel()->setPresentation(v.render);
    m_ui.viewer->setObjectTransform(v.object.position[0], v.object.position[1],
                                    v.object.position[2], v.object.rotation[0],
                                    v.object.rotation[1], v.object.rotation[2], v.object.scale[0],
                                    v.object.scale[1], v.object.scale[2]);
    m_ui.viewer->setObjectVisible(v.object.visible);
    m_ui.viewer->setObjectRepresentation(v.object.representation);
    m_ui.viewer->setObjectOpacity(v.object.opacity);
    m_ui.viewer->setObjectColor(v.object.color);
    m_ui.viewer->clearBodyTexture();
    if (texture)
        m_ui.viewer->setBodyTextureData(texture, v.render.texturePath);
    m_ui.viewer->setTextureProjection(v.render.textureProjection);
    m_ui.viewer->setBodyTextureEnabled(v.render.textureEnabled);
    m_ui.viewer->setBackgroundColor(v.render.background);
    m_ui.viewer->setLightingEnabled(v.render.lighting);
    m_ui.viewer->setBackgroundGradient(v.render.gradientBackground, v.render.background2);
    m_ui.viewer->setPointCloudVisible(v.cloudVisible);
    m_ui.viewer->setPointCloudPointSize(v.pointSize);
    m_ui.viewer->setPointCloudSelectionMode(v.selectionMode);
    m_ui.datumPlanesAction->setChecked(v.datumPlanes);
    m_ui.viewer->setDatumPlanesVisible(v.datumPlanes);
    m_ui.properties->cameraPanel()->setOriginLock(v.originLock);
    m_ui.viewer->applyCamera(v.camera);
    m_ui.viewer->setParallelScale(v.parallelScale);
    m_ui.scene->updateNodeLabel(SceneNodeType::Model, prepared->currentBody.IsNull()
                                                          ? workspaceTr("Model")
                                                          : QFileInfo(path).fileName());
    if (!m_cloud->hasCloud())
        m_ui.scene->updateNodeLabel(SceneNodeType::PointCloud, workspaceTr("PointCloud"));
    m_projectPath = QFileInfo(path).absoluteFilePath();
    emit projectPathChanged(m_projectPath);
    emit logInfo(workspaceTr("Project loaded from %1").arg(m_projectPath));
    emit sceneFramed();
    emit statusTextChanged(workspaceTr("Ready | Project: %1").arg(QFileInfo(path).fileName()));
    return true;
}
