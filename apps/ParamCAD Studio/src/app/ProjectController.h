#pragma once
#include "app/WorkspaceController.h"
#include <QTemporaryDir>
#include <memory>

class ModelingController;
class ModelViewController;
class PointCloudDocument;
class ReconstructionController;
class PropertyPanel;
class ScenePanel;
class VTKViewer;
class QAction;

struct ProjectUi
{
    VTKViewer* viewer;
    PropertyPanel* properties;
    ScenePanel* scene;
    QAction* openAction;
    QAction* saveAction;
    QAction* saveAsAction;
    QAction* datumPlanesAction;
};

class ProjectController final : public WorkspaceController
{
    Q_OBJECT
  public:
    ProjectController(ModelingController* model, PointCloudDocument* cloud,
                      ModelViewController* modelView, ReconstructionController* reconstruction,
                      const ProjectUi& ui, QWidget* dialogParent, QObject* parent = nullptr);
    void openProject();
    void saveProject();
    void saveProjectAs();
    bool saveTo(const QString& path, QString& error);
    bool loadFrom(const QString& path, QString& error);
    const QString& projectPath() const { return m_projectPath; }
    void setCaptureBusy(bool busy);
  signals:
    void projectAboutToReplace();
    void projectPathChanged(const QString& path);

  private:
    void save(bool choosePath);
    void reportFailure(const QString& message);
    void updateActions();
    ModelingController* const m_model;
    PointCloudDocument* const m_cloud;
    ModelViewController* const m_modelView;
    ReconstructionController* const m_reconstruction;
    const ProjectUi m_ui;
    QString m_projectPath;
    std::shared_ptr<QTemporaryDir> m_assets;
    bool m_processing = false, m_captureBusy = false;
};
