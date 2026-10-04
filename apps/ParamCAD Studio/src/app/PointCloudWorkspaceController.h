#pragma once

#include "app/WorkspaceController.h"

class QAction;
class ModelingController;
class ModelingPanel;
class PointCloudRelationsPanel;
class PropertyPanel;
class ReconstructionController;
struct ReconstructRequest;
class ScenePanel;
class QTabWidget;
class QThread;
class VTKViewer;

struct PointCloudWorkspaceUi
{
    VTKViewer* viewer;
    ModelingPanel* modelingPanel;
    PointCloudRelationsPanel* relationsPanel;
    QTabWidget* structureTabs;
    ScenePanel* scenePanel;
    PropertyPanel* propertyPanel;
    QAction* reconstructCadAction;
    QAction* reconstructToolAction;
};

// Owns cloud editing/analysis and the UI side of photo and CAD reconstruction.
class PointCloudWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    PointCloudWorkspaceController(ModelingController* modelingController,
                                  ReconstructionController* reconstructionController,
                                  const PointCloudWorkspaceUi& ui,
                                  QWidget* dialogParent, QObject* parent = nullptr);
    ~PointCloudWorkspaceController() override;
    void connectUi();
    void openPointCloud();
    void reconstructCad();

private:
    void startPointCloudAnalysis(int regionId = 0);
    void startReconstruction(const ReconstructRequest& request);
    void onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                  const QString& cameraMode, bool dense);

    ModelingController* const m_modelingController;
    ReconstructionController* const m_reconstructionController;
    const PointCloudWorkspaceUi m_ui;
    QString m_pointCloudPath;
    QThread* m_analysisThread = nullptr;
};
