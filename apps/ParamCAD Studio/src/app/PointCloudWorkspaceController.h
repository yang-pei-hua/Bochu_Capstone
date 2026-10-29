#pragma once

#include "app/WorkspaceController.h"
#include <modeling/ModelPatch.h>
#include <reconstruction/analysis/SurfaceAnalysis.h>

class QAction;
class CadReconstructionTask;
struct CadReconstructionResult;
class ModelingController;
class ModelingPanel;
class PointCloudDocument;
class PointCloudRelationsPanel;
class PropertyPanel;
class ReconstructionController;
struct ReconstructRequest;
class ScenePanel;
class QThread;
class VTKViewer;

struct PointCloudWorkspaceUi
{
    VTKViewer* viewer;
    ModelingPanel* modelingPanel;
    PointCloudRelationsPanel* relationsPanel;
    ScenePanel* scenePanel;
    PropertyPanel* propertyPanel;
    QAction* reconstructCadAction;
    QAction* cancelCadAction;
    QAction* reconstructToolAction;
};

// Owns cloud editing/analysis and the UI side of photo and CAD reconstruction.
class PointCloudWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    PointCloudWorkspaceController(PointCloudDocument* document,
                                  ModelingController* modelingController,
                                  ReconstructionController* reconstructionController,
                                  const PointCloudWorkspaceUi& ui,
                                  QWidget* dialogParent, QObject* parent = nullptr);
    ~PointCloudWorkspaceController() override;
    void connectUi();
    void openPointCloud();
    void loadPointCloudFromFile(const QString& filePath);
    void reconstructCad();
    void cancelCadReconstruction();
    void prepareProjectReplacement();

private:
    void updateCadActions();
    void requestGuidedReconstruction();
    void startQueuedReconstruction();
    void onCadReconstructionFinished(const CadReconstructionResult& result);
    void startPointCloudAnalysis(int regionId = 0, bool recordUndo = true);
    void startReconstruction(const ReconstructRequest& request);
    void onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                  const QString& cameraMode, bool dense);

    PointCloudDocument* const m_document;
    ModelingController* const m_modelingController;
    ReconstructionController* const m_reconstructionController;
    const PointCloudWorkspaceUi m_ui;
    QThread* m_analysisThread = nullptr;
    CadReconstructionTask* m_cadTask;
    bool m_startingCad = false;
    bool m_cadOverwriteManual = false;
    bool m_analyseAfterCad = false;
    bool m_guidedQueued = false;
    quint64 m_cadCloudRevision = 0;
    quint64 m_cadBodyRevision = 0;
    modeling::ModelRevision m_cadModelRevision = 0;
    QString m_cadSourcePath;
    std::vector<qint64> m_cadOriginalPointIds;
    std::optional<reconstruction::SurfaceAnalysisResult> m_surfaceAnalysis;
};
