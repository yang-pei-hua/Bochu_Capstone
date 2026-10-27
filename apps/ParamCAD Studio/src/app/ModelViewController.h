#pragma once

#include "app/WorkspaceController.h"
#include "io/ModelLoader.h"

class CaptureController;
class QAction;
class PropertyPanel;
class ScenePanel;
class VTKViewer;

struct ModelViewUi
{
    VTKViewer* viewer;
    PropertyPanel* propertyPanel;
    ScenePanel* scenePanel;
    QAction* exportStepAction;
};

// Owns the current native B-Rep for STEP export, import metadata, presentation
// controls and validation capture.
class ModelViewController final : public WorkspaceController
{
    Q_OBJECT
public:
    ModelViewController(CaptureController* captureController, const ModelViewUi& ui,
                        QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
    void openModel();
    void loadModelFromFile(const QString& filePath);
    void closeModel();
    void deleteModel();
    void exportModel();
    bool exportStepTo(const QString& path, QString& error) const;
    const TopoDS_Shape& currentBody() const { return m_currentBody; }
    void chooseTexture();
    void clearTexture();
    bool runPerformanceCapture(const QString& modelPath, const QString& outputDirectory,
                               const QString& texturePath, QString& errorMessage);

private:
    CaptureController* const m_captureController;
    const ModelViewUi m_ui;
    LoadedModel m_loadedModel;
    TopoDS_Shape m_currentBody;
};
