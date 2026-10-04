#pragma once

#include "app/WorkspaceController.h"
#include "io/ModelLoader.h"

class CaptureController;
class PropertyPanel;
class ScenePanel;
class VTKViewer;

struct ModelViewUi
{
    VTKViewer* viewer;
    PropertyPanel* propertyPanel;
    ScenePanel* scenePanel;
};

// Keeps imported B-Rep ownership, model presentation and validation capture here.
class ModelViewController final : public WorkspaceController
{
    Q_OBJECT
public:
    ModelViewController(CaptureController* captureController, const ModelViewUi& ui,
                        QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
    void openModel();
    void closeModel();
    void chooseTexture();
    void clearTexture();
    bool runPerformanceCapture(const QString& modelPath, const QString& outputDirectory,
                               const QString& texturePath, QString& errorMessage);

private:
    CaptureController* const m_captureController;
    const ModelViewUi m_ui;
    LoadedModel m_loadedModel;
};
