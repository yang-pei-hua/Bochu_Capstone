#pragma once
#include "app/WorkspaceController.h"
class ModelingController;
class ModelingPanel;
class PropertyPanel;
class VTKViewer;
class QAction;
class FeatureEditorPanel;

struct FeatureEditingUi {
    VTKViewer* viewer;
    ModelingPanel* features;
    PropertyPanel* properties;
    QAction* undo;
    QAction* redo;
};

class FeatureEditingController final : public WorkspaceController
{
    Q_OBJECT
public:
    FeatureEditingController(ModelingController* model, const FeatureEditingUi& ui,
                             QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
private:
    void refresh();
    void updateHistoryActions();
    bool editable() const;
    ModelingController* const m_model;
    const FeatureEditingUi m_ui;
    FeatureEditorPanel* m_editor;
    bool m_modelDisplayed = true;
};
