#pragma once
#include "app/WorkspaceController.h"
#include "core/TextParameterCommand.h"
#include <optional>
class ModelingController;
class ModelingPanel;
class PropertyPanel;
class VTKViewer;
class QAction;
class FeatureEditorPanel;
class TextParameterPanel;

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
    TextParameterPanel* m_textEditor;
    std::optional<textcad::PrimitiveEdit> m_textProposal;
    void invalidateTextPreview();
    bool m_modelDisplayed = true;
};
