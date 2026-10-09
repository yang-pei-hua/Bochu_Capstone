#include "app/FeatureEditingController.h"
#include "app/ModelingController.h"
#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/FeatureEditorPanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/viewport/VTKViewer.h"
#include <QAction>

FeatureEditingController::FeatureEditingController(ModelingController* model,
    const FeatureEditingUi& ui, QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent,parent), m_model(model), m_ui(ui),
      m_editor(ui.properties->featureDetailsPanel()->editor()) {}

bool FeatureEditingController::editable() const
{
    return m_modelDisplayed && !m_ui.viewer->sketchInteractionEnabled();
}

void FeatureEditingController::connectUi()
{
    connect(m_ui.features,&ModelingPanel::featureSelected,this,[this] { refresh(); });
    connect(m_model,&ModelingController::modelRebuilt,this,&FeatureEditingController::refresh);
    connect(m_model,&ModelingController::historyChanged,this,&FeatureEditingController::updateHistoryActions);
    connect(m_ui.viewer,&VTKViewer::sketchInteractionChanged,this,[this] {
        m_editor->setEditingAllowed(editable()); updateHistoryActions();
    });
    connect(m_ui.viewer,&VTKViewer::bodyShapeChanged,this,[this](const TopoDS_Shape& shape) {
        const auto& body=m_model->bodyShape();
        m_modelDisplayed=shape.IsNull() ? body.IsNull() : !body.IsNull() && shape.IsSame(body);
        if (!m_modelDisplayed) m_model->clearHistory();
        m_editor->setEditingAllowed(editable()); updateHistoryActions();
    });
    connect(m_editor,&FeatureEditorPanel::applyRequested,this,
        [this](modeling::FeatureId id,double a,double b,double c,modeling::ResizeAnchor anchor,quint64 revision) {
        m_ui.viewer->endBoxReferenceSelection();
        if (!editable()) { m_editor->setMessage(workspaceTr("请退出草图，并选择当前参数模型中的特征。")); return; }
        if (!m_model->editPrimitive(id,{a,b,c},anchor,revision)) m_editor->setMessage(m_model->lastError());
        else { m_editor->setMessage(workspaceTr("尺寸已更新，可使用 CAD 撤销 / 重做。")); emit logInfo(workspaceTr("特征 #%1 参数已更新").arg(id)); }
    });
    connect(m_editor,&FeatureEditorPanel::selectReferenceRequested,this,[this] {
        if (!editable()) { m_editor->setMessage(workspaceTr("请先退出草图编辑。")); return; }
        for (const auto& f:m_model->features()) if (f.id==m_editor->featureId()) {
            const auto* box=std::get_if<modeling::BoxPrimitiveParams>(&f.params);
            if (box && m_ui.viewer->beginBoxReferenceSelection(*box))
                m_editor->setMessage(workspaceTr("点击彩色参照面或黄色中心。可旋转视图；按 Esc 取消选取。"));
            else m_editor->setMessage(workspaceTr("请先显示模型，并退出草图编辑。"));
            return;
        }
    });
    connect(m_ui.viewer,&VTKViewer::boxReferencePicked,this,[this](int anchor) {
        m_editor->setAnchor(static_cast<modeling::ResizeAnchor>(anchor));
        m_editor->setMessage(workspaceTr("参照已选择；输入尺寸后点击「应用」。"));
    });
    connect(m_ui.viewer,&VTKViewer::boxReferenceSelectionCancelled,this,[this] {
        m_editor->setMessage(workspaceTr("已取消参照选取，当前尺寸尚未提交。"));
    });
    connect(m_editor,&FeatureEditorPanel::cancelRequested,m_ui.viewer,&VTKViewer::endBoxReferenceSelection);
    connect(m_ui.undo,&QAction::triggered,this,[this] { if (editable()) m_model->undo(); });
    connect(m_ui.redo,&QAction::triggered,this,[this] { if (editable()) m_model->redo(); });
    refresh();
}

void FeatureEditingController::refresh()
{
    m_ui.viewer->endBoxReferenceSelection();
    const auto id=m_ui.features->selectedFeatureId();
    const modeling::Feature* selected=nullptr;
    for (const auto& feature:m_model->features()) if (feature.id==id) { selected=&feature; break; }
    m_editor->setFeature(selected,m_model->revision());
    m_editor->setEditingAllowed(editable()); updateHistoryActions();
}

void FeatureEditingController::updateHistoryActions()
{
    m_ui.undo->setEnabled(editable() && m_model->canUndo());
    m_ui.redo->setEnabled(editable() && m_model->canRedo());
}
