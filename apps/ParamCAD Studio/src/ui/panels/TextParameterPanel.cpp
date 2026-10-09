#include "ui/panels/TextParameterPanel.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QVBoxLayout>

TextParameterPanel::TextParameterPanel(QWidget* parent) : QWidget(parent) {
    setObjectName("textParameterPanel");
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* group = new QGroupBox(tr("文字修改参数"), this);
    group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    auto* contents = new QVBoxLayout(group);
    auto* help = new QLabel(tr("修改选中特征，或以 #编号 指定目标。未注明单位按 mm；长宽高对应局部轴。"), group);
    help->setWordWrap(true);
    contents->addWidget(help);
    m_input = new QLineEdit(group);
    m_input->setObjectName("textParameterInput");
    // Let the parser report excessive input instead of truncating a pasted
    // sentence into a shorter, accidentally valid command.
    m_input->setMaxLength(4096);
    m_input->setPlaceholderText(tr("高度增加 2 mm，固定中心"));
    m_input->setToolTip(tr("例如：#3 的孔直径改为 8 mm；直径减少 10%，高度改为 3 cm，固定底面。"));
    contents->addWidget(m_input);
    auto* buttons = new QHBoxLayout;
    m_preview = new QPushButton(tr("预览修改"), group);
    m_preview->setObjectName("previewTextParametersButton");
    m_apply = new QPushButton(tr("应用预览"), group);
    m_apply->setObjectName("applyTextParametersButton");
    auto* cancel = new QPushButton(tr("取消"), group);
    cancel->setObjectName("cancelTextParametersButton");
    buttons->addWidget(m_preview); buttons->addWidget(m_apply); buttons->addWidget(cancel);
    contents->addLayout(buttons);
    m_message = new QPlainTextEdit(group);
    m_message->setObjectName("textParameterMessage");
    m_message->setReadOnly(true);
    m_message->setMinimumHeight(120);
    m_message->setMaximumHeight(180);
    m_message->setTabChangesFocus(true);
    contents->addWidget(m_message);
    layout->addWidget(group);
    connect(m_input, &QLineEdit::textChanged, this, [this] {
        invalidatePreview(); emit inputChanged();
    });
    connect(m_input, &QLineEdit::returnPressed, this, &TextParameterPanel::previewRequested);
    connect(m_preview, &QPushButton::clicked, this, &TextParameterPanel::previewRequested);
    connect(m_apply, &QPushButton::clicked, this, &TextParameterPanel::applyRequested);
    connect(cancel, &QPushButton::clicked, this, [this] { m_input->clear(); invalidatePreview(tr("已取消待应用的文字修改。")); });
    invalidatePreview();
}
QString TextParameterPanel::input() const { return m_input->text(); }
void TextParameterPanel::updateControls() {
    m_input->setEnabled(m_allowed);
    m_preview->setEnabled(m_allowed && !m_input->text().trimmed().isEmpty());
    m_apply->setEnabled(m_allowed && m_ready);
}
void TextParameterPanel::setEditingAllowed(bool allowed) {
    m_allowed = allowed;
    if (!allowed) invalidatePreview(tr("请退出草图编辑，并显示当前参数模型后修改。"));
    updateControls();
}
void TextParameterPanel::setPreview(const QString& summary) {
    m_message->setPlainText(summary + tr("\n完整模型重建校验通过；点击“应用预览”提交，可撤销。"));
    m_ready = true; updateControls();
}
void TextParameterPanel::invalidatePreview(const QString& message) {
    m_ready = false;
    m_message->setPlainText(message.isEmpty() ? tr("预览展示修改前后数值与固定参照，应用后才改变模型。") : message);
    updateControls();
}
