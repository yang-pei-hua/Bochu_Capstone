#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "core/PointCloudSelection.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>

PointCloudPanel::PointCloudPanel(QWidget* parent) : QWidget(parent)
{
    auto* selection = new QWidget;
    auto* layout = new QVBoxLayout(selection);
    layout->setContentsMargins(6, 8, 6, 6);
    auto* form = new QFormLayout;
    m_mode = new QComboBox(selection);
    m_mode->setObjectName("pointCloudSelectionMode");
    m_mode->addItems({tr("模型交互"), tr("单点选择"), tr("三维框选")});
    form->addRow(tr("交互方式"), m_mode);
    layout->addLayout(form);

    m_boxTools = new QGroupBox(tr("三维框选"), selection);
    auto* boxLayout = new QVBoxLayout(m_boxTools);
    m_add = new QCheckBox(tr("追加到现有选区"), m_boxTools);
    boxLayout->addWidget(m_add);
    auto* boxRow = new QHBoxLayout;
    m_box = new QPushButton(tr("选取框内点"), m_boxTools);
    m_box->setObjectName("selectCloudBoxButton");
    auto* reset = new QToolButton(m_boxTools);
    reset->setText(tr("重置框"));
    reset->setObjectName("resetCloudBoxButton");
    boxRow->addWidget(m_box, 1); boxRow->addWidget(reset);
    boxLayout->addLayout(boxRow);
    layout->addWidget(m_boxTools);

    m_neighbourTools = new QGroupBox(tr("扩展选区"), selection);
    auto* neighbourLayout = new QFormLayout(m_neighbourTools);
    m_radius = new QDoubleSpinBox(m_neighbourTools);
    m_radius->setObjectName("pointCloudNeighbourRadius");
    m_radius->setDecimals(6); m_radius->setRange(0.000001, 1.0e12); m_radius->setValue(1.0);
    m_radius->setToolTip(tr("使用原始点云坐标单位；每次只向外扩展一轮。"));
    neighbourLayout->addRow(tr("邻域半径"), m_radius);
    m_expand = new QPushButton(tr("扩展一轮"), m_neighbourTools);
    m_expand->setObjectName("expandPointCloudSelection");
    neighbourLayout->addRow(m_expand);
    layout->addWidget(m_neighbourTools);

    auto* pointForm = new QFormLayout;
    m_pointId = new QLineEdit(selection);
    m_pointId->setObjectName("relationshipPointId");
    m_pointId->setPlaceholderText(tr("原始点 ID，从 0 开始；回车定位"));
    pointForm->addRow(tr("查找点"), m_pointId);
    layout->addLayout(pointForm);
    m_selectionTools = new QToolButton(selection);
    m_selectionTools->setObjectName("cloudSelectionToolsButton");
    m_selectionTools->setText(tr("选区操作"));
    m_selectionTools->setPopupMode(QToolButton::InstantPopup);
    layout->addWidget(m_selectionTools, 0, Qt::AlignLeft);

    auto* help = new QGroupBox(tr("操作帮助"), selection);
    help->setCheckable(true); help->setChecked(false);
    auto* helpLayout = new QVBoxLayout(help);
    auto* text = new QLabel(tr("单点：左键选择，Ctrl / Shift + 左键增减点。\n"
        "选择框：拖动面或手柄缩放，拖动框体旋转，Shift + 左键平移。\n"
        "中键旋转视角，右键平移，滚轮缩放。"), help);
    text->setWordWrap(true); text->setVisible(false);
    helpLayout->addWidget(text);
    connect(help, &QGroupBox::toggled, text, &QLabel::setVisible);
    layout->addWidget(help);
    layout->addStretch();
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidget(selection);
    m_relationsPanel = new PointCloudRelationsPanel(this);
    m_relationsPanel->insertSelectionPage(scroll);
    auto* menu = m_relationsPanel->createSelectionMenu(m_selectionTools);
    auto* clear = menu->addAction(tr("清空选区"));
    clear->setObjectName("clearCloudSelectionAction");
    connect(clear, &QAction::triggered, this, &PointCloudPanel::clearRequested);
    m_selectionTools->setMenu(menu);
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0); outer->addWidget(m_relationsPanel);

    connect(m_mode, &QComboBox::currentIndexChanged, this, &PointCloudPanel::modeChanged);
    connect(m_box, &QPushButton::clicked, this, &PointCloudPanel::boxRequested);
    connect(reset, &QToolButton::clicked, this, &PointCloudPanel::resetBoxRequested);
    connect(m_expand, &QPushButton::clicked, this, &PointCloudPanel::expandRequested);
    connect(m_pointId, &QLineEdit::returnPressed, this, [this] {
        bool valid = false;
        const auto id = m_pointId->text().toLongLong(&valid);
        if (valid && id >= 0) emit pointRequested(id);
        else m_relationsPanel->setStatus(tr("请输入从 0 开始的原始点 ID。"));
    });
    setEnabled(false);
}

void PointCloudPanel::refresh(const PointCloudSelection& selection, bool available, int mode)
{
    setEnabled(available);
    const QSignalBlocker blocker(m_mode);
    m_mode->setCurrentIndex(mode);
    m_boxTools->setVisible(mode == 2);
    m_neighbourTools->setVisible(mode != 0);
    m_expand->setEnabled(!selection.selected().empty());
    m_box->setEnabled(mode == 2);
    m_add->setEnabled(mode == 2);
    auto* clear = findChild<QAction*>("clearCloudSelectionAction");
    clear->setEnabled(!selection.selected().empty());
}
double PointCloudPanel::radius() const { return m_radius->value(); }
bool PointCloudPanel::addBoxSelection() const { return m_add->isChecked(); }
void PointCloudPanel::setDefaultRadius(double radius)
{
    m_radius->setValue(radius);
    m_radius->setSingleStep(std::max(0.000001, radius / 10.0));
}
