#include "ui/panels/PointCloudPanel.h"
#include "core/PointCloudSelection.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

PointCloudPanel::PointCloudPanel(QWidget* parent) : QWidget(parent)
{
    auto* content = new QWidget(this);
    auto* layout = new QVBoxLayout(content);
    m_count = new QLabel(tr("请先打开点云"), content);
    layout->addWidget(m_count);
    auto* selectionGroup = new QGroupBox(tr("点云选择"), content);
    auto* form = new QFormLayout(selectionGroup);
    m_mode = new QComboBox(content);
    m_mode->setObjectName("pointCloudSelectionMode");
    m_mode->addItems({tr("模型交互"), tr("单点选择"), tr("三维框选")});
    form->addRow(tr("交互方式"), m_mode);
    auto* help = new QLabel(tr("单点：左键选择；Ctrl / Shift + 左键增减点。\n"
        "三维框：左键拖动面/手柄缩放，拖动框体旋转；Shift + 左键平移。\n"
        "中键旋转视角，右键平移视角，滚轮缩放。"), content);
    help->setWordWrap(true);
    form->addRow(help);
    m_add = new QCheckBox(tr("框选追加到现有选区"), content);
    form->addRow(m_add);
    m_box = new QPushButton(tr("选取框内点"), content);
    m_resetBox = new QPushButton(tr("重置选择框"), content);
    auto* boxRow = new QHBoxLayout;
    boxRow->addWidget(m_box); boxRow->addWidget(m_resetBox);
    form->addRow(boxRow);
    m_radius = new QDoubleSpinBox(content);
    m_radius->setObjectName("pointCloudNeighbourRadius");
    m_radius->setDecimals(6);
    m_radius->setRange(0.000001, 1.0e12);
    m_radius->setValue(1.0);
    m_radius->setToolTip(tr("使用当前 PLY 点云的坐标单位。每次只向外扩展一轮。"));
    form->addRow(tr("邻域半径（点云单位）"), m_radius);
    m_expand = new QPushButton(tr("选取附近点 / 向外扩展一轮"), content);
    m_expand->setObjectName("expandPointCloudSelection");
    form->addRow(m_expand);
    m_clear = new QPushButton(tr("清空选区"), content);
    form->addRow(m_clear);
    layout->addWidget(selectionGroup);

    auto* regionGroup = new QGroupBox(tr("模板归属"), content);
    auto* regionForm = new QFormLayout(regionGroup);
    m_geometry = new QComboBox(content);
    m_geometry->addItem(tr("长方体"), "cuboid");
    m_geometry->addItem(tr("球"), "sphere");
    m_geometry->addItem(tr("圆柱"), "cylinder");
    m_geometry->addItem(tr("面"), "plane");
    m_geometry->addItem(tr("环"), "ring");
    m_geometry->addItem(tr("圆锥"), "cone");
    m_name = new QLineEdit(content);
    m_name->setPlaceholderText(tr("例如：顶面、圆柱 1、外环"));
    m_regions = new QComboBox(content);
    m_regions->setObjectName("pointCloudRegions");
    m_regions->addItem(tr("新建分组"), 0);
    regionForm->addRow(tr("目标分组"), m_regions);
    regionForm->addRow(tr("新分组模板"), m_geometry);
    regionForm->addRow(tr("新分组名称"), m_name);
    m_assign = new QPushButton(tr("将选中点归入此分组"), content);
    m_assign->setObjectName("assignPointCloudRegion");
    regionForm->addRow(m_assign);
    m_recall = new QPushButton(tr("选中此分组的点"), content);
    m_remove = new QPushButton(tr("删除此分组"), content);
    auto* regionRow = new QHBoxLayout;
    regionRow->addWidget(m_recall); regionRow->addWidget(m_remove);
    regionForm->addRow(regionRow);
    m_unassign = new QPushButton(tr("移除选中点的归属"), content);
    regionForm->addRow(m_unassign);
    auto* note = new QLabel(tr("选中点显示为橙色，分组用不同颜色显示。每个点属于一个分组；"
        "重新归属会替换旧归属。模板作为标注保存。"), content);
    note->setWordWrap(true);
    regionForm->addRow(note);
    layout->addWidget(regionGroup);
    auto* save = new QPushButton(tr("保存标注…"), content);
    auto* load = new QPushButton(tr("载入标注…"), content);
    auto* files = new QHBoxLayout;
    files->addWidget(save); files->addWidget(load);
    layout->addLayout(files);
    layout->addStretch();
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(content);
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(scroll);

    connect(m_mode, &QComboBox::currentIndexChanged, this, &PointCloudPanel::modeChanged);
    connect(m_box, &QPushButton::clicked, this, &PointCloudPanel::boxRequested);
    connect(m_resetBox, &QPushButton::clicked, this, &PointCloudPanel::resetBoxRequested);
    connect(m_expand, &QPushButton::clicked, this, &PointCloudPanel::expandRequested);
    connect(m_clear, &QPushButton::clicked, this, &PointCloudPanel::clearRequested);
    connect(m_assign, &QPushButton::clicked, this, &PointCloudPanel::assignRequested);
    connect(m_recall, &QPushButton::clicked, this, &PointCloudPanel::recallRequested);
    connect(m_remove, &QPushButton::clicked, this, &PointCloudPanel::removeRegionRequested);
    connect(m_unassign, &QPushButton::clicked, this, &PointCloudPanel::unassignRequested);
    connect(save, &QPushButton::clicked, this, &PointCloudPanel::saveRequested);
    connect(load, &QPushButton::clicked, this, &PointCloudPanel::loadRequested);
    connect(m_regions, &QComboBox::currentIndexChanged, this, [this] {
        const bool existing = regionId() > 0;
        m_geometry->setEnabled(!existing); m_name->setEnabled(!existing);
        m_recall->setEnabled(existing); m_remove->setEnabled(existing);
    });
    setEnabled(false);
}

void PointCloudPanel::refresh(const PointCloudSelection& selection, bool available, int mode)
{
    setEnabled(available);
    m_count->setText(available ? tr("已选 %1 / 有效 %2 点 · %3 个分组")
        .arg(selection.selected().size()).arg(selection.activePointCount()).arg(selection.regions().size())
        : tr("请先打开点云"));
    const QSignalBlocker modeBlocker(m_mode);
    m_mode->setCurrentIndex(mode);
    const int current = regionId();
    const QSignalBlocker regionBlocker(m_regions);
    m_regions->clear();
    m_regions->addItem(tr("新建分组"), 0);
    for (const auto& region : selection.regions()) {
        const int index = m_geometry->findData(region.geometry);
        m_regions->addItem(tr("%1 · %2 (%3 点)").arg(region.name,
            index >= 0 ? m_geometry->itemText(index) : region.geometry)
            .arg(selection.regionPointCount(region.id)), region.id);
    }
    m_regions->setCurrentIndex(std::max(0, m_regions->findData(current)));
    const bool existing = regionId() > 0;
    m_geometry->setEnabled(!existing); m_name->setEnabled(!existing);
    m_recall->setEnabled(existing); m_remove->setEnabled(existing);
    m_box->setEnabled(mode == 2); m_resetBox->setEnabled(mode == 2);
    m_add->setEnabled(mode == 2);
    const bool hasSelection = !selection.selected().empty();
    m_expand->setEnabled(hasSelection); m_clear->setEnabled(hasSelection);
    m_assign->setEnabled(hasSelection); m_unassign->setEnabled(hasSelection);
}

double PointCloudPanel::radius() const { return m_radius->value(); }
bool PointCloudPanel::addBoxSelection() const { return m_add->isChecked(); }
QString PointCloudPanel::geometry() const { return m_geometry->currentData().toString(); }
QString PointCloudPanel::regionName() const { return m_name->text(); }
int PointCloudPanel::regionId() const { return m_regions->currentData().toInt(); }
void PointCloudPanel::setTargetRegion(int id)
{
    m_regions->setCurrentIndex(std::max(0, m_regions->findData(id)));
}
void PointCloudPanel::setDefaultRadius(double radius)
{
    m_radius->setValue(radius);
    m_radius->setSingleStep(std::max(0.000001, radius / 10.0));
}
