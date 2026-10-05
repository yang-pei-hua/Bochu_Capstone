#include "ui/panels/PointCloudRelationsPanel.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>
#include <vtkPolyData.h>

#include <algorithm>
#include <set>

namespace {
enum Kind { Category = 0, Region = 1, Relation = 2, PointList = 3, Point = 4, More = 5 };
constexpr int KindRole = Qt::UserRole, IdRole = Qt::UserRole + 1, OffsetRole = Qt::UserRole + 2;
constexpr int PageSize = 200;
const std::vector<std::pair<QString, QString>> geometries{
    {"cuboid", "长方体集合"}, {"plane", "平面"}, {"sphere", "球面"},
    {"cylinder", "圆柱面"}, {"cone", "圆锥面"}, {"ring", "环面"}};
const std::vector<std::pair<QString, QString>> relationTypes{
    {"parallel", "平行"}, {"perpendicular", "垂直"}, {"adjacent", "邻近 / 相邻"},
    {"intersect", "相交"}, {"coaxial", "同轴"}, {"tangent", "相切"},
    {"same_radius", "等半径"}, {"coplanar", "共面"}};
QString label(const std::vector<std::pair<QString, QString>>& values, const QString& key)
{
    for (const auto& [value, text] : values) if (value == key) return text;
    return key;
}
QTreeWidgetItem* node(QTreeWidgetItem* parent, const QStringList& labels, int kind, qint64 id)
{
    auto* item = new QTreeWidgetItem(parent, labels);
    item->setData(0, KindRole, kind); item->setData(0, IdRole, id);
    for (int column = 0; column < labels.size(); ++column)
        item->setToolTip(column, labels.join(QStringLiteral(" · ")));
    return item;
}
QString itemKey(QTreeWidgetItem* item)
{
    return item ? QStringLiteral("%1:%2:%3").arg(item->data(0, KindRole).toInt())
        .arg(item->data(0, IdRole).toLongLong()).arg(item->text(0)) : QString();
}
QDoubleSpinBox* tolerance(QWidget* parent, double value)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setDecimals(6); spin->setRange(0.000001, 1.0e12); spin->setValue(value);
    return spin;
}
}

PointCloudRelationsPanel::PointCloudRelationsPanel(QWidget* parent) : QWidget(parent)
{
    setObjectName("pointCloudRelationsPanel");
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6); layout->setSpacing(5);
    m_summary = new QLabel(tr("请先打开点云"), this); m_summary->setWordWrap(true);
    layout->addWidget(m_summary);
    m_filter = new QLineEdit(this); m_filter->setPlaceholderText(tr("搜索集合名称、模板或关系"));
    layout->addWidget(m_filter);
    m_tree = new QTreeWidget(this);
    m_tree->setObjectName("pointCloudRelationshipTree");
    m_tree->setHeaderLabels({tr("集合 / 关系"), tr("类型 / 来源"), tr("点数")});
    m_tree->setMinimumHeight(220); m_tree->setUniformRowHeights(true);
    m_tree->setIndentation(14);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->header()->setStretchLastSection(false);
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Interactive);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::Interactive);
    m_tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    m_tree->setColumnWidth(0, 190);
    m_tree->setColumnWidth(1, 100);
    layout->addWidget(m_tree, 1);
    m_selection = new QLabel(this); m_selection->setWordWrap(true);
    m_selection->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_selection);
    auto* pointRow = new QHBoxLayout;
    m_pointId = new QLineEdit(this); m_pointId->setObjectName("relationshipPointId");
    m_pointId->setPlaceholderText(tr("原始点 ID（从 0 开始）"));
    auto* locate = new QPushButton(tr("查看点"), this); locate->setObjectName("relationshipLocatePoint");
    pointRow->addWidget(m_pointId, 1); pointRow->addWidget(locate); layout->addLayout(pointRow);

    auto* operations = new QTabWidget(this);
    auto* regions = new QWidget(operations);
    auto* regionLayout = new QVBoxLayout(regions); regionLayout->setContentsMargins(4, 4, 4, 4);
    const auto buttons = [&](QVBoxLayout* target, QPushButton*& first, const QString& firstText,
                              QPushButton*& second, const QString& secondText) {
        auto* row = new QHBoxLayout;
        first = new QPushButton(firstText, this); second = new QPushButton(secondText, this);
        row->addWidget(first); row->addWidget(second); target->addLayout(row);
    };
    buttons(regionLayout, m_editRegion, tr("编辑集合"), m_refit, tr("按此模板重建整体"));
    m_refit->setObjectName("rebuildSelectedTemplateButton");
    buttons(regionLayout, m_append, tr("加入当前选区"), m_replace, tr("以选区替换集合"));
    buttons(regionLayout, m_removeRegion, tr("删除集合"), m_deleteRegionPoints, tr("删除集合内所有点"));
    m_deleteRegionPoints->setObjectName("deleteRegionPointsButton");
    m_removeRegion->setObjectName("removePointCollectionButton");
    m_removeRegion->setToolTip(tr("移除集合及其关系，保留点并解除归属。"));
    m_deleteRegionPoints->setToolTip(tr("从工作点云移除集合内所有点，可撤销。后续拟合和导出将排除这些点。"));
    m_deleteSelected = new QPushButton(tr("删除当前选中的点"), this);
    m_deleteSelected->setObjectName("deleteSelectedCloudPointsButton"); regionLayout->addWidget(m_deleteSelected);
    operations->addTab(regions, tr("集合"));
    auto* relations = new QWidget(operations);
    auto* relationLayout = new QVBoxLayout(relations); relationLayout->setContentsMargins(4, 4, 4, 4);
    buttons(relationLayout, m_addRelation, tr("添加关系"), m_editRelation, tr("编辑关系"));
    m_removeRelation = new QPushButton(tr("删除关系（保留点和集合）"), this);
    m_removeRelation->setObjectName("removeCloudRelationButton"); relationLayout->addWidget(m_removeRelation);
    auto* note = new QLabel(tr("自动关系是按容差推断的几何证据；邻近关系来自点集距离。"
        "删除的关系在更新时保持删除。手动关系作为标注保存。"), this);
    note->setWordWrap(true); relationLayout->addWidget(note);
    operations->addTab(relations, tr("关系"));
    auto* analysis = new QWidget(operations);
    auto* form = new QFormLayout(analysis); form->setContentsMargins(4, 4, 4, 4);
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    m_surfaceTolerance = tolerance(this, 0.1); m_surfaceTolerance->setObjectName("surfaceFitTolerance");
    m_relationTolerance = tolerance(this, 1.0);
    m_angleTolerance = tolerance(this, 2.0); m_angleTolerance->setRange(0.01, 30); m_angleTolerance->setDecimals(2);
    m_angleTolerance->setSuffix(QStringLiteral("°"));
    form->addRow(tr("拟合距离容差"), m_surfaceTolerance);
    form->addRow(tr("关系距离容差"), m_relationTolerance);
    form->addRow(tr("关系角度容差"), m_angleTolerance);
    auto* units = new QLabel(tr("距离使用原始点云的坐标单位。未识别点保留在未归属集合。"), this);
    units->setWordWrap(true); form->addRow(units);
    operations->addTab(analysis, tr("分析设置"));
    layout->addWidget(operations);
    auto* analysisButtons = new QHBoxLayout;
    m_detect = new QPushButton(tr("识别未归属点"), this); m_detect->setObjectName("detectCloudSurfacesButton");
    m_infer = new QPushButton(tr("更新关系"), this); m_infer->setObjectName("inferCloudRelationsButton");
    analysisButtons->addWidget(m_detect); analysisButtons->addWidget(m_infer); layout->addLayout(analysisButtons);
    m_cancelCad = new QPushButton(tr("取消 CAD 重建"), this);
    m_cancelCad->setObjectName("cancelCadReconstructionButton");
    m_cancelCad->hide();
    layout->addWidget(m_cancelCad);
    connect(m_cancelCad, &QPushButton::clicked, this, &PointCloudRelationsPanel::cancelCadRequested);
    auto* files = new QHBoxLayout;
    m_undo = new QPushButton(tr("撤销"), this); m_undo->setObjectName("undoCloudEditButton");
    auto* save = new QPushButton(tr("保存关系…"), this);
    auto* load = new QPushButton(tr("载入…"), this);
    files->addWidget(m_undo); files->addWidget(save); files->addWidget(load); layout->addLayout(files);
    auto* exportCloud = new QPushButton(tr("导出当前工作点云…"), this); layout->addWidget(exportCloud);
    m_status = new QLabel(this); m_status->setObjectName("pointCloudRelationStatus");
    m_status->setWordWrap(true); layout->addWidget(m_status);

    connect(m_filter, &QLineEdit::textChanged, this, &PointCloudRelationsPanel::applyFilter);
    connect(m_tree, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* item) {
        updateActions();
        if (!item) return;
        const int kind = item->data(0, KindRole).toInt();
        const auto id = item->data(0, IdRole).toLongLong();
        if (kind == Region || kind == PointList) emit regionSelected(static_cast<int>(id));
        if (kind == Relation) emit relationSelected(static_cast<int>(id));
        if (kind == Point) emit pointSelected(static_cast<vtkIdType>(id));
    });
    connect(m_tree, &QTreeWidget::itemExpanded, this, [this](QTreeWidgetItem* item) {
        if (item->data(0, KindRole).toInt() == PointList && item->childCount() == 0) loadPointPage(item, 0);
    });
    connect(m_tree, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item) {
        if (item->data(0, KindRole).toInt() != More) return;
        auto* parent = item->parent();
        const int offset = item->data(0, OffsetRole).toInt();
        delete parent->takeChild(parent->indexOfChild(item));
        loadPointPage(parent, offset);
    });
    const auto locatePoint = [this] {
        if (!m_document) return;
        bool valid = false; const auto id = m_pointId->text().toLongLong(&valid);
        if (!valid || !m_document->active(static_cast<vtkIdType>(id))) {
            setStatus(tr("该点 ID 不存在或已删除。")); return;
        }
        emit pointSelected(static_cast<vtkIdType>(id));
    };
    connect(locate, &QPushButton::clicked, this, locatePoint);
    connect(m_pointId, &QLineEdit::returnPressed, this, locatePoint);
    connect(m_editRegion, &QPushButton::clicked, this, &PointCloudRelationsPanel::editRegion);
    connect(m_refit, &QPushButton::clicked, this, [this] { emit refitRequested(selectedRegion()); });
    connect(m_append, &QPushButton::clicked, this, [this] { emit appendSelectionRequested(selectedRegion()); });
    connect(m_replace, &QPushButton::clicked, this, [this] { emit replaceSelectionRequested(selectedRegion()); });
    connect(m_removeRegion, &QPushButton::clicked, this, [this] { emit removeRegionRequested(selectedRegion()); });
    connect(m_deleteRegionPoints, &QPushButton::clicked, this, [this] { emit deleteRegionPointsRequested(selectedRegion()); });
    connect(m_deleteSelected, &QPushButton::clicked, this, &PointCloudRelationsPanel::deleteSelectedPointsRequested);
    connect(m_addRelation, &QPushButton::clicked, this, [this] { editRelation(true); });
    connect(m_editRelation, &QPushButton::clicked, this, [this] { editRelation(false); });
    connect(m_removeRelation, &QPushButton::clicked, this, [this] { emit removeRelationRequested(selectedRelation()); });
    connect(m_detect, &QPushButton::clicked, this, &PointCloudRelationsPanel::detectRequested);
    connect(m_infer, &QPushButton::clicked, this, &PointCloudRelationsPanel::inferRequested);
    connect(m_undo, &QPushButton::clicked, this, &PointCloudRelationsPanel::undoRequested);
    connect(save, &QPushButton::clicked, this, &PointCloudRelationsPanel::saveRequested);
    connect(load, &QPushButton::clicked, this, &PointCloudRelationsPanel::loadRequested);
    connect(exportCloud, &QPushButton::clicked, this, &PointCloudRelationsPanel::exportRequested);
    setEnabled(false);
}

void PointCloudRelationsPanel::setDocument(const PointCloudSelection& document, bool available)
{
    m_document = &document; setEnabled(available);
    const QString previous = itemKey(m_tree->currentItem());
    std::set<QString> expanded;
    QTreeWidgetItemIterator before(m_tree);
    while (*before) { if ((*before)->isExpanded()) expanded.insert(itemKey(*before)); ++before; }
    const QSignalBlocker blocker(m_tree);
    m_tree->clear(); m_points.clear();
    for (size_t i = 0; i < document.memberships().size(); ++i) {
        if (document.active(static_cast<vtkIdType>(i))) m_points[document.memberships()[i]].push_back(static_cast<vtkIdType>(i));
    }
    m_summary->setText(available ? tr("有效 %1 / 原始 %2 点 · 已删除 %3\n%4 个集合 · %5 条关系")
        .arg(document.activePointCount()).arg(document.memberships().size())
        .arg(document.memberships().size()-static_cast<size_t>(document.activePointCount()))
        .arg(document.regions().size()).arg(document.relations().size()) : tr("请先打开点云"));
    auto* collections = new QTreeWidgetItem(m_tree, {tr("表面 / 点集合"), QString(), QString()});
    auto* relations = new QTreeWidgetItem(m_tree, {tr("关系列表"), QString(), QString()});
    collections->setData(0, IdRole, -1); relations->setData(0, IdRole, -2);
    const auto relationNode = [&](QTreeWidgetItem* parent, const PointCloudRelation& relation) {
        const auto* first = document.region(relation.first); const auto* second = document.region(relation.second);
        if (!first || !second) return;
        auto* item = node(parent, {QStringLiteral("%1 ↔ %2").arg(first->name, second->name),
            label(relationTypes, relation.type), QString()}, Relation, relation.id);
        item->setToolTip(0, tr("%1 · 置信度 %2\n点击同时高亮两个集合")
            .arg(relation.automatic ? tr("按容差自动推断") : tr("手动标注"))
            .arg(relation.confidence, 0, 'f', 3));
        node(item, {relation.automatic ? tr("来源：自动推断") : tr("来源：手动标注"),
            tr("置信度 %1").arg(relation.confidence, 0, 'f', 3)}, Relation, relation.id);
    };
    for (const auto& region : document.regions()) {
        auto* item = node(collections, {region.name, label(geometries, region.geometry),
            QString::number(m_points[region.id].size())}, Region, region.id);
        item->setToolTip(0, tr("集合 #%1 · %2").arg(region.id).arg(region.name));
        node(item, {tr("拟合状态"), region.fitStale ? tr("点集已修改，需重新拟合")
            : region.parameters.isEmpty() ? tr("手动集合，尚未拟合") : tr("已拟合")}, Region, region.id);
        if (!region.parameters.isEmpty()) {
            node(item, {tr("平均残差 / 置信度"), QStringLiteral("%1 / %2").arg(region.residual, 0, 'g', 6)
                .arg(region.confidence, 0, 'f', 3)}, Region, region.id);
            const std::map<QString, QString> names{{"normal", "法向"}, {"offset", "平面偏移"}, {"center", "中心 / 轴点"},
                {"axis", "轴方向"}, {"radius", "半径"}, {"major_radius", "主半径"}, {"minor_radius", "管半径"},
                {"angle", "半角（弧度）"}, {"axial_min", "轴向下界"}, {"axial_max", "轴向上界"}};
            for (auto param = region.parameters.begin(); param != region.parameters.end(); ++param) {
                QString text;
                if (param.value().isArray()) {
                    QStringList components; for (const auto& component : param.value().toArray()) components << QString::number(component.toDouble(), 'g', 6);
                    text = components.join(", ");
                } else text = QString::number(param.value().toDouble(), 'g', 6);
                const auto found = names.find(param.key());
                node(item, {found == names.end() ? param.key() : found->second, text}, Region, region.id);
            }
        }
        auto* points = node(item, {tr("所属点（点击展开，每页 200 点）"), QString(), QString::number(m_points[region.id].size())}, PointList, region.id);
        points->setChildIndicatorPolicy(QTreeWidgetItem::ShowIndicator);
        auto* linked = node(item, {tr("与其他集合的关系")}, Category, region.id);
        for (const auto& relation : document.relations()) {
            if (relation.first == region.id || relation.second == region.id) relationNode(linked, relation);
        }
    }
    auto* unassigned = node(collections, {tr("未归属点"), tr("尚无集合归属"), QString::number(m_points[0].size())}, Region, 0);
    auto* unassignedPoints = node(unassigned, {tr("所属点（点击展开，每页 200 点）")}, PointList, 0);
    unassignedPoints->setChildIndicatorPolicy(QTreeWidgetItem::ShowIndicator);
    for (const auto& relation : document.relations()) relationNode(relations, relation);
    collections->setExpanded(true); relations->setExpanded(true);
    // Restoring expansion is explicit because the tree's signals are blocked.
    QTreeWidgetItemIterator after(m_tree);
    while (*after) {
        auto* item = *after;
        if (expanded.count(itemKey(item))) {
            if (item->data(0, KindRole).toInt() == PointList) loadPointPage(item, 0);
            item->setExpanded(true);
        }
        if (itemKey(item) == previous) m_tree->setCurrentItem(item);
        ++after;
    }
    applyFilter(); updateSelection(); updateActions();
}

void PointCloudRelationsPanel::loadPointPage(QTreeWidgetItem* item, int offset)
{
    if (!m_document) return;
    const auto& ids = m_points[item->data(0, IdRole).toInt()];
    const int end = std::min(static_cast<int>(ids.size()), offset + PageSize);
    for (int i = offset; i < end; ++i) {
        const vtkIdType id = ids[static_cast<size_t>(i)]; double p[3]; m_document->data()->GetPoint(id, p);
        auto* point = node(item, {tr("点 #%1").arg(id), QStringLiteral("%1, %2, %3")
            .arg(p[0], 0, 'g', 5).arg(p[1], 0, 'g', 5).arg(p[2], 0, 'g', 5)}, Point, id);
        point->setToolTip(0, tr("原始 PLY 点 ID：%1（从 0 开始）").arg(id));
    }
    if (end < static_cast<int>(ids.size())) {
        auto* more = node(item, {tr("加载下一页…（还剩 %1 点）").arg(ids.size()-static_cast<size_t>(end))}, More, 0);
        more->setData(0, OffsetRole, end);
    }
}

int PointCloudRelationsPanel::selectedRegion() const
{
    auto* item = m_tree->currentItem();
    while (item) {
        const int kind = item->data(0, KindRole).toInt();
        if (kind == Region || kind == PointList) return item->data(0, IdRole).toInt();
        if (kind == Relation) return -1;
        item = item->parent();
    }
    return -1;
}
int PointCloudRelationsPanel::selectedRelation() const
{
    const auto* item = m_tree->currentItem();
    return item && item->data(0, KindRole).toInt() == Relation ? item->data(0, IdRole).toInt() : 0;
}
void PointCloudRelationsPanel::updateActions()
{
    if (!m_document) return;
    const int id = selectedRegion(); const auto* region = m_document->region(id);
    const bool selected = !m_document->selected().empty();
    m_editRegion->setEnabled(region); m_removeRegion->setEnabled(region);
    m_refit->setEnabled(region && !m_busy && m_document->regionPointCount(id) >= 20);
    m_append->setEnabled(region && selected); m_replace->setEnabled(region && selected);
    m_deleteRegionPoints->setEnabled(id >= 0 && m_document->regionPointCount(id) > 0);
    m_deleteSelected->setEnabled(selected);
    m_addRelation->setEnabled(m_document->regions().size() >= 2);
    m_editRelation->setEnabled(selectedRelation() != 0); m_removeRelation->setEnabled(selectedRelation() != 0);
    m_undo->setEnabled(m_document->canUndo());
    m_detect->setEnabled(!m_busy && m_document->regionPointCount(0) >= 20);
    m_infer->setEnabled(!m_busy && m_document->regions().size() >= 2);
}
void PointCloudRelationsPanel::updateSelection()
{
    if (!m_document) return;
    const auto& selected = m_document->selected();
    QString text = tr("三维视图已选 %1 点").arg(selected.size());
    if (selected.size() == 1) {
        const auto id = selected.front(); double p[3]; m_document->data()->GetPoint(id, p);
        const auto* region = m_document->region(m_document->memberships()[static_cast<size_t>(id)]);
        text += tr("\n点 #%1 → %2\n坐标：%3, %4, %5").arg(id)
            .arg(region ? region->name : tr("未归属"))
            .arg(p[0], 0, 'g', 7).arg(p[1], 0, 'g', 7).arg(p[2], 0, 'g', 7);
    }
    m_selection->setText(text); updateActions();
}
void PointCloudRelationsPanel::applyFilter()
{
    const auto filter = m_filter->text().trimmed();
    for (int root = 0; root < m_tree->topLevelItemCount(); ++root) {
        auto* category = m_tree->topLevelItem(root);
        for (int child = 0; child < category->childCount(); ++child) {
            auto* item = category->child(child);
            item->setHidden(!filter.isEmpty() && !(item->text(0)+" "+item->text(1)).contains(filter, Qt::CaseInsensitive));
        }
    }
}
void PointCloudRelationsPanel::editRegion()
{
    const auto* region = m_document ? m_document->region(selectedRegion()) : nullptr;
    if (!region) return;
    const int id = region->id;
    QDialog dialog(this); dialog.setWindowTitle(tr("编辑集合"));
    auto* form = new QFormLayout(&dialog);
    auto* name = new QLineEdit(region->name, &dialog);
    auto* geometry = new QComboBox(&dialog);
    for (const auto& [key, text] : geometries) geometry->addItem(text, key);
    geometry->setCurrentIndex(geometry->findData(region->geometry));
    form->addRow(tr("名称"), name); form->addRow(tr("模板"), geometry);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() == QDialog::Accepted && !name->text().trimmed().isEmpty())
        emit editRegionRequested(id, name->text(), geometry->currentData().toString());
}
void PointCloudRelationsPanel::editRelation(bool create)
{
    if (!m_document) return;
    const auto* current = create ? nullptr : m_document->relation(selectedRelation());
    if (!create && !current) return;
    const int id = current ? current->id : 0;
    QDialog dialog(this); dialog.setWindowTitle(create ? tr("添加关系") : tr("编辑关系"));
    auto* form = new QFormLayout(&dialog);
    auto* first = new QComboBox(&dialog); auto* second = new QComboBox(&dialog); auto* type = new QComboBox(&dialog);
    for (const auto& region : m_document->regions()) if (m_document->regionPointCount(region.id)) {
        first->addItem(region.name, region.id); second->addItem(region.name, region.id);
    }
    for (const auto& [key, text] : relationTypes) type->addItem(text, key);
    if (current) {
        first->setCurrentIndex(first->findData(current->first)); second->setCurrentIndex(second->findData(current->second));
        type->setCurrentIndex(type->findData(current->type));
    } else if (second->count() > 1) second->setCurrentIndex(1);
    form->addRow(tr("集合 A"), first); form->addRow(tr("关系"), type); form->addRow(tr("集合 B"), second);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog); form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() == QDialog::Accepted)
        emit setRelationRequested(id, first->currentData().toInt(), second->currentData().toInt(), type->currentData().toString());
}
void PointCloudRelationsPanel::setBusy(bool busy) { m_busy = busy; updateActions(); }
void PointCloudRelationsPanel::setCadReconstructionBusy(bool busy, bool cancellable)
{
    m_cancelCad->setVisible(busy);
    m_cancelCad->setEnabled(cancellable);
}
void PointCloudRelationsPanel::setStatus(const QString& text) { m_status->setText(text); }
void PointCloudRelationsPanel::setDefaultTolerances(double diagonal)
{
    m_surfaceTolerance->setValue(std::max(0.000001, diagonal*0.001));
    m_relationTolerance->setValue(std::max(0.000001, diagonal*0.01));
}
double PointCloudRelationsPanel::surfaceTolerance() const { return m_surfaceTolerance->value(); }
double PointCloudRelationsPanel::relationTolerance() const { return m_relationTolerance->value(); }
double PointCloudRelationsPanel::angleTolerance() const { return m_angleTolerance->value(); }
