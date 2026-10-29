#include "ui/panels/PointCloudRelationsPanel.h"

#include <QAbstractItemView>
#include <QAction>
#include <QFileInfo>
#include <QMenu>
#include <QScrollArea>
#include <QToolButton>
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
    if(key=="part")return QStringLiteral("部件候选");
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
    layout->setContentsMargins(6, 6, 6, 6); layout->setSpacing(6);
    m_source = new QLabel(this); m_source->setWordWrap(true);
    m_summary = new QLabel(tr("请先打开点云"), this); m_summary->setWordWrap(true);
    m_selection = new QLabel(this); m_selection->setWordWrap(true);
    layout->addWidget(m_source); layout->addWidget(m_summary); layout->addWidget(m_selection);
    m_pages = new QTabWidget(this); m_pages->setObjectName("pointCloudPages");
    layout->addWidget(m_pages, 1);

    const auto makeAction = [this](const QString& text, const char* name) {
        auto* result = new QAction(text, this); result->setObjectName(name); return result;
    };
    m_newRegion = makeAction(tr("从选区新建集合…"), "createPointCollectionAction");
    m_editRegion = makeAction(tr("编辑名称 / 模板…"), "editPointCollectionAction");
    m_refit = makeAction(tr("按此集合模板重建"), "rebuildSelectedTemplateAction");
    m_append = makeAction(tr("加入当前选区"), "appendCloudSelectionAction");
    m_replace = makeAction(tr("以当前选区替换成员"), "replaceCloudCollectionAction");
    m_unassign = makeAction(tr("解除选中点的归属"), "unassignCloudSelectionAction");
    m_removeRegion = makeAction(tr("删除集合（保留点）"), "removePointCollectionAction");
    m_deleteRegionPoints = makeAction(tr("删除集合内所有点"), "deleteRegionPointsAction");
    m_deleteSelected = makeAction(tr("删除选中点"), "deleteSelectedCloudPointsAction");
    m_addRelation = makeAction(tr("添加关系…"), "addCloudRelationAction");
    m_editRelation = makeAction(tr("编辑关系…"), "editCloudRelationAction");
    m_removeRelation = makeAction(tr("删除关系（保留点和集合）"), "removeCloudRelationAction");
    m_undo = makeAction(tr("撤销点云编辑"), "undoCloudEditAction");
    m_reconstructAction = makeAction(tr("重建 CAD"), "rebuildCloudTemplatesAction");

    auto* collections = new QWidget(m_pages);
    auto* collectionLayout = new QVBoxLayout(collections);
    collectionLayout->setContentsMargins(4, 6, 4, 4);
    auto* collectionTools = new QHBoxLayout;
    m_filter = new QLineEdit(collections);
    m_filter->setObjectName("cloudCollectionFilter");
    m_filter->setPlaceholderText(tr("搜索集合"));
    auto* assignment = new QToolButton(collections);
    assignment->setText(tr("选区归属")); assignment->setObjectName("cloudAssignmentButton");
    assignment->setPopupMode(QToolButton::InstantPopup);
    auto* assignments = new QMenu(assignment);
    assignments->addAction(m_newRegion); assignments->addAction(m_append);
    assignments->addAction(m_replace); assignments->addAction(m_unassign);
    assignment->setMenu(assignments);
    collectionTools->addWidget(m_filter, 1); collectionTools->addWidget(assignment);
    collectionLayout->addLayout(collectionTools);
    const auto makeTree = [this](QWidget* parent, const char* name) {
        auto* tree = new QTreeWidget(parent); tree->setObjectName(name);
        tree->setHeaderLabels({tr("集合 / 关系"), tr("类型 / 来源"), tr("点数")});
        tree->setMinimumHeight(180); tree->setUniformRowHeights(true); tree->setIndentation(14);
        tree->setSelectionMode(QAbstractItemView::SingleSelection);
        tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
        tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        tree->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(tree, &QTreeWidget::customContextMenuRequested, this,
            [this, tree](const QPoint& pos) { showContextMenu(tree, pos); });
        connect(tree, &QTreeWidget::currentItemChanged, this, [this, tree](QTreeWidgetItem* item) {
            m_activeTree = tree; updateActions();
            if (!item) return;
            const int kind = item->data(0, KindRole).toInt();
            const auto id = item->data(0, IdRole).toLongLong();
            if (kind == Region || kind == PointList) emit regionSelected(static_cast<int>(id));
            if (kind == Relation) emit relationSelected(static_cast<int>(id));
            if (kind == Point) emit pointSelected(static_cast<vtkIdType>(id));
        });
        connect(tree, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item) {
            if (item->data(0, KindRole).toInt() == Region && item->data(0, IdRole).toInt() > 0) editRegion();
            else if (item->data(0, KindRole).toInt() == Relation) editRelation(false);
        });
        return tree;
    };
    m_tree = makeTree(collections, "pointCloudRelationshipTree");
    m_activeTree = m_tree;
    collectionLayout->addWidget(m_tree, 1);
    auto* hint = new QLabel(tr("右键集合可编辑、调整成员或删除；双击编辑。"), collections);
    hint->setWordWrap(true); collectionLayout->addWidget(hint);
    m_pages->addTab(collections, tr("集合"));

    auto* relations = new QWidget(m_pages);
    auto* relationLayout = new QVBoxLayout(relations);
    relationLayout->setContentsMargins(4, 6, 4, 4);
    auto* relationTools = new QHBoxLayout;
    m_relationFilter = new QLineEdit(relations);
    m_relationFilter->setObjectName("cloudRelationFilter");
    m_relationFilter->setPlaceholderText(tr("搜索关系"));
    auto* add = new QToolButton(relations); add->setDefaultAction(m_addRelation);
    m_infer = new QPushButton(tr("更新关系"), relations);
    m_infer->setObjectName("inferCloudRelationsButton");
    relationTools->addWidget(m_relationFilter, 1);
    relationTools->addWidget(add); relationTools->addWidget(m_infer);
    relationLayout->addLayout(relationTools);
    m_relationTree = makeTree(relations, "cloudRelationsTree");
    relationLayout->addWidget(m_relationTree, 1);
    auto* note = new QLabel(tr("关系用于几何分析和标注，尚不驱动模型变形。右键关系可编辑或删除。"), relations);
    note->setWordWrap(true); relationLayout->addWidget(note);
    m_pages->addTab(relations, tr("关系"));

    auto* settings = new QWidget;
    auto* form = new QFormLayout(settings);
    form->setContentsMargins(6, 8, 6, 6); form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    m_surfaceTolerance = tolerance(settings, 0.1); m_surfaceTolerance->setObjectName("surfaceFitTolerance");
    m_relationTolerance = tolerance(settings, 1.0);
    m_angleTolerance = tolerance(settings, 2.0); m_angleTolerance->setRange(0.01, 30);
    m_angleTolerance->setDecimals(2); m_angleTolerance->setSuffix(QStringLiteral("°"));
    form->addRow(tr("拟合距离容差"), m_surfaceTolerance);
    form->addRow(tr("关系距离容差"), m_relationTolerance);
    form->addRow(tr("关系角度容差"), m_angleTolerance);
    auto* units = new QLabel(tr("距离使用原始点云坐标单位。调整容差后可重新识别或重建；更新关系会采用新的关系容差。"), settings);
    units->setWordWrap(true); form->addRow(units);
    auto* settingsScroll = new QScrollArea;
    settingsScroll->setWidgetResizable(true); settingsScroll->setFrameShape(QFrame::NoFrame);
    settingsScroll->setWidget(settings);
    m_pages->addTab(settingsScroll, tr("设置"));

    m_status = new QLabel(this); m_status->setObjectName("pointCloudRelationStatus");
    m_status->setWordWrap(true); layout->addWidget(m_status);
    auto* footer = new QHBoxLayout;
    m_detect = new QPushButton(tr("识别表面"), this); m_detect->setObjectName("detectCloudSurfacesButton");
    m_detect->setToolTip(tr("识别未归属点的曲面，保留已有集合；此操作不生成 CAD 实体。"));
    m_reconstruct = new QToolButton(this); m_reconstruct->setObjectName("rebuildCloudTemplatesButton");
    m_reconstruct->setDefaultAction(m_reconstructAction);
    m_reconstruct->setPopupMode(QToolButton::MenuButtonPopup);
    auto* reconstruction = new QMenu(m_reconstruct); reconstruction->addAction(m_refit);
    m_reconstruct->setMenu(reconstruction);
    auto* more = new QToolButton(this); more->setText(tr("更多"));
    more->setObjectName("cloudMoreButton"); more->setPopupMode(QToolButton::InstantPopup);
    auto* files = new QMenu(more); files->addAction(m_undo); files->addSeparator();
    auto* save = files->addAction(tr("保存点云标注…")); save->setObjectName("saveCloudAnnotationsAction");
    auto* load = files->addAction(tr("载入点云标注…")); load->setObjectName("loadCloudAnnotationsAction");
    auto* exportCloud = files->addAction(tr("导出工作点云…")); exportCloud->setObjectName("exportWorkingCloudAction");
    more->setMenu(files);
    footer->addWidget(m_detect, 1); footer->addWidget(m_reconstruct, 1); footer->addWidget(more);
    layout->addLayout(footer);
    m_cancelCad = new QPushButton(tr("取消 CAD 重建"), this);
    m_cancelCad->setObjectName("cancelCadReconstructionButton"); m_cancelCad->hide();
    layout->addWidget(m_cancelCad);

    connect(m_filter, &QLineEdit::textChanged, this, &PointCloudRelationsPanel::applyFilter);
    connect(m_relationFilter, &QLineEdit::textChanged, this, &PointCloudRelationsPanel::applyFilter);
    connect(m_tree, &QTreeWidget::itemExpanded, this, [this](QTreeWidgetItem* item) {
        if (item->data(0, KindRole).toInt() == PointList && item->childCount() == 0) loadPointPage(item, 0);
    });
    connect(m_tree, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item) {
        if (item->data(0, KindRole).toInt() != More) return;
        auto* parent = item->parent(); const int offset = item->data(0, OffsetRole).toInt();
        delete parent->takeChild(parent->indexOfChild(item)); loadPointPage(parent, offset);
    });
    connect(m_newRegion, &QAction::triggered, this, &PointCloudRelationsPanel::createRegion);
    connect(m_editRegion, &QAction::triggered, this, &PointCloudRelationsPanel::editRegion);
    connect(m_refit, &QAction::triggered, this, [this] { if (selectedRegion() > 0) emit refitRequested(selectedRegion()); });
    connect(m_append, &QAction::triggered, this, [this] { if (selectedRegion() > 0) emit appendSelectionRequested(selectedRegion()); });
    connect(m_replace, &QAction::triggered, this, [this] { if (selectedRegion() > 0) emit replaceSelectionRequested(selectedRegion()); });
    connect(m_removeRegion, &QAction::triggered, this, [this] { if (selectedRegion() > 0) emit removeRegionRequested(selectedRegion()); });
    connect(m_deleteRegionPoints, &QAction::triggered, this, [this] { if (selectedRegion() >= 0) emit deleteRegionPointsRequested(selectedRegion()); });
    connect(m_unassign, &QAction::triggered, this, &PointCloudRelationsPanel::unassignRequested);
    connect(m_deleteSelected, &QAction::triggered, this, &PointCloudRelationsPanel::deleteSelectedPointsRequested);
    connect(m_addRelation, &QAction::triggered, this, [this] { editRelation(true); });
    connect(m_editRelation, &QAction::triggered, this, [this] { editRelation(false); });
    connect(m_removeRelation, &QAction::triggered, this, [this] { if (selectedRelation()) emit removeRelationRequested(selectedRelation()); });
    connect(m_detect, &QPushButton::clicked, this, &PointCloudRelationsPanel::detectRequested);
    connect(m_reconstructAction, &QAction::triggered, this, &PointCloudRelationsPanel::reconstructRequested);
    connect(m_infer, &QPushButton::clicked, this, &PointCloudRelationsPanel::inferRequested);
    connect(m_undo, &QAction::triggered, this, &PointCloudRelationsPanel::undoRequested);
    connect(save, &QAction::triggered, this, &PointCloudRelationsPanel::saveRequested);
    connect(load, &QAction::triggered, this, &PointCloudRelationsPanel::loadRequested);
    connect(exportCloud, &QAction::triggered, this, &PointCloudRelationsPanel::exportRequested);
    connect(m_cancelCad, &QPushButton::clicked, this, &PointCloudRelationsPanel::cancelCadRequested);
    connect(m_pages, &QTabWidget::currentChanged, this, [this](int) {
        m_activeTree = m_pages->currentWidget() == m_relationTree->parentWidget() ? m_relationTree : m_tree;
        updateActions();
    });
    setEnabled(false);
}
void PointCloudRelationsPanel::insertSelectionPage(QWidget* page)
{
    m_pages->insertTab(0, page, tr("选点"));
    m_pages->setCurrentIndex(1);
}
QMenu* PointCloudRelationsPanel::createSelectionMenu(QWidget* parent)
{
    auto* menu = new QMenu(parent);
    menu->addAction(m_newRegion); menu->addAction(m_unassign);
    menu->addSeparator(); menu->addAction(m_deleteSelected); menu->addAction(m_undo);
    return menu;
}
void PointCloudRelationsPanel::setSourceInfo(const QString& path, const QString& unit)
{
    const QString displayUnit = unit == "mm" ? tr("mm（毫米）")
        : unit == "m" ? tr("m（米）") : tr("未知，重建时需确认");
    m_source->setText(path.isEmpty() ? QString() : tr("%1 · 单位：%2").arg(QFileInfo(path).fileName(), displayUnit));
    m_source->setToolTip(path);
}
void PointCloudRelationsPanel::showContextMenu(QTreeWidget* tree, const QPoint& position)
{
    if (!m_document || !isEnabled()) return;
    auto* item = tree->itemAt(position);
    // Right-click changes the command target while retaining the user's 3D
    // selection for append/replace. Ordinary left-click still highlights members.
    { const QSignalBlocker blocker(tree); tree->setCurrentItem(item); }
    m_activeTree = tree; updateActions();
    QMenu menu(this); menu.setObjectName("cloudContextMenu");
    if (selectedRelation()) {
        menu.addAction(m_editRelation); menu.addAction(m_removeRelation);
    } else if (selectedRegion() >= 0) {
        if (selectedRegion() > 0) {
            menu.addAction(m_editRegion); menu.addAction(m_refit);
            menu.addSeparator(); menu.addAction(m_append); menu.addAction(m_replace);
            menu.addAction(m_addRelation);
            menu.addSeparator(); menu.addAction(m_removeRegion);
        }
        menu.addAction(m_deleteRegionPoints);
    }
    if (item && item->data(0, KindRole).toInt() == Point) {
        const auto id = item->data(0, IdRole).toLongLong();
        auto* select = menu.addAction(tr("选中此点"));
        connect(select, &QAction::triggered, this, [this, id] { emit pointSelected(id); });
    }
    menu.addSeparator(); menu.addAction(m_newRegion); menu.addAction(m_unassign);
    menu.addAction(m_deleteSelected); menu.addAction(m_undo);
    menu.exec(tree->viewport()->mapToGlobal(position));
}
void PointCloudRelationsPanel::createRegion()
{
    if (!m_document || m_document->selected().empty()) return;
    QDialog dialog(this); dialog.setObjectName("createCloudCollectionDialog");
    dialog.setWindowTitle(tr("从选区新建集合"));
    auto* form = new QFormLayout(&dialog);
    auto* name = new QLineEdit(&dialog); name->setObjectName("pointCloudCollectionName");
    name->setPlaceholderText(tr("留空则自动命名"));
    auto* geometry = new QComboBox(&dialog); geometry->setObjectName("pointCloudTemplateGeometry");
    for (const auto& [key, text] : geometries) geometry->addItem(text, key);
    form->addRow(tr("名称"), name); form->addRow(tr("几何模板"), geometry);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() == QDialog::Accepted)
        emit assignRequested(geometry->currentData().toString(), name->text().trimmed(), 0);
}

void PointCloudRelationsPanel::setDocument(const PointCloudSelection& document, bool available)
{
    m_document = &document; setEnabled(available);
    const QString previous = itemKey(m_tree->currentItem());
    const QString previousRelation = itemKey(m_relationTree->currentItem());
    const auto* currentCollection = m_tree->currentItem();
    const int previousRegion = currentCollection
        && (currentCollection->data(0, KindRole).toInt() == Region
            || currentCollection->data(0, KindRole).toInt() == PointList)
        ? currentCollection->data(0, IdRole).toInt() : -1;
    std::set<QString> expanded;
    QTreeWidgetItemIterator before(m_tree);
    while (*before) { if ((*before)->isExpanded()) expanded.insert(itemKey(*before)); ++before; }
    const QSignalBlocker blocker(m_tree), relationBlocker(m_relationTree);
    m_tree->clear(); m_relationTree->clear(); m_points.clear();
    for (size_t i = 0; i < document.memberships().size(); ++i) {
        if (document.active(static_cast<vtkIdType>(i))) m_points[document.memberships()[i]].push_back(static_cast<vtkIdType>(i));
    }
    m_summary->setText(available ? tr("有效 %1 / 原始 %2 点 · 已删除 %3\n%4 个集合 · %5 条关系")
        .arg(document.activePointCount()).arg(document.memberships().size())
        .arg(document.memberships().size()-static_cast<size_t>(document.activePointCount()))
        .arg(document.regions().size()).arg(document.relations().size()) : tr("请先打开点云"));
    auto* collections = new QTreeWidgetItem(m_tree, {tr("表面 / 点集合"), QString(), QString()});
    auto* relations = new QTreeWidgetItem(m_relationTree, {tr("关系列表"), QString(), QString()});
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
        if (region.id == previousRegion) m_tree->setCurrentItem(item);
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
                } else if(param.value().isString())text=param.value().toString();
                else if(param.value().isBool())text=param.value().toBool()?tr("是"):tr("否");
                else text = QString::number(param.value().toDouble(), 'g', 6);
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
    if (previousRegion == 0) m_tree->setCurrentItem(unassigned);
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
    QTreeWidgetItemIterator relationItems(m_relationTree);
    while (*relationItems) {
        if (itemKey(*relationItems) == previousRelation) m_relationTree->setCurrentItem(*relationItems);
        ++relationItems;
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
    auto* item = m_activeTree->currentItem();
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
    const auto* item = m_activeTree->currentItem();
    return item && item->data(0, KindRole).toInt() == Relation ? item->data(0, IdRole).toInt() : 0;
}
void PointCloudRelationsPanel::updateActions()
{
    if (!m_document) return;
    const int id = selectedRegion(); const auto* region = m_document->region(id);
    const bool selected = !m_document->selected().empty();
    m_newRegion->setEnabled(selected); m_unassign->setEnabled(selected);
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
    m_reconstructAction->setEnabled(!m_busy && m_document->activePointCount() > 0);
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
    for (auto* tree : {m_tree, m_relationTree}) {
        const auto filter = (tree == m_tree ? m_filter : m_relationFilter)->text().trimmed();
        for (int root = 0; root < tree->topLevelItemCount(); ++root) {
            auto* category = tree->topLevelItem(root);
            for (int child = 0; child < category->childCount(); ++child) {
                auto* item = category->child(child);
                item->setHidden(!filter.isEmpty() && !(item->text(0)+" "+item->text(1)).contains(filter, Qt::CaseInsensitive));
            }
        }
    }
}
void PointCloudRelationsPanel::editRegion()
{
    const auto* region = m_document ? m_document->region(selectedRegion()) : nullptr;
    if (!region) return;
    const int id = region->id;
    QDialog dialog(this); dialog.setObjectName("editCloudCollectionDialog"); dialog.setWindowTitle(tr("编辑集合"));
    auto* form = new QFormLayout(&dialog);
    auto* name = new QLineEdit(region->name, &dialog); name->setObjectName("pointCloudCollectionName");
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
