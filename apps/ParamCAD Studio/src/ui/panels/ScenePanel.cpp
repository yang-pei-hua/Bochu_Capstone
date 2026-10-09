#include "ui/panels/ScenePanel.h"

#include <QHash>
#include <QMenu>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace
{
constexpr int NodeTypeRole = Qt::UserRole + 1;
constexpr int VisibleRole = Qt::UserRole + 2;
}
ScenePanel::ScenePanel(QWidget* parent)
    : QWidget(parent)
    , m_tree(new QTreeWidget(this))
{
    m_tree->setHeaderHidden(true);
    m_tree->setAlternatingRowColors(true);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(m_tree, &QWidget::customContextMenuRequested, this, [this](const QPoint& position) {
        auto* item = m_tree->itemAt(position);
        if (!item || item->isHidden()) return;
        const auto type = static_cast<SceneNodeType>(item->data(0, NodeTypeRole).toInt());
        if (type != SceneNodeType::Model && type != SceneNodeType::PointCloud) return;

        m_tree->setCurrentItem(item);
        const bool visible = item->data(0, VisibleRole).toBool();
        QMenu menu(this);
        auto* visibilityAction = menu.addAction(visible ? tr("隐藏") : tr("显示"));
        visibilityAction->setObjectName(QStringLiteral("sceneVisibilityAction"));
        auto* deleteAction = menu.addAction(tr("删除"));
        deleteAction->setObjectName(QStringLiteral("sceneDeleteAction"));
        connect(visibilityAction, &QAction::triggered, this, [this, type, visible] {
            emit nodeVisibilityRequested(type, !visible);
        });
        connect(deleteAction, &QAction::triggered, this, [this, type] {
            emit nodeDeleteRequested(type);
        });
        menu.exec(m_tree->viewport()->mapToGlobal(position));
    });

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemSelectionChanged, this, [this]() {
        const auto selected = m_tree->selectedItems();
        if (selected.isEmpty()) {
            return;
        }
        const auto type = static_cast<SceneNodeType>(selected.first()->data(0, NodeTypeRole).toInt());
        emit nodeSelected(type);
    });
}

void ScenePanel::setScene(const Scene& scene)
{
    m_tree->clear();
    QHash<QString, QTreeWidgetItem*> items;

    for (const SceneNode& node : scene.nodes()) {
        auto* item = new QTreeWidgetItem();
        item->setText(0, node.name);
        item->setData(0, NodeTypeRole, static_cast<int>(node.type));
        item->setData(0, VisibleRole, true);
        items.insert(node.id, item);

        if (node.parentId.isEmpty()) {
            m_tree->addTopLevelItem(item);
        } else if (auto* parentItem = items.value(node.parentId, nullptr)) {
            parentItem->addChild(item);
        }
    }

    m_tree->expandAll();
}

bool ScenePanel::updateNodeLabel(SceneNodeType type, const QString& label)
{
    // Walking the tree keeps the order deterministic, which a hash lookup of
    // node ids would not.
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator != nullptr) {
        QTreeWidgetItem* item = *iterator;
        if (static_cast<SceneNodeType>(item->data(0, NodeTypeRole).toInt()) == type) {
            item->setText(0, label);
            return true;
        }
        ++iterator;
    }
    return false;
}

void ScenePanel::setNodeState(SceneNodeType type, bool present, bool visible)
{
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator != nullptr) {
        auto* item = *iterator;
        if (static_cast<SceneNodeType>(item->data(0, NodeTypeRole).toInt()) == type) {
            if (!present && item->isSelected()) item->setSelected(false);
            item->setHidden(!present);
            item->setData(0, VisibleRole, visible);
            QFont font = item->font(0);
            font.setItalic(!visible);
            item->setFont(0, font);
            item->setToolTip(0, visible ? tr("可见") : tr("已隐藏；右击可显示"));
            return;
        }
        ++iterator;
    }
}
