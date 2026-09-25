#include "widgets/ScenePanel.h"

#include <QHash>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace
{
constexpr int NodeTypeRole = Qt::UserRole + 1;
}
ScenePanel::ScenePanel(QWidget* parent)
    : QWidget(parent)
    , m_tree(new QTreeWidget(this))
{
    m_tree->setHeaderHidden(true);
    m_tree->setAlternatingRowColors(true);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);

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
        items.insert(node.id, item);

        if (node.parentId.isEmpty()) {
            m_tree->addTopLevelItem(item);
        } else if (auto* parentItem = items.value(node.parentId, nullptr)) {
            parentItem->addChild(item);
        }
    }

    m_tree->expandAll();
    const auto modelItems = m_tree->findItems(QStringLiteral("DemoCube"), Qt::MatchExactly | Qt::MatchRecursive);
    if (!modelItems.isEmpty()) {
        m_tree->setCurrentItem(modelItems.first());
    }
}
