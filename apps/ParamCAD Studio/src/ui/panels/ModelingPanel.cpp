#include "ui/panels/ModelingPanel.h"

#include <QAbstractItemView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace {

QString featureTypeLabel(modeling::FeatureType type)
{
    switch (type) {
    case modeling::FeatureType::Sketch:
        return QStringLiteral("Sketch");
    case modeling::FeatureType::Extrude:
        return QStringLiteral("Extrude");
    case modeling::FeatureType::Cut:
        return QStringLiteral("Cut");
    case modeling::FeatureType::BoxPrimitive:
        return QStringLiteral("Box");
    case modeling::FeatureType::ThroughHolePrimitive:
        return QStringLiteral("Through Hole");
    case modeling::FeatureType::SpherePrimitive: return QStringLiteral("Sphere");
    case modeling::FeatureType::CylinderPrimitive: return QStringLiteral("Cylinder");
    case modeling::FeatureType::ConePrimitive: return QStringLiteral("Cone / Frustum");
    case modeling::FeatureType::TorusPrimitive: return QStringLiteral("Torus");
    case modeling::FeatureType::ProfileBody: return QStringLiteral("Profile Body");
    }
    return QStringLiteral("Unknown");
}

}  // namespace

ModelingPanel::ModelingPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(8, 8, 8, 8);
    outer->setSpacing(8);

    m_status = new QLabel(tr("Ready"), this);
    m_status->setObjectName(QStringLiteral("modelingStatus"));
    m_status->setWordWrap(true);
    outer->addWidget(m_status);

    auto* featureGroup = new QGroupBox(tr("Features"), this);
    auto* featureLayout = new QVBoxLayout(featureGroup);
    featureLayout->setContentsMargins(6, 6, 6, 6);

    m_features = new QTreeWidget(featureGroup);
    m_features->setObjectName(QStringLiteral("featureList"));
    m_features->setColumnCount(5);
    m_features->setHeaderLabels({tr("Id"), tr("Name"), tr("Type"), tr("Valid"),
                                 tr("Error")});
    m_features->setRootIsDecorated(false);
    m_features->setUniformRowHeights(true);
    m_features->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_features->setSelectionMode(QAbstractItemView::SingleSelection);
    m_features->setMinimumHeight(120);
    m_features->header()->setSectionResizeMode(4, QHeaderView::Stretch);
    featureLayout->addWidget(m_features);

    auto* featureButtons = new QHBoxLayout();
    auto* deleteFeatureButton = new QPushButton(tr("Delete Feature"), featureGroup);
    deleteFeatureButton->setObjectName(QStringLiteral("deleteFeatureButton"));
    featureButtons->addWidget(deleteFeatureButton);
    featureButtons->addStretch();
    featureLayout->addLayout(featureButtons);
    outer->addWidget(featureGroup);

    outer->addStretch();

    connect(m_features, &QTreeWidget::itemSelectionChanged, this, [this] {
        emit featureSelected(selectedFeatureId());
    });

    connect(deleteFeatureButton, &QPushButton::clicked, this, [this] {
        const modeling::FeatureId id = selectedFeatureId();
        if (id != modeling::kInvalidFeatureId) {
            emit deleteFeatureRequested(id);
        }
    });
}

void ModelingPanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void ModelingPanel::setFeatures(const std::vector<modeling::Feature>& features)
{
    // Repopulating the list must not look like the user cleared the selection, so
    // it is remembered and restored by feature id.
    const modeling::FeatureId previous = selectedFeatureId();
    const QSignalBlocker blocker(m_features);
    m_features->clear();
    for (const modeling::Feature& feature : features) {
        auto* item = new QTreeWidgetItem(m_features);
        item->setText(0, QString::number(feature.id));
        item->setText(1, QString::fromStdString(feature.name));
        item->setText(2, featureTypeLabel(feature.type));
        item->setText(3, feature.valid ? tr("yes") : tr("no"));
        item->setText(4, QString::fromStdString(feature.errorText));
        if (feature.id == previous) {
            m_features->setCurrentItem(item);
        }
    }
    m_features->resizeColumnToContents(0);
    m_features->resizeColumnToContents(1);
    m_features->resizeColumnToContents(2);
    m_features->resizeColumnToContents(3);
}

void ModelingPanel::selectFeature(modeling::FeatureId id)
{
    for (int i = 0; i < m_features->topLevelItemCount(); ++i) {
        auto* item = m_features->topLevelItem(i);
        if (item->text(0).toULongLong() == id) {
            m_features->setCurrentItem(item);
            m_features->scrollToItem(item);
            return;
        }
    }
}

modeling::FeatureId ModelingPanel::selectedFeatureId() const noexcept
{
    const QList<QTreeWidgetItem*> selected = m_features->selectedItems();
    if (selected.isEmpty()) {
        return modeling::kInvalidFeatureId;
    }
    bool ok = false;
    const qulonglong id = selected.first()->text(0).toULongLong(&ok);
    return ok ? static_cast<modeling::FeatureId>(id) : modeling::kInvalidFeatureId;
}
