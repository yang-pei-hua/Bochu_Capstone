#include "ui/panels/ModelingPanel.h"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
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

    auto* group = new QGroupBox(tr("Demo Model"), this);
    auto* grid = new QGridLayout(group);
    grid->setColumnStretch(1, 1);

    m_width = addParameter(grid, 0, tr("Rectangle Width"), 50.0, 0.0, 1000.0);
    m_height = addParameter(grid, 1, tr("Rectangle Height"), 30.0, 0.0, 1000.0);
    m_extrusionDepth = addParameter(grid, 2, tr("Extrude Depth"), 30.0, 0.0, 1000.0);
    m_holeCenterX = addParameter(grid, 3, tr("Hole Center X"), 25.0, -1000.0, 1000.0);
    m_holeCenterY = addParameter(grid, 4, tr("Hole Center Y"), 15.0, -1000.0, 1000.0);
    m_holeRadius = addParameter(grid, 5, tr("Hole Radius"), 5.0, 0.0, 1000.0);
    m_cutDepth = addParameter(grid, 6, tr("Cut Depth"), 10.0, 0.0, 1000.0);
    outer->addWidget(group);

    auto* buttons = new QHBoxLayout();
    auto* generateButton = new QPushButton(tr("Generate Model"), this);
    auto* applyButton = new QPushButton(tr("Apply Parameters"), this);
    auto* fitViewButton = new QPushButton(tr("Fit View"), this);
    buttons->addWidget(generateButton);
    buttons->addWidget(applyButton);
    buttons->addWidget(fitViewButton);
    outer->addLayout(buttons);

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
    outer->addWidget(featureGroup);

    outer->addStretch();

    connect(generateButton, &QPushButton::clicked, this, [this] {
        emit generateRequested(parameters());
    });
    connect(applyButton, &QPushButton::clicked, this, [this] {
        emit applyRequested(parameters());
    });
    connect(fitViewButton, &QPushButton::clicked, this, &ModelingPanel::fitViewRequested);
}

DemoModelParameters ModelingPanel::parameters() const
{
    DemoModelParameters result;
    result.width = m_width->value();
    result.height = m_height->value();
    result.extrusionDepth = m_extrusionDepth->value();
    result.holeCenterX = m_holeCenterX->value();
    result.holeCenterY = m_holeCenterY->value();
    result.holeRadius = m_holeRadius->value();
    result.cutDepth = m_cutDepth->value();
    return result;
}

void ModelingPanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void ModelingPanel::setFeatures(const std::vector<modeling::Feature>& features)
{
    m_features->clear();
    for (const modeling::Feature& feature : features) {
        auto* item = new QTreeWidgetItem(m_features);
        item->setText(0, QString::number(feature.id));
        item->setText(1, QString::fromStdString(feature.name));
        item->setText(2, featureTypeLabel(feature.type));
        item->setText(3, feature.valid ? tr("yes") : tr("no"));
        item->setText(4, QString::fromStdString(feature.errorText));
    }
    m_features->resizeColumnToContents(0);
    m_features->resizeColumnToContents(1);
    m_features->resizeColumnToContents(2);
    m_features->resizeColumnToContents(3);
}

QDoubleSpinBox* ModelingPanel::addParameter(QGridLayout* layout, int row,
                                            const QString& label, double value,
                                            double minimum, double maximum)
{
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(minimum, maximum);
    spin->setDecimals(2);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    layout->addWidget(new QLabel(label, this), row, 0);
    layout->addWidget(spin, row, 1);
    return spin;
}
