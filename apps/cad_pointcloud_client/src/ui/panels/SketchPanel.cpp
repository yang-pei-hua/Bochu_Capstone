#include "ui/panels/SketchPanel.h"

#include <modeling/SketchValidation.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <cstddef>

namespace {

constexpr double kDefaultExtrudeDepthMm = 30.0;
constexpr double kDefaultCutDepthMm = 10.0;

QString kindLabel(const modeling::SketchGeometry& geometry)
{
    if (std::holds_alternative<modeling::Point2D>(geometry)) {
        return QStringLiteral("Point");
    }
    if (std::holds_alternative<modeling::Line2D>(geometry)) {
        return QStringLiteral("Line");
    }
    if (std::holds_alternative<modeling::Rectangle2D>(geometry)) {
        return QStringLiteral("Rectangle");
    }
    return QStringLiteral("Circle");
}

QString describe(const modeling::SketchGeometry& geometry)
{
    if (const auto* point = std::get_if<modeling::Point2D>(&geometry)) {
        return QStringLiteral("%1, %2")
            .arg(point->x, 0, 'f', 2)
            .arg(point->y, 0, 'f', 2);
    }
    if (const auto* line = std::get_if<modeling::Line2D>(&geometry)) {
        return QStringLiteral("(%1, %2) -> (%3, %4)")
            .arg(line->x1, 0, 'f', 2)
            .arg(line->y1, 0, 'f', 2)
            .arg(line->x2, 0, 'f', 2)
            .arg(line->y2, 0, 'f', 2);
    }
    if (const auto* rectangle = std::get_if<modeling::Rectangle2D>(&geometry)) {
        return QStringLiteral("at (%1, %2), %3 x %4")
            .arg(rectangle->x, 0, 'f', 2)
            .arg(rectangle->y, 0, 'f', 2)
            .arg(rectangle->width, 0, 'f', 2)
            .arg(rectangle->height, 0, 'f', 2);
    }
    const auto& circle = std::get<modeling::Circle2D>(geometry);
    return QStringLiteral("center (%1, %2), r %3")
        .arg(circle.x, 0, 'f', 2)
        .arg(circle.y, 0, 'f', 2)
        .arg(circle.radius, 0, 'f', 2);
}

modeling::ExtrudeOperation operationAt(int index)
{
    switch (index) {
    case 1:
        return modeling::ExtrudeOperation::Join;
    case 2:
        return modeling::ExtrudeOperation::Cut;
    case 3:
        return modeling::ExtrudeOperation::Intersect;
    default:
        return modeling::ExtrudeOperation::NewBody;
    }
}

}  // namespace

SketchPanel::SketchPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(8, 8, 8, 8);
    outer->setSpacing(6);

    m_planeLabel = new QLabel(tr("Plane: none"), this);
    m_planeLabel->setObjectName(QStringLiteral("sketchPlaneLabel"));
    outer->addWidget(m_planeLabel);

    auto* extrudeGroup = new QGroupBox(tr("Extrude / Cut"), this);
    auto* extrudeLayout = new QGridLayout(extrudeGroup);

    m_extrudeOperation = new QComboBox(extrudeGroup);
    m_extrudeOperation->setObjectName(QStringLiteral("extrudeOperation"));
    m_extrudeOperation->addItems({tr("New Body"), tr("Join"), tr("Cut"), tr("Intersect")});

    m_extrudeDepth = new QLineEdit(QStringLiteral("30.0"), extrudeGroup);
    m_extrudeDepth->setObjectName(QStringLiteral("extrudeDepth"));
    m_extrudeDepth->setValidator(new QDoubleValidator(0.001, 1000.0, 3, m_extrudeDepth));
    m_extrudeReverse = new QCheckBox(tr("Reverse"), extrudeGroup);
    m_extrudeReverse->setObjectName(QStringLiteral("extrudeReverse"));
    auto* extrudeButton = new QPushButton(tr("Extrude"), extrudeGroup);
    extrudeButton->setObjectName(QStringLiteral("extrudeButton"));

    m_cutDepth = new QLineEdit(QStringLiteral("10.0"), extrudeGroup);
    m_cutDepth->setObjectName(QStringLiteral("cutDepth"));
    m_cutDepth->setValidator(new QDoubleValidator(0.001, 1000.0, 3, m_cutDepth));
    m_cutThroughAll = new QCheckBox(tr("Through all"), extrudeGroup);
    m_cutThroughAll->setObjectName(QStringLiteral("cutThroughAll"));
    m_cutReverse = new QCheckBox(tr("Reverse"), extrudeGroup);
    m_cutReverse->setObjectName(QStringLiteral("cutReverse"));
    auto* cutButton = new QPushButton(tr("Cut"), extrudeGroup);
    cutButton->setObjectName(QStringLiteral("cutButton"));

    extrudeLayout->addWidget(new QLabel(tr("Operation"), extrudeGroup), 0, 0);
    extrudeLayout->addWidget(m_extrudeOperation, 0, 1);
    extrudeLayout->addWidget(new QLabel(tr("Depth"), extrudeGroup), 0, 2);
    extrudeLayout->addWidget(m_extrudeDepth, 0, 3);
    extrudeLayout->addWidget(m_extrudeReverse, 0, 4);
    extrudeLayout->addWidget(extrudeButton, 0, 5);
    extrudeLayout->addWidget(new QLabel(tr("Cut Depth"), extrudeGroup), 1, 0);
    extrudeLayout->addWidget(m_cutDepth, 1, 1);
    extrudeLayout->addWidget(m_cutThroughAll, 1, 2);
    extrudeLayout->addWidget(m_cutReverse, 1, 4);
    extrudeLayout->addWidget(cutButton, 1, 5);
    outer->addWidget(extrudeGroup);

    m_entities = new QTreeWidget(this);
    m_entities->setObjectName(QStringLiteral("sketchEntityList"));
    m_entities->setColumnCount(3);
    m_entities->setHeaderLabels({tr("Id"), tr("Type"), tr("Geometry")});
    m_entities->setRootIsDecorated(false);
    m_entities->setUniformRowHeights(true);
    m_entities->setMinimumHeight(110);
    m_entities->setSelectionMode(QAbstractItemView::SingleSelection);
    outer->addWidget(m_entities, 1);

    auto* deleteButton = new QPushButton(tr("Delete Entity"), this);
    deleteButton->setObjectName(QStringLiteral("deleteEntityButton"));
    outer->addWidget(deleteButton);

    m_status = new QLabel(tr("Ready"), this);
    m_status->setObjectName(QStringLiteral("sketchStatus"));
    m_status->setWordWrap(true);
    outer->addWidget(m_status);

    connect(deleteButton, &QPushButton::clicked, this, &SketchPanel::deleteSelected);
    connect(extrudeButton, &QPushButton::clicked, this, [this] {
        emit extrudeRequested(extrudeDepthValue(), m_extrudeReverse->isChecked(),
                              operationAt(m_extrudeOperation->currentIndex()));
    });
    connect(cutButton, &QPushButton::clicked, this, [this] {
        emit cutRequested(cutDepthValue(), m_cutThroughAll->isChecked(),
                          m_cutReverse->isChecked());
    });

    connect(m_entities, &QTreeWidget::itemSelectionChanged, this, [this] {
        if (m_refreshing) {
            return;
        }
        const modeling::SketchEntityId id = selectedEntityId();
        if (id == modeling::kInvalidSketchEntityId) {
            emit entitySelectionCleared();
        } else {
            emit entitySelected(id);
        }
    });

    rebuildList();
    updateStatus();
}

void SketchPanel::setSketch(const modeling::SketchFeatureParams& params,
                            const sketchapp::PlaneFrame& frame)
{
    m_params = params;
    m_frame = frame;
    m_hasSketch = true;
    setPlaneText(QStringLiteral("Plane: %1").arg(sketchapp::planeLabel(m_params.plane)));
    rebuildList();
    updateStatus();
}

void SketchPanel::clearSketch()
{
    m_params = modeling::SketchFeatureParams{};
    m_frame = sketchapp::PlaneFrame{};
    m_hasSketch = false;
    setPlaneText(tr("Plane: none"));
    rebuildList();
    updateStatus();
}

bool SketchPanel::hasSketch() const noexcept
{
    return m_hasSketch;
}

void SketchPanel::setPlaneText(const QString& text)
{
    m_planeLabel->setText(text);
}

const modeling::SketchEntity* SketchPanel::findEntity(
    modeling::SketchEntityId entityId) const noexcept
{
    for (const modeling::SketchEntity& entity : m_params.entities) {
        if (entity.id == entityId) {
            return &entity;
        }
    }
    return nullptr;
}

modeling::SketchEntityId SketchPanel::selectedEntityId() const noexcept
{
    const auto selected = m_entities->selectedItems();
    if (selected.isEmpty()) {
        return modeling::kInvalidSketchEntityId;
    }
    return selected.first()->text(0).toULongLong();
}

void SketchPanel::selectEntity(modeling::SketchEntityId entityId)
{
    if (entityId == modeling::kInvalidSketchEntityId) {
        m_entities->setCurrentItem(nullptr);
        m_entities->clearSelection();
        return;
    }
    for (int row = 0; row < m_entities->topLevelItemCount(); ++row) {
        QTreeWidgetItem* item = m_entities->topLevelItem(row);
        if (item->text(0).toULongLong() == entityId) {
            m_entities->setCurrentItem(item);
            return;
        }
    }
    m_entities->clearSelection();
}

void SketchPanel::deleteSelected()
{
    const modeling::SketchEntityId id = selectedEntityId();
    if (id == modeling::kInvalidSketchEntityId) {
        setStatusText(tr("Select an entity to delete"));
        return;
    }
    emit entityRemoveRequested(id);
}

double SketchPanel::extrudeDepthValue() const
{
    bool ok = false;
    const double value = m_extrudeDepth->text().toDouble(&ok);
    return ok && value > 0.0 ? value : kDefaultExtrudeDepthMm;
}

double SketchPanel::cutDepthValue() const
{
    bool ok = false;
    const double value = m_cutDepth->text().toDouble(&ok);
    return ok && value > 0.0 ? value : kDefaultCutDepthMm;
}

void SketchPanel::rebuildList()
{
    // Rebuilding the rows would otherwise look like the user cleared the
    // selection, which would close the Properties tab mid-edit.
    const modeling::SketchEntityId previous = selectedEntityId();
    m_refreshing = true;
    m_entities->clear();
    bool restored = false;
    for (const modeling::SketchEntity& entity : m_params.entities) {
        auto* row = new QTreeWidgetItem(m_entities);
        row->setText(0, QString::number(entity.id));
        row->setText(1, kindLabel(entity.geometry));
        row->setText(2, describe(entity.geometry));
        if (entity.id == previous) {
            row->setSelected(true);
            restored = true;
        }
    }
    if (previous == modeling::kInvalidSketchEntityId) {
        m_entities->clearSelection();
    }
    m_entities->resizeColumnToContents(0);
    m_entities->resizeColumnToContents(1);
    m_refreshing = false;

    if (previous != modeling::kInvalidSketchEntityId && !restored) {
        // The selected entity is gone, so the parameter editor has nothing left
        // to describe; the rebuild above suppressed the signal that would say so.
        emit entitySelectionCleared();
    }
}

void SketchPanel::updateStatus()
{
    if (!m_hasSketch) {
        m_status->setText(tr("Ready | no sketch open"));
        return;
    }

    // The core decides whether the sketch currently forms an extrudable profile;
    // the panel only reports its verdict, which is why an unfinished sketch is a
    // perfectly normal state here.
    std::string error;
    const bool ready = modeling::validateSketch(m_params, error);
    m_status->setText(tr("%1 entities | %2 | %3")
                          .arg(static_cast<int>(m_params.entities.size()))
                          .arg(sketchapp::planeLabel(m_params.plane))
                          .arg(ready ? tr("closed profile ready")
                                     : QString::fromStdString(error)));
}

void SketchPanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}
