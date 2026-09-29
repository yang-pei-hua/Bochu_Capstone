#pragma once

#include "modeling/Sketch.h"

#include <QWidget>

class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QStackedWidget;

// Parameter editor for one sketch entity. It is shown as an on-demand tab of the
// Properties dock - never as a permanent one - so its numeric fields only exist
// while the user is actually editing a sketch entity.
//
// The panel is a pure view: the sketch lives in the core's feature graph, and
// this widget only mirrors the selected entity and reports edits back.
class SketchEntityPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit SketchEntityPanel(QWidget* parent = nullptr);

    // Passing nullptr shows the empty state and hides every field.
    void setEntity(const modeling::SketchEntity* entity);

    // Rebuilds the geometry described by the currently visible fields.
    modeling::SketchGeometry geometry() const;

signals:
    void entityEdited();

private:
    QDoubleSpinBox* addField(QWidget* page, QFormLayout* layout, const QString& label,
                             double value, double minimum, const QString& objectName);
    void connectField(QDoubleSpinBox* field);

    QStackedWidget* m_stack = nullptr;
    QLabel* m_title = nullptr;
    modeling::SketchEntityId m_currentId = modeling::kInvalidSketchEntityId;
    modeling::SketchEntityType m_kind = modeling::SketchEntityType::Point;

    QDoubleSpinBox* m_pointX = nullptr;
    QDoubleSpinBox* m_pointY = nullptr;
    QDoubleSpinBox* m_lineX1 = nullptr;
    QDoubleSpinBox* m_lineY1 = nullptr;
    QDoubleSpinBox* m_lineX2 = nullptr;
    QDoubleSpinBox* m_lineY2 = nullptr;
    QDoubleSpinBox* m_rectX = nullptr;
    QDoubleSpinBox* m_rectY = nullptr;
    QDoubleSpinBox* m_rectWidth = nullptr;
    QDoubleSpinBox* m_rectHeight = nullptr;
    QDoubleSpinBox* m_circleX = nullptr;
    QDoubleSpinBox* m_circleY = nullptr;
    QDoubleSpinBox* m_circleRadius = nullptr;
};
