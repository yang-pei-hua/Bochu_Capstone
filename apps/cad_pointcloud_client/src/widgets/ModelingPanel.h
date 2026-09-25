#pragma once

#include "app/ModelingController.h"

#include <QWidget>

class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QPushButton;
class QTreeWidget;

// Minimal parameter panel for the demo model. It only collects UI values and
// forwards them; it never stores features or touches the modeling core.
class ModelingPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit ModelingPanel(QWidget* parent = nullptr);

    DemoModelParameters parameters() const;
    void setStatusText(const QString& text);

    // Read-only mirror of the core feature history. Nothing here is editable and
    // nothing is written back; the panel is a view, not a second model.
    void setFeatures(const std::vector<modeling::Feature>& features);

signals:
    void generateRequested(const DemoModelParameters& parameters);
    void applyRequested(const DemoModelParameters& parameters);
    void fitViewRequested();

private:
    QDoubleSpinBox* addParameter(QGridLayout* layout, int row, const QString& label,
                                 double value, double minimum, double maximum);

    QDoubleSpinBox* m_width = nullptr;
    QDoubleSpinBox* m_height = nullptr;
    QDoubleSpinBox* m_extrusionDepth = nullptr;
    QDoubleSpinBox* m_holeCenterX = nullptr;
    QDoubleSpinBox* m_holeCenterY = nullptr;
    QDoubleSpinBox* m_holeRadius = nullptr;
    QDoubleSpinBox* m_cutDepth = nullptr;
    QTreeWidget* m_features = nullptr;
    QLabel* m_status = nullptr;
};
