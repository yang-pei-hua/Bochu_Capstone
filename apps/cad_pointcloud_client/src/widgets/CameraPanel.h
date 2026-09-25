#pragma once

#include "core/CameraController.h"

#include <QWidget>

#include <array>

class QComboBox;
class QDoubleSpinBox;

class CameraPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit CameraPanel(QWidget* parent = nullptr);
    void setParameters(const CameraParameters& parameters);

signals:
    void applyRequested(const CameraParameters& parameters);
    void resetRequested();

private:
    CameraParameters parameters() const;

    std::array<QDoubleSpinBox*, 3> m_position{};
    std::array<QDoubleSpinBox*, 3> m_target{};
    std::array<QDoubleSpinBox*, 3> m_up{};
    QDoubleSpinBox* m_fov = nullptr;
    QComboBox* m_projection = nullptr;
};
