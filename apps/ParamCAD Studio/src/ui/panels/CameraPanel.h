#pragma once

#include "core/CameraController.h"

#include <QWidget>

#include <array>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;

class CameraPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit CameraPanel(QWidget* parent = nullptr);
    void setParameters(const CameraParameters& parameters);

    // While a batch capture drives the camera the panel must not chase it,
    // otherwise every captured step rewrites the orbit readout and fights the
    // batch for control of the pose.
    void setFeedbackSuppressed(bool suppressed);

    // Panning moves the target, which the "look at origin" pin would drag back
    // to the origin on the next camera change, so a pan drops the pin instead of
    // being undone by it.
    void releaseOriginLock();

signals:
    void applyRequested(const CameraParameters& parameters);
    void resetRequested();

private:
    CameraParameters parameters() const;
    void updateEnabledState();
    void applyFields();
    void applyOrbitFields();

    std::array<QDoubleSpinBox*, 3> m_position{};
    std::array<QDoubleSpinBox*, 3> m_target{};
    std::array<QDoubleSpinBox*, 3> m_up{};
    QDoubleSpinBox* m_fov = nullptr;
    QComboBox* m_projection = nullptr;

    QDoubleSpinBox* m_orbitDistance = nullptr;
    QDoubleSpinBox* m_orbitAzimuth = nullptr;
    QDoubleSpinBox* m_orbitElevation = nullptr;
    QCheckBox* m_lookAtOrigin = nullptr;

    // True while the orbit fields are the ones being authored, so their echo is
    // not written back over the values the user is typing.
    bool m_authoringOrbit = false;
    bool m_snapping = false;
    bool m_ready = false;
    bool m_feedbackSuppressed = false;
};