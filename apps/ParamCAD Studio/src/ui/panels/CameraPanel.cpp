#include "ui/panels/CameraPanel.h"

#include "core/OrbitCamera.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <cmath>

namespace
{
QDoubleSpinBox* makeCoordinateSpinBox(QWidget* parent)
{
    auto* spinBox = new QDoubleSpinBox(parent);
    spinBox->setRange(-100000.0, 100000.0);
    spinBox->setDecimals(4);
    spinBox->setSingleStep(0.1);
    return spinBox;
}

QWidget* makeVectorRow(const std::array<QDoubleSpinBox*, 3>& fields, QWidget* parent)
{
    auto* row = new QWidget(parent);
    auto* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);
    const QStringList labels{QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    for (int index = 0; index < 3; ++index) {
        layout->addWidget(new QLabel(labels[index], row));
        layout->addWidget(fields[static_cast<std::size_t>(index)], 1);
    }
    return row;
}

// Writing an unchanged value would move the cursor of a field the user is
// editing, so only touch the widget when the value actually differs.
void setIfDifferent(QDoubleSpinBox* spinBox, double value)
{
    if (std::abs(spinBox->value() - value) > 1.0e-9) {
        const QSignalBlocker blocker(spinBox);
        spinBox->setValue(value);
    }
}
}

CameraPanel::CameraPanel(QWidget* parent)
    : QWidget(parent)
{
    for (auto*& field : m_position) {
        field = makeCoordinateSpinBox(this);
    }
    for (auto*& field : m_target) {
        field = makeCoordinateSpinBox(this);
    }
    for (auto*& field : m_up) {
        field = makeCoordinateSpinBox(this);
    }

    m_fov = new QDoubleSpinBox(this);
    m_fov->setRange(1.0, 179.0);
    m_fov->setDecimals(1);
    m_fov->setSuffix(QStringLiteral("°"));

    m_projection = new QComboBox(this);
    m_projection->addItems({QStringLiteral("Perspective"), QStringLiteral("Orthographic")});

    m_orbitDistance = new QDoubleSpinBox(this);
    m_orbitDistance->setObjectName(QStringLiteral("orbitDistance"));
    m_orbitDistance->setRange(0.001, 100000.0);
    m_orbitDistance->setDecimals(3);
    m_orbitDistance->setSingleStep(1.0);

    m_orbitAzimuth = new QDoubleSpinBox(this);
    m_orbitAzimuth->setObjectName(QStringLiteral("orbitAzimuth"));
    m_orbitAzimuth->setRange(0.0, 360.0);
    m_orbitAzimuth->setDecimals(1);
    m_orbitAzimuth->setSingleStep(15.0);
    m_orbitAzimuth->setWrapping(true);
    m_orbitAzimuth->setSuffix(QStringLiteral("°"));

    m_orbitElevation = new QDoubleSpinBox(this);
    m_orbitElevation->setObjectName(QStringLiteral("orbitElevation"));
    m_orbitElevation->setRange(-OrbitCamera::kMaxElevation, OrbitCamera::kMaxElevation);
    m_orbitElevation->setDecimals(1);
    m_orbitElevation->setSingleStep(5.0);
    m_orbitElevation->setSuffix(QStringLiteral("°"));

    m_lookAtOrigin = new QCheckBox(tr("Look at origin"), this);
    m_lookAtOrigin->setObjectName(QStringLiteral("lookAtOrigin"));
    m_lookAtOrigin->setToolTip(
        tr("Keep the target pinned to (0, 0, 0) and place the camera from the orbit values."));
    m_lookAtOrigin->setChecked(true);

    auto* form = new QFormLayout();
    form->addRow(tr("Position"), makeVectorRow(m_position, this));
    form->addRow(tr("Target"), makeVectorRow(m_target, this));
    form->addRow(tr("Up"), makeVectorRow(m_up, this));
    form->addRow(tr("FOV"), m_fov);
    form->addRow(tr("Projection"), m_projection);

    auto* orbitGroup = new QGroupBox(tr("Orbit"), this);
    auto* orbitForm = new QFormLayout(orbitGroup);
    orbitForm->addRow(tr("Distance"), m_orbitDistance);
    orbitForm->addRow(tr("Azimuth"), m_orbitAzimuth);
    orbitForm->addRow(tr("Elevation"), m_orbitElevation);
    orbitForm->addRow(QString(), m_lookAtOrigin);

    auto* applyButton = new QPushButton(tr("Apply"), this);
    auto* resetButton = new QPushButton(tr("Reset Camera"), this);
    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(applyButton);
    buttonRow->addWidget(resetButton);

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(orbitGroup);
    layout->addLayout(buttonRow);
    layout->addStretch();

    for (QDoubleSpinBox* field : m_position) {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, [this](double) { applyFields(); });
    }
    for (QDoubleSpinBox* field : m_target) {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, [this](double) { applyFields(); });
    }
    for (QDoubleSpinBox* field : m_up) {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, [this](double) { applyFields(); });
    }
    connect(m_fov, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this](double) { applyFields(); });
    connect(m_projection, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int) { applyFields(); });

    for (QDoubleSpinBox* field : {m_orbitDistance, m_orbitAzimuth, m_orbitElevation}) {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, [this](double) { applyOrbitFields(); });
    }

    connect(m_lookAtOrigin, &QCheckBox::toggled, this, [this](bool checked) {
        if (checked) {
            // Ticking the box means "aim at the origin", so the now read-only
            // target and up fields are normalized to match before applying.
            for (QDoubleSpinBox* field : m_target) {
                const QSignalBlocker blocker(field);
                field->setValue(0.0);
            }
            const double up[3]{0.0, 0.0, 1.0};
            for (int index = 0; index < 3; ++index) {
                const QSignalBlocker blocker(m_up[static_cast<std::size_t>(index)]);
                m_up[static_cast<std::size_t>(index)]->setValue(up[index]);
            }
        }
        updateEnabledState();
        applyFields();
    });

    connect(applyButton, &QPushButton::clicked, this, [this]() { applyFields(); });
    connect(resetButton, &QPushButton::clicked, this, &CameraPanel::resetRequested);

    updateEnabledState();
    setParameters(CameraParameters{});
    m_ready = true;
}

void CameraPanel::setParameters(const CameraParameters& parameters)
{
    if (m_feedbackSuppressed) {
        return;
    }

    const bool lookAtOrigin = m_lookAtOrigin->isChecked();
    for (int index = 0; index < 3; ++index) {
        const std::size_t i = static_cast<std::size_t>(index);
        setIfDifferent(m_position[i], parameters.position[index]);
        setIfDifferent(m_target[i], lookAtOrigin ? 0.0 : parameters.target[index]);
        setIfDifferent(m_up[i], parameters.up[index]);
    }
    setIfDifferent(m_fov, parameters.fieldOfView);

    const int wantedProjection = parameters.parallelProjection ? 1 : 0;
    if (m_projection->currentIndex() != wantedProjection) {
        const QSignalBlocker blocker(m_projection);
        m_projection->setCurrentIndex(wantedProjection);
    }

    // The sphere -> camera mapping is not bit-exact, so echoing it back while the
    // orbit fields are being authored would nudge the values under the user.
    if (!m_authoringOrbit) {
        const OrbitParameters orbit = OrbitCamera::fromCamera(parameters);
        setIfDifferent(m_orbitDistance, orbit.distance);
        setIfDifferent(m_orbitAzimuth, orbit.azimuthDeg);
        setIfDifferent(m_orbitElevation, orbit.elevationDeg);
    }

    // With "look at origin" enabled, any camera the rest of the app applies must
    // be snapped onto the origin. Only the target is corrected: forcing the
    // position or up would also move the Top/Bottom pole views, which sit exactly
    // at the clamped limit. The second cameraChanged this triggers finds a zero
    // target and stops, so the correction converges without an extra flag.
    if (lookAtOrigin && m_ready && !m_snapping
        && (std::abs(parameters.target[0]) > 1.0e-9
            || std::abs(parameters.target[1]) > 1.0e-9
            || std::abs(parameters.target[2]) > 1.0e-9)) {
        m_snapping = true;
        CameraParameters aimed = parameters;
        aimed.target[0] = 0.0;
        aimed.target[1] = 0.0;
        aimed.target[2] = 0.0;
        emit applyRequested(aimed);
        m_snapping = false;
    }
}

void CameraPanel::setFeedbackSuppressed(bool suppressed)
{
    m_feedbackSuppressed = suppressed;
}

void CameraPanel::releaseOriginLock()
{
    if (!m_lookAtOrigin->isChecked()) {
        return;
    }
    // The box is dropped without going through the toggled handler: the camera
    // is being dragged right now, and applying the fields back would fight it.
    // The next camera change fills the target in from the panned pose.
    const QSignalBlocker blocker(m_lookAtOrigin);
    m_lookAtOrigin->setChecked(false);
    updateEnabledState();
}

CameraParameters CameraPanel::parameters() const
{
    CameraParameters result;
    for (int index = 0; index < 3; ++index) {
        const std::size_t i = static_cast<std::size_t>(index);
        result.position[index] = m_position[i]->value();
        result.target[index] = m_target[i]->value();
        result.up[index] = m_up[i]->value();
    }
    result.fieldOfView = m_fov->value();
    result.parallelProjection = m_projection->currentIndex() == 1;
    return result;
}

void CameraPanel::applyFields()
{
    if (m_feedbackSuppressed || m_authoringOrbit || m_snapping) {
        return;
    }
    emit applyRequested(parameters());
}

void CameraPanel::applyOrbitFields()
{
    if (m_feedbackSuppressed || m_authoringOrbit || m_snapping) {
        return;
    }

    const OrbitParameters orbit{m_orbitDistance->value(), m_orbitAzimuth->value(),
                               m_orbitElevation->value()};
    const CameraParameters applied =
        OrbitCamera::toCamera(orbit, parameters(), m_lookAtOrigin->isChecked());

    m_authoringOrbit = true;
    emit applyRequested(applied);
    m_authoringOrbit = false;
}

void CameraPanel::updateEnabledState()
{
    const bool lookAtOrigin = m_lookAtOrigin->isChecked();

    // Aiming at the origin makes the target implicit, and the orbit angles then
    // define the up vector, so both become derived rather than editable.
    for (QDoubleSpinBox* field : m_target) {
        field->setEnabled(!lookAtOrigin);
    }
    for (QDoubleSpinBox* field : m_up) {
        field->setEnabled(!lookAtOrigin);
    }
    for (QDoubleSpinBox* field : {m_orbitDistance, m_orbitAzimuth, m_orbitElevation}) {
        field->setReadOnly(!lookAtOrigin);
    }
}