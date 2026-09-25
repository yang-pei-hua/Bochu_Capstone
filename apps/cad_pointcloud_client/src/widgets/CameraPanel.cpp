#include "widgets/CameraPanel.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

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

    auto* form = new QFormLayout();
    form->addRow(tr("Position"), makeVectorRow(m_position, this));
    form->addRow(tr("Target"), makeVectorRow(m_target, this));
    form->addRow(tr("Up"), makeVectorRow(m_up, this));
    form->addRow(tr("FOV"), m_fov);
    form->addRow(tr("Projection"), m_projection);

    auto* applyButton = new QPushButton(tr("Apply"), this);
    auto* resetButton = new QPushButton(tr("Reset Camera"), this);
    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(applyButton);
    buttonRow->addWidget(resetButton);

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addLayout(buttonRow);
    layout->addStretch();

    connect(applyButton, &QPushButton::clicked, this, [this]() {
        emit applyRequested(parameters());
    });
    connect(resetButton, &QPushButton::clicked, this, &CameraPanel::resetRequested);

    setParameters(CameraParameters{});
}

void CameraPanel::setParameters(const CameraParameters& parameters)
{
    for (int index = 0; index < 3; ++index) {
        const std::size_t i = static_cast<std::size_t>(index);
        const QSignalBlocker positionBlocker(m_position[i]);
        const QSignalBlocker targetBlocker(m_target[i]);
        const QSignalBlocker upBlocker(m_up[i]);
        m_position[i]->setValue(parameters.position[index]);
        m_target[i]->setValue(parameters.target[index]);
        m_up[i]->setValue(parameters.up[index]);
    }
    const QSignalBlocker fovBlocker(m_fov);
    const QSignalBlocker projectionBlocker(m_projection);
    m_fov->setValue(parameters.fieldOfView);
    m_projection->setCurrentIndex(parameters.parallelProjection ? 1 : 0);
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
