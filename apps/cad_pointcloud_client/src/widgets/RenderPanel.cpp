#include "widgets/RenderPanel.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QFormLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

RenderPanel::RenderPanel(QWidget* parent)
    : QWidget(parent)
    , m_backgroundButton(new QPushButton(tr("Choose..."), this))
    , m_lighting(new QCheckBox(tr("Enable Lighting"), this))
    , m_width(new QSpinBox(this))
    , m_height(new QSpinBox(this))
{
    m_lighting->setChecked(true);

    for (QSpinBox* spinBox : {m_width, m_height}) {
        spinBox->setRange(64, 16384);
        spinBox->setSingleStep(64);
    }
    m_width->setValue(1920);
    m_height->setValue(1080);

    auto* captureButton = new QPushButton(tr("Capture Image"), this);
    captureButton->setProperty("primary", true);

    auto* form = new QFormLayout();
    form->addRow(tr("Background Color"), m_backgroundButton);
    form->addRow(QString(), m_lighting);
    form->addRow(tr("Output Width"), m_width);
    form->addRow(tr("Output Height"), m_height);

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(captureButton);
    layout->addStretch();

    connect(m_backgroundButton, &QPushButton::clicked, this, &RenderPanel::chooseBackgroundColor);
    connect(m_lighting, &QCheckBox::toggled, this, &RenderPanel::lightingChanged);
    connect(captureButton, &QPushButton::clicked, this, &RenderPanel::captureRequested);
    updateColorButton();
}
int RenderPanel::outputWidth() const
{
    return m_width->value();
}

int RenderPanel::outputHeight() const
{
    return m_height->value();
}

void RenderPanel::chooseBackgroundColor()
{
    const QColor color = QColorDialog::getColor(m_backgroundColor, this, tr("Viewport Background"));
    if (!color.isValid()) {
        return;
    }
    m_backgroundColor = color;
    updateColorButton();
    emit backgroundColorChanged(color);
}

void RenderPanel::updateColorButton()
{
    m_backgroundButton->setText(m_backgroundColor.name(QColor::HexRgb));
    m_backgroundButton->setStyleSheet(QStringLiteral(
        "QPushButton { border-left: 18px solid %1; }").arg(m_backgroundColor.name()));
}
