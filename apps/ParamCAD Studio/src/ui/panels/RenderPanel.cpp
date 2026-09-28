#include "ui/panels/RenderPanel.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

RenderPanel::RenderPanel(QWidget* parent)
    : QWidget(parent)
    , m_backgroundButton(new QPushButton(tr("Choose..."), this))
    , m_lighting(new QCheckBox(tr("Enable Lighting"), this))
    , m_width(new QSpinBox(this))
    , m_height(new QSpinBox(this))
    , m_texturePath(new QLabel(this))
    , m_textureEnabled(new QCheckBox(tr("Project Texture"), this))
    , m_textureAxis(new QComboBox(this))
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

    auto* textureGroup = new QGroupBox(tr("Texture"), this);
    auto* textureLayout = new QVBoxLayout(textureGroup);

    auto* browseButton = new QPushButton(tr("Load Image..."), textureGroup);
    browseButton->setObjectName(QStringLiteral("textureBrowseButton"));
    auto* clearButton = new QPushButton(tr("Remove"), textureGroup);
    clearButton->setObjectName(QStringLiteral("textureClearButton"));

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(browseButton);
    buttonRow->addWidget(clearButton);

    // The picture is cast onto the body from one side, so the only thing the
    // user has to choose besides the file is which side that is.
    m_textureEnabled->setObjectName(QStringLiteral("textureApply"));
    m_textureAxis->setObjectName(QStringLiteral("textureProjection"));
    // Per Face projects each face from its own normal, so it is the only choice
    // that keeps the picture square on every side of a box.
    m_textureAxis->addItems({tr("Auto"), tr("X Axis"), tr("Y Axis"), tr("Z Axis"),
                             tr("Per Face")});
    m_texturePath->setObjectName(QStringLiteral("texturePathLabel"));
    m_texturePath->setWordWrap(true);

    auto* textureForm = new QFormLayout();
    textureForm->addRow(tr("Image"), m_texturePath);
    textureForm->addRow(tr("Display"), m_textureEnabled);
    textureForm->addRow(tr("Direction"), m_textureAxis);

    textureLayout->addLayout(buttonRow);
    textureLayout->addLayout(textureForm);

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(textureGroup);
    layout->addWidget(captureButton);
    layout->addStretch();

    connect(m_backgroundButton, &QPushButton::clicked, this, &RenderPanel::chooseBackgroundColor);
    connect(m_lighting, &QCheckBox::toggled, this, &RenderPanel::lightingChanged);
    connect(captureButton, &QPushButton::clicked, this, &RenderPanel::captureRequested);
    connect(browseButton, &QPushButton::clicked, this, &RenderPanel::textureBrowseRequested);
    connect(clearButton, &QPushButton::clicked, this, &RenderPanel::textureCleared);
    connect(m_textureEnabled, &QCheckBox::toggled, this, &RenderPanel::textureEnabledChanged);
    connect(m_textureAxis, &QComboBox::currentIndexChanged,
            this, &RenderPanel::textureProjectionChanged);
    updateColorButton();
    setTexturePath(QString());
}

int RenderPanel::outputWidth() const
{
    return m_width->value();
}

int RenderPanel::outputHeight() const
{
    return m_height->value();
}

void RenderPanel::setTexturePath(const QString& path)
{
    m_texturePath->setText(path.isEmpty() ? tr("No texture") : QFileInfo(path).fileName());
    m_texturePath->setToolTip(path);
}

void RenderPanel::setTextureEnabled(bool enabled)
{
    // The viewport is the one that owns the state; this only mirrors it back
    // into the check box without asking it to change again.
    const QSignalBlocker blocker(m_textureEnabled);
    m_textureEnabled->setChecked(enabled);
}

int RenderPanel::textureProjection() const
{
    return m_textureAxis->currentIndex();
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