#include "ui/panels/RenderPanel.h"

#include "app/CaptureSettings.h"

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
    , m_backgroundButton(new QPushButton(tr("选择…"), this))
    , m_lighting(new QCheckBox(tr("启用光照"), this))
    , m_width(new QSpinBox(this))
    , m_height(new QSpinBox(this))
    , m_texturePath(new QLabel(this))
    , m_textureEnabled(new QCheckBox(tr("投影纹理"), this))
    , m_textureAxis(new QComboBox(this))
{
    m_lighting->setChecked(true);

    for (QSpinBox* spinBox : {m_width, m_height}) {
        spinBox->setRange(64, 16384);
        spinBox->setSingleStep(64);
    }
    int width = 1920, height = 1080;
    try {
        const auto settings = loadCaptureSettings();
        width = settings.imageWidth;
        height = settings.imageHeight;
    } catch (const std::exception&) {
    }
    m_width->setValue(width);
    m_height->setValue(height);

    auto* captureButton = new QPushButton(tr("拍摄图像"), this);
    captureButton->setProperty("primary", true);

    auto* form = new QFormLayout();
    form->addRow(tr("背景颜色"), m_backgroundButton);
    form->addRow(QString(), m_lighting);
    form->addRow(tr("输出宽度"), m_width);
    form->addRow(tr("输出高度"), m_height);

    auto* textureGroup = new QGroupBox(tr("纹理"), this);
    auto* textureLayout = new QVBoxLayout(textureGroup);

    auto* browseButton = new QPushButton(tr("载入图像…"), textureGroup);
    browseButton->setObjectName(QStringLiteral("textureBrowseButton"));
    auto* clearButton = new QPushButton(tr("移除"), textureGroup);
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
    m_textureAxis->addItems({tr("自动"), tr("X 轴"), tr("Y 轴"), tr("Z 轴"),
                             tr("按面")});
    m_texturePath->setObjectName(QStringLiteral("texturePathLabel"));
    m_texturePath->setWordWrap(true);

    auto* textureForm = new QFormLayout();
    textureForm->addRow(tr("图像"), m_texturePath);
    textureForm->addRow(tr("显示"), m_textureEnabled);
    textureForm->addRow(tr("方向"), m_textureAxis);

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
    m_texturePath->setText(path.isEmpty() ? tr("无纹理") : QFileInfo(path).fileName());
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

RenderPresentation RenderPanel::presentation() const
{
    RenderPresentation state;
    state.background=m_backgroundColor; state.lighting=m_lighting->isChecked();
    state.textureEnabled=m_textureEnabled->isChecked(); state.width=outputWidth(); state.height=outputHeight();
    state.textureProjection=textureProjection(); state.texturePath=m_texturePath->toolTip();
    return state;
}
void RenderPanel::setPresentation(const RenderPresentation& state)
{
    const QSignalBlocker block(this);
    m_backgroundColor=state.background; updateColorButton(); m_lighting->setChecked(state.lighting);
    m_width->setValue(state.width); m_height->setValue(state.height);
    m_textureAxis->setCurrentIndex(state.textureProjection); setTextureEnabled(state.textureEnabled);
    setTexturePath(state.texturePath);
}

void RenderPanel::chooseBackgroundColor()
{
    const QColor color = QColorDialog::getColor(m_backgroundColor, this, tr("视口背景"));
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
