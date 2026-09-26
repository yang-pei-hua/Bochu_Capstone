#include "ui/panels/PropertyPanel.h"

#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/RenderPanel.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

namespace
{
QDoubleSpinBox* makeTransformSpinBox(QWidget* parent, double minimum, double maximum,
                                     double value, double step)
{
    auto* spinBox = new QDoubleSpinBox(parent);
    spinBox->setRange(minimum, maximum);
    spinBox->setDecimals(3);
    spinBox->setSingleStep(step);
    spinBox->setValue(value);
    return spinBox;
}
QWidget* makeVectorEditor(const std::array<QDoubleSpinBox*, 3>& fields, QWidget* parent)
{
    auto* editor = new QWidget(parent);
    auto* layout = new QHBoxLayout(editor);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(3);
    const QStringList axes{QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    for (int index = 0; index < 3; ++index) {
        layout->addWidget(new QLabel(axes[index], editor));
        layout->addWidget(fields[static_cast<std::size_t>(index)], 1);
    }
    return editor;
}
}

PropertyPanel::PropertyPanel(QWidget* parent)
    : QWidget(parent)
    , m_tabs(new QTabWidget(this))
    , m_cameraPanel(new CameraPanel(this))
    , m_renderPanel(new RenderPanel(this))
    , m_capturePanel(new CapturePanel(this))
    , m_reconstructPanel(new ReconstructPanel(this))
{
    m_objectTab = createObjectTab();
    m_tabs->addTab(m_objectTab, tr("Object"));
    m_tabs->addTab(m_cameraPanel, tr("Camera"));
    m_tabs->addTab(m_renderPanel, tr("Render"));
    // Capture and Reconstruct are tools, not permanent tabs: they are inserted
    // on demand.

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_tabs);
}

CameraPanel* PropertyPanel::cameraPanel() const
{
    return m_cameraPanel;
}

RenderPanel* PropertyPanel::renderPanel() const
{
    return m_renderPanel;
}

CapturePanel* PropertyPanel::capturePanel() const
{
    return m_capturePanel;
}

ReconstructPanel* PropertyPanel::reconstructPanel() const
{
    return m_reconstructPanel;
}

void PropertyPanel::showCapturePanel(bool visible)
{
    const int index = m_tabs->indexOf(m_capturePanel);
    if (visible) {
        if (index < 0) {
            // removeTab() only detaches the page, so the panel keeps its shot
            // list, output directory and orbit settings across a hide/show.
            m_tabs->addTab(m_capturePanel, tr("Capture"));
        }
        m_tabs->setCurrentWidget(m_capturePanel);
    } else if (index >= 0) {
        m_tabs->removeTab(index);
    }
}

bool PropertyPanel::isCapturePanelVisible() const
{
    return m_tabs->indexOf(m_capturePanel) >= 0;
}

void PropertyPanel::showReconstructPanel(bool visible)
{
    const int index = m_tabs->indexOf(m_reconstructPanel);
    if (visible) {
        if (index < 0) {
            // Detaching keeps the paths and options across a hide/show.
            m_tabs->addTab(m_reconstructPanel, tr("Reconstruct"));
        }
        m_tabs->setCurrentWidget(m_reconstructPanel);
    } else if (index >= 0) {
        m_tabs->removeTab(index);
    }
}

bool PropertyPanel::isReconstructPanelVisible() const
{
    return m_tabs->indexOf(m_reconstructPanel) >= 0;
}

void PropertyPanel::showObjectProperties()
{
    m_tabs->setCurrentWidget(m_objectTab);
}

void PropertyPanel::showCameraProperties()
{
    m_tabs->setCurrentWidget(m_cameraPanel);
}

QWidget* PropertyPanel::createObjectTab()
{
    auto* tab = new QWidget(this);

    for (auto*& field : m_position) {
        field = makeTransformSpinBox(tab, -100000.0, 100000.0, 0.0, 0.1);
    }
    for (auto*& field : m_rotation) {
        field = makeTransformSpinBox(tab, -360.0, 360.0, 0.0, 1.0);
        field->setSuffix(QStringLiteral("°"));
    }
    for (auto*& field : m_scale) {
        field = makeTransformSpinBox(tab, 0.001, 1000.0, 1.0, 0.1);
    }

    auto* transformGroup = new QGroupBox(tr("Transform"), tab);
    auto* transformForm = new QFormLayout(transformGroup);
    transformForm->addRow(tr("Position"), makeVectorEditor(m_position, transformGroup));
    transformForm->addRow(tr("Rotation"), makeVectorEditor(m_rotation, transformGroup));
    transformForm->addRow(tr("Scale"), makeVectorEditor(m_scale, transformGroup));

    m_visible = new QCheckBox(tr("Visible"), tab);
    m_visible->setChecked(true);
    m_representation = new QComboBox(tab);
    m_representation->addItems({tr("Surface"), tr("Wireframe"), tr("Points")});
    m_opacity = makeTransformSpinBox(tab, 0.0, 1.0, 1.0, 0.05);
    m_colorButton = new QPushButton(tab);
    updateColorButton();

    auto* displayGroup = new QGroupBox(tr("Display"), tab);
    auto* displayForm = new QFormLayout(displayGroup);
    displayForm->addRow(QString(), m_visible);
    displayForm->addRow(tr("Representation"), m_representation);
    displayForm->addRow(tr("Opacity"), m_opacity);
    displayForm->addRow(tr("Color"), m_colorButton);

    auto* layout = new QVBoxLayout(tab);
    layout->addWidget(transformGroup);
    layout->addWidget(displayGroup);
    layout->addStretch();

    const auto connectTransformFields = [this](const auto& fields) {
        for (QDoubleSpinBox* field : fields) {
            connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged),
                    this, [this](double) { emitTransform(); });
        }
    };
    connectTransformFields(m_position);
    connectTransformFields(m_rotation);
    connectTransformFields(m_scale);

    connect(m_visible, &QCheckBox::toggled, this, &PropertyPanel::objectVisibilityChanged);
    connect(m_representation, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::objectRepresentationChanged);
    connect(m_opacity, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::objectOpacityChanged);
    connect(m_colorButton, &QPushButton::clicked, this, &PropertyPanel::chooseObjectColor);

    return tab;
}

void PropertyPanel::emitTransform()
{
    emit objectTransformChanged(
        m_position[0]->value(), m_position[1]->value(), m_position[2]->value(),
        m_rotation[0]->value(), m_rotation[1]->value(), m_rotation[2]->value(),
        m_scale[0]->value(), m_scale[1]->value(), m_scale[2]->value());
}

void PropertyPanel::chooseObjectColor()
{
    const QColor color = QColorDialog::getColor(m_objectColor, this, tr("Object Color"));
    if (!color.isValid()) {
        return;
    }
    m_objectColor = color;
    updateColorButton();
    emit objectColorChanged(color);
}

void PropertyPanel::updateColorButton()
{
    if (!m_colorButton) {
        return;
    }
    m_colorButton->setText(m_objectColor.name(QColor::HexRgb));
    m_colorButton->setStyleSheet(QStringLiteral(
        "QPushButton { border-left: 18px solid %1; }").arg(m_objectColor.name()));
}
