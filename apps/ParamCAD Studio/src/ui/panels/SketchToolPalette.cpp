#include "ui/panels/SketchToolPalette.h"

#include <QAction>
#include <QLabel>
#include <QMenu>
#include <QToolButton>
#include <QVBoxLayout>

namespace {

constexpr int kPaletteWidth = 84;
constexpr int kButtonHeight = 28;

QString shapeName(ViewportTool tool)
{
    switch (tool) {
    case ViewportTool::Rectangle:
        return QStringLiteral("Rectangle");
    case ViewportTool::Circle:
        return QStringLiteral("Circle");
    default:
        return QStringLiteral("Shape");
    }
}

}  // namespace

SketchToolPalette::SketchToolPalette(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("sketchToolPalette"));
    setFixedWidth(kPaletteWidth);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(6, 6, 6, 6);
    outer->setSpacing(6);

    m_selection = new QLabel(tr("未选择任何对象"), this);
    m_selection->setObjectName(QStringLiteral("selectionLabel"));
    m_selection->setWordWrap(true);
    outer->addWidget(m_selection);

    m_sketchSelect = addToolButton(QStringLiteral("sketchSelect"), tr("选择"),
                                   tr("选择草图图元并查看其属性"), true);
    m_sketchSelect->setChecked(true);
    m_toolPoint = addToolButton(QStringLiteral("toolPoint"), tr("点"),
                                tr("在草图平面上点击放置点"), true);
    m_toolLine = addToolButton(QStringLiteral("toolLine"), tr("直线"),
                               tr("点击两点绘制直线"), true);

    m_shapeButton = new QToolButton(this);
    m_shapeButton->setObjectName(QStringLiteral("shapeButton"));
    m_shapeButton->setText(tr("形状"));
    m_shapeButton->setToolTip(tr("矩形或圆"));
    m_shapeButton->setCheckable(true);
    m_shapeButton->setFixedHeight(kButtonHeight);
    m_shapeButton->setPopupMode(QToolButton::InstantPopup);
    auto* shapeMenu = new QMenu(m_shapeButton);
    QAction* rectangleAction = shapeMenu->addAction(tr("矩形"));
    rectangleAction->setObjectName(QStringLiteral("shapeRectangle"));
    QAction* circleAction = shapeMenu->addAction(tr("圆"));
    circleAction->setObjectName(QStringLiteral("shapeCircle"));
    m_shapeButton->setMenu(shapeMenu);

    m_shapeLabel = new QLabel(tr("形状"), this);
    m_shapeLabel->setObjectName(QStringLiteral("shapeLabel"));
    m_exitSketch = addToolButton(QStringLiteral("exitSketch"), tr("退出草图"),
                                 tr("退出草图编辑，草图会保留"), false);

    outer->addWidget(m_sketchSelect);
    outer->addWidget(m_toolPoint);
    outer->addWidget(m_toolLine);
    outer->addWidget(m_shapeButton);
    outer->addWidget(m_shapeLabel);
    outer->addWidget(m_exitSketch);
    outer->addStretch(1);

    connect(m_sketchSelect, &QToolButton::clicked, this,
            [this]() { onSketchToolClicked(ViewportTool::Select); });
    connect(m_toolPoint, &QToolButton::clicked, this,
            [this]() { onSketchToolClicked(ViewportTool::Point); });
    connect(m_toolLine, &QToolButton::clicked, this,
            [this]() { onSketchToolClicked(ViewportTool::Line); });
    connect(rectangleAction, &QAction::triggered, this,
            [this]() { onSketchToolClicked(ViewportTool::Rectangle); });
    connect(circleAction, &QAction::triggered, this,
            [this]() { onSketchToolClicked(ViewportTool::Circle); });
    connect(m_exitSketch, &QToolButton::clicked, this,
            [this]() { emit exitSketchRequested(); });

    applyToolState();
}

QToolButton* SketchToolPalette::addToolButton(const QString& objectName, const QString& text,
                                              const QString& tooltip, bool checkable)
{
    auto* button = new QToolButton(this);
    button->setObjectName(objectName);
    button->setText(text);
    button->setToolTip(tooltip);
    button->setCheckable(checkable);
    button->setFixedHeight(kButtonHeight);
    return button;
}

void SketchToolPalette::setSelectionText(const QString& text)
{
    m_selection->setText(text.isEmpty() ? tr("未选择任何对象") : text);
}

void SketchToolPalette::setActiveTool(ViewportTool tool)
{
    m_tool = tool;
    applyToolState();
}

ViewportTool SketchToolPalette::activeTool() const noexcept
{
    return m_tool;
}

void SketchToolPalette::onSketchToolClicked(ViewportTool tool)
{
    m_tool = tool;
    applyToolState();
    emit toolChanged(m_tool);
}

// The drawing tools behave as one exclusive choice, so exactly one of the
// buttons is checked at any time.
void SketchToolPalette::applyToolState()
{
    m_sketchSelect->setChecked(m_tool == ViewportTool::Select);
    m_toolPoint->setChecked(m_tool == ViewportTool::Point);
    m_toolLine->setChecked(m_tool == ViewportTool::Line);
    m_shapeButton->setChecked(m_tool == ViewportTool::Rectangle ||
                              m_tool == ViewportTool::Circle);
    m_shapeLabel->setText(shapeName(m_tool));
}