#pragma once

#include "ui/viewport/ViewportTool.h"

#include <QWidget>

class QLabel;
class QToolButton;

// SolidWorks-style tool strip that sits directly against the left edge of the
// 3D viewport while a sketch is open: the drawing tools plus the exit button, so
// the tools only exist for as long as there is a sketch to draw on.
//
// It is shown by the window in sketch mode only. In model mode the sketch plane
// is picked on the datum planes drawn in the viewport itself, which is why this
// strip has no plane buttons of its own.
//
// The palette owns no model state. It reports the user's intent and is told what
// to display.
class SketchToolPalette final : public QWidget
{
    Q_OBJECT

public:
    explicit SketchToolPalette(QWidget* parent = nullptr);

    void setSelectionText(const QString& text);

    void setActiveTool(ViewportTool tool);
    ViewportTool activeTool() const noexcept;

signals:
    void toolChanged(ViewportTool tool);
    void exitSketchRequested();

private:
    QToolButton* addToolButton(const QString& objectName, const QString& text,
                               const QString& tooltip, bool checkable);
    void onSketchToolClicked(ViewportTool tool);
    void applyToolState();

    QLabel* m_selection = nullptr;

    QToolButton* m_sketchSelect = nullptr;
    QToolButton* m_toolPoint = nullptr;
    QToolButton* m_toolLine = nullptr;
    QToolButton* m_shapeButton = nullptr;
    QLabel* m_shapeLabel = nullptr;
    QToolButton* m_exitSketch = nullptr;

    ViewportTool m_tool = ViewportTool::Select;
};