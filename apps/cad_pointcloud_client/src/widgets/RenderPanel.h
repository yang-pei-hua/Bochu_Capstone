#pragma once

#include <QColor>
#include <QWidget>

class QCheckBox;
class QSpinBox;
class QPushButton;

class RenderPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit RenderPanel(QWidget* parent = nullptr);

    int outputWidth() const;
    int outputHeight() const;

signals:
    void backgroundColorChanged(const QColor& color);
    void lightingChanged(bool enabled);
    void captureRequested();

private:
    void chooseBackgroundColor();
    void updateColorButton();

    QColor m_backgroundColor{26, 28, 33};
    QPushButton* m_backgroundButton = nullptr;
    QCheckBox* m_lighting = nullptr;
    QSpinBox* m_width = nullptr;
    QSpinBox* m_height = nullptr;
};
