#pragma once

#include <QColor>
#include <QString>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QSpinBox;
class QPushButton;

class RenderPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit RenderPanel(QWidget* parent = nullptr);

    int outputWidth() const;
    int outputHeight() const;

    // Texture controls. The panel only reports the intent: the window owns the
    // file dialog and the viewport owns the texture itself, so these two are the
    // viewport's state pushed back into the widgets.
    void setTexturePath(const QString& path);
    void setTextureEnabled(bool enabled);
    int textureProjection() const;

signals:
    void backgroundColorChanged(const QColor& color);
    void lightingChanged(bool enabled);
    void captureRequested();

    void textureBrowseRequested();
    void textureEnabledChanged(bool enabled);
    void textureCleared();
    void textureProjectionChanged(int axis);

private:
    void chooseBackgroundColor();
    void updateColorButton();

    QColor m_backgroundColor{26, 28, 33};
    QPushButton* m_backgroundButton = nullptr;
    QCheckBox* m_lighting = nullptr;
    QSpinBox* m_width = nullptr;
    QSpinBox* m_height = nullptr;
    QLabel* m_texturePath = nullptr;
    QCheckBox* m_textureEnabled = nullptr;
    QComboBox* m_textureAxis = nullptr;
};
