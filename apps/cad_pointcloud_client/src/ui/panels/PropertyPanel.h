#pragma once

#include <QColor>
#include <QWidget>

#include <array>

class CameraPanel;
class CapturePanel;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QTabWidget;
class ReconstructPanel;
class RenderPanel;

class PropertyPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit PropertyPanel(QWidget* parent = nullptr);

    CameraPanel* cameraPanel() const;
    RenderPanel* renderPanel() const;
    CapturePanel* capturePanel() const;
    ReconstructPanel* reconstructPanel() const;

    void showCapturePanel(bool visible);
    bool isCapturePanelVisible() const;
    void showReconstructPanel(bool visible);
    bool isReconstructPanelVisible() const;

public slots:
    void showObjectProperties();
    void showCameraProperties();

signals:
    void objectTransformChanged(double px, double py, double pz,
                                double rx, double ry, double rz,
                                double sx, double sy, double sz);
    void objectVisibilityChanged(bool visible);
    void objectRepresentationChanged(int representation);
    void objectOpacityChanged(double opacity);
    void objectColorChanged(const QColor& color);

private:
    QWidget* createObjectTab();
    void emitTransform();
    void chooseObjectColor();
    void updateColorButton();

    QTabWidget* m_tabs = nullptr;
    QWidget* m_objectTab = nullptr;
    CameraPanel* m_cameraPanel = nullptr;
    RenderPanel* m_renderPanel = nullptr;
    CapturePanel* m_capturePanel = nullptr;
    ReconstructPanel* m_reconstructPanel = nullptr;

    std::array<QDoubleSpinBox*, 3> m_position{};
    std::array<QDoubleSpinBox*, 3> m_rotation{};
    std::array<QDoubleSpinBox*, 3> m_scale{};
    QCheckBox* m_visible = nullptr;
    QComboBox* m_representation = nullptr;
    QDoubleSpinBox* m_opacity = nullptr;
    QPushButton* m_colorButton = nullptr;
    QColor m_objectColor{71, 145, 214};
};
