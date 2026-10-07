#pragma once
#include <QWidget>

class PointCloudSelection;
class PointCloudRelationsPanel;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QToolButton;

class PointCloudPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudPanel(QWidget* parent = nullptr);
    void refresh(const PointCloudSelection& selection, bool available, int mode);
    double radius() const;
    bool addBoxSelection() const;
    void setDefaultRadius(double radius);
    PointCloudRelationsPanel* relationsPanel() const { return m_relationsPanel; }

signals:
    void modeChanged(int mode);
    void boxRequested();
    void resetBoxRequested();
    void expandRequested();
    void clearRequested();
    void pointRequested(qint64 id);

private:
    PointCloudRelationsPanel* m_relationsPanel;
    QComboBox* m_mode;
    QDoubleSpinBox* m_radius;
    QCheckBox* m_add;
    QWidget* m_boxTools;
    QWidget* m_neighbourTools;
    QLineEdit* m_pointId;
    QPushButton* m_box;
    QToolButton* m_selectionTools;
    QPushButton* m_expand;
};
