#pragma once

#include <QWidget>

class PointCloudSelection;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;

class PointCloudPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudPanel(QWidget* parent = nullptr);
    void refresh(const PointCloudSelection& selection, bool available, int mode);
    double radius() const;
    bool addBoxSelection() const;
    QString geometry() const;
    QString regionName() const;
    int regionId() const;
    void setDefaultRadius(double radius);
    void setTargetRegion(int id);
    void setReconstructionBusy(bool busy);

signals:
    void modeChanged(int mode); // 0: modeling, 1: point picking, 2: 3D box
    void boxRequested();
    void resetBoxRequested();
    void expandRequested();
    void clearRequested();
    void assignRequested();
    void reconstructRequested();
    void recallRequested();
    void removeRegionRequested();
    void unassignRequested();
    void saveRequested();
    void loadRequested();

private:
    QComboBox* m_mode;
    QDoubleSpinBox* m_radius;
    QCheckBox* m_add;
    QComboBox* m_geometry;
    QLineEdit* m_name;
    QComboBox* m_regions;
    QLabel* m_count;
    QPushButton* m_box;
    QPushButton* m_resetBox;
    QPushButton* m_expand;
    QPushButton* m_clear;
    QPushButton* m_assign;
    QPushButton* m_recall;
    QPushButton* m_remove;
    QPushButton* m_unassign;
    QPushButton* m_reconstruct;
};
