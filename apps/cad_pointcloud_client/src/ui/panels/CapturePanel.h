#pragma once

#include "app/CaptureController.h"

#include <QWidget>

#include <vector>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTreeWidget;

// Capture page: where shots go, how many orbit steps to take, and what has been
// captured so far. It only collects values and forwards them - the controller
// owns the camera and the files.
class CapturePanel final : public QWidget
{
    Q_OBJECT

public:
    explicit CapturePanel(QWidget* parent = nullptr);

    QString baseDirectory() const;
    void setBaseDirectory(const QString& directory);

    // Orbit distance is seeded from the live camera once at start-up; the user
    // can still change it afterwards.
    void setDefaultDistance(double distance);

    void setShots(const std::vector<CaptureShot>& shots);
    void setStatusText(const QString& text);
    void setBusy(bool busy);

signals:
    void baseDirectoryChanged(const QString& directory);
    void capturePhotoRequested();
    void captureOrbitRequested(int count, double elevationDeg, double distance);

private:
    void browseForDirectory();
    void updateDirectoryLabel();

    QString m_baseDirectory;
    QLineEdit* m_directoryLabel = nullptr;
    QPushButton* m_browseButton = nullptr;
    QSpinBox* m_count = nullptr;
    QDoubleSpinBox* m_elevation = nullptr;
    QDoubleSpinBox* m_distance = nullptr;
    QPushButton* m_photoButton = nullptr;
    QPushButton* m_orbitButton = nullptr;
    QTreeWidget* m_shots = nullptr;
    QLabel* m_status = nullptr;
    QLabel* m_aspectHint = nullptr;
};