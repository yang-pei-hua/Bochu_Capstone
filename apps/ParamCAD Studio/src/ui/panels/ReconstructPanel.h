#pragma once

#include "app/ReconstructionController.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;

// Reconstruct page: which image folder to feed to COLMAP, where the result
// goes, and how far the current run has progressed. Like CapturePanel it only
// collects values and forwards them - the controller owns the process.
class ReconstructPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit ReconstructPanel(QWidget* parent = nullptr);

    QString imageDirectory() const;
    void setImageDirectory(const QString& directory);
    QString outputRoot() const;
    void setOutputRoot(const QString& directory);

    // Where captures are written; "Use Latest Capture" looks for the newest
    // group folder inside it.
    void setCaptureBaseDirectory(const QString& directory);

    void setStatusText(const QString& text);
    void setBusy(bool busy);
    void setProgress(int value, int maximum);

signals:
    void reconstructRequested(const ReconstructRequest& request);
    void cancelRequested();

private:
    void browseForImages();
    void browseForOutput();
    void useLatestCapture();
    void startReconstruction();
    QString latestCaptureDirectory() const;

    QString m_captureBaseDirectory;
    QLineEdit* m_imageDirectory = nullptr;
    QPushButton* m_browseImagesButton = nullptr;
    QPushButton* m_useLatestButton = nullptr;
    QLineEdit* m_outputDirectory = nullptr;
    QPushButton* m_browseOutputButton = nullptr;
    QCheckBox* m_useCameraInfo = nullptr;
    QCheckBox* m_dense = nullptr;
    QComboBox* m_matcher = nullptr;
    QPushButton* m_runButton = nullptr;
    QPushButton* m_cancelButton = nullptr;
    QProgressBar* m_progress = nullptr;
    QLabel* m_status = nullptr;
};