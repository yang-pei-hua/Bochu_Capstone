#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>

class QTimer;

// What the reconstruction panel asks for. Everything else (run directory,
// workspace, camera info) is derived from it.
struct ReconstructRequest
{
    QString imageDirectory;
    QString outputRoot;
    bool useCameraInfo = true;
    bool dense = false;
    QString matcher = QStringLiteral("exhaustive"); // "exhaustive" | "sequential"
};

// Drives core/reconstruction/reconstruct.py as a child process. All COLMAP
// orchestration stays in that Python package; this class only prepares the run
// directory, streams progress out of colmap.log and reports the result.
class ReconstructionController final : public QObject
{
    Q_OBJECT

public:
    explicit ReconstructionController(QObject* parent = nullptr);

    bool isRunning() const noexcept;
    // True from cancel() until the next start(): lets the UI tell a cancelled
    // run apart from a real failure, which share the failed() signal.
    bool wasCancelled() const noexcept;
    const QString& lastError() const noexcept;
    QString runDirectory() const;

    // Returns false when the request cannot be started; lastError() explains.
    bool start(const ReconstructRequest& request);
    void cancel();

signals:
    void started(const QString& runDirectory, const QString& cameraModeHint);
    void progressChanged(int stage, int totalStages, const QString& label);
    void logMessage(const QString& line);
    void finished(const QString& pointCloudPath, int pointCount,
                  const QString& cameraMode, bool dense);
    void failed(const QString& message);

private:
    QString resolveRepoRoot() const;
    QString resolvePython(QStringList& prefixArgs) const;
    QString createRunDirectory(const QString& outputRoot) const;
    bool buildArguments(const ReconstructRequest& request, QStringList& arguments,
                        QString& cameraModeHint);
    void pollColmapLog();
    void parseStdout();
    void handleStdoutLine(const QString& line);
    void handleFinished(int exitCode, QProcess::ExitStatus status);
    void reportFailure(const QString& message);

    QProcess* m_process = nullptr;
    QTimer* m_pollTimer = nullptr;
    QString m_repoRoot;
    QString m_runDirectory;
    QString m_outputPath;
    QString m_cameraInfoNote;
    QString m_lastError;
    QString m_stdoutBuffer;
    QString m_lastOutputLine;
    QString m_pointCloudPath;
    QString m_cameraMode;
    int m_pointCount = 0;
    int m_lastStage = 0;
    qint64 m_logOffset = 0;
    bool m_dense = false;
    bool m_running = false;
    bool m_cancelled = false;
};