#include "app/ReconstructionController.h"

#include "app/ProjectPaths.h"
#include "io/CameraInfoBuilder.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QSettings>
#include <QStandardPaths>
#include <QTimer>

namespace {

// colmap.log records every invocation as "$ colmap <command> <args>", which is
// enough to tell the user which stage the pipeline is in.
struct StageInfo
{
    int stage = 0;
    const char* label = "";
};

StageInfo stageFor(const QString& command, bool dense)
{
    if (command == QLatin1String("feature_extractor")) {
        return {1, "特征提取"};
    }
    if (command == QLatin1String("exhaustive_matcher") || command == QLatin1String("sequential_matcher")) {
        return {2, "特征匹配"};
    }
    if (command == QLatin1String("mapper") || command == QLatin1String("point_triangulator")) {
        return {3, "稀疏重建"};
    }
    if (command == QLatin1String("image_undistorter")) {
        return {4, "图像去畸变"};
    }
    if (command == QLatin1String("patch_match_stereo")) {
        return {5, "稠密深度图"};
    }
    if (command == QLatin1String("stereo_fusion")) {
        return {6, "稠密点云融合"};
    }
    if (command == QLatin1String("model_converter")) {
        // The sparse pipeline writes the cloud directly; the dense one is done
        // by stereo_fusion afterwards.
        return {dense ? 6 : 4, "导出 PLY"};
    }
    return {0, ""};
}

const int kSparseStages = 4;
const int kDenseStages = 6;
const int kPollIntervalMs = 400;

} // namespace

ReconstructionController::ReconstructionController(QObject* parent)
    : QObject(parent)
{
    m_process = new QProcess(this);
    m_process->setProcessChannelMode(QProcess::MergedChannels);

    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    // Unbuffered so progress arrives while COLMAP runs, and UTF-8 so non-ASCII
    // paths survive the round trip.
    environment.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    environment.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
    m_process->setProcessEnvironment(environment);

    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(kPollIntervalMs);

    connect(m_pollTimer, &QTimer::timeout, this, &ReconstructionController::pollColmapLog);
    connect(m_process, &QProcess::readyReadStandardOutput, this, &ReconstructionController::parseStdout);
    connect(m_process, &QProcess::finished, this, &ReconstructionController::handleFinished);
}

bool ReconstructionController::isRunning() const noexcept
{
    return m_running;
}

bool ReconstructionController::wasCancelled() const noexcept
{
    return m_cancelled;
}

const QString& ReconstructionController::lastError() const noexcept
{
    return m_lastError;
}

QString ReconstructionController::runDirectory() const
{
    return m_runDirectory;
}

QString ReconstructionController::resolveRepoRoot() const
{
    return ProjectPaths::projectRoot();
}

QString ReconstructionController::resolvePython(QStringList& prefixArgs) const
{
    const QSettings settings;
    const QString configured = settings.value(QStringLiteral("reconstruction/pythonInterpreter")).toString();
    if (!configured.isEmpty() && QFileInfo::exists(configured)) {
        return configured;
    }

    const QString python = QStandardPaths::findExecutable(QStringLiteral("python"));
    if (!python.isEmpty()) {
        return python;
    }
    const QString python3 = QStandardPaths::findExecutable(QStringLiteral("python3"));
    if (!python3.isEmpty()) {
        return python3;
    }
    const QString launcher = QStandardPaths::findExecutable(QStringLiteral("py"));
    if (!launcher.isEmpty()) {
        prefixArgs << QStringLiteral("-3");
        return launcher;
    }
    return QString();
}

QString ReconstructionController::createRunDirectory(const QString& outputRoot) const
{
    if (!QDir().mkpath(outputRoot)) {
        return QString();
    }
    const QDir root(outputRoot);
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));

    // Two runs within the same second must not collide, and the pipeline
    // refuses to overwrite an existing output or a non-empty workspace.
    QString name = stamp;
    for (int suffix = 2; root.exists(name); ++suffix) {
        name = stamp + QStringLiteral("-") + QString::number(suffix);
    }
    if (!root.mkpath(name)) {
        return QString();
    }
    return root.filePath(name);
}

bool ReconstructionController::buildArguments(const ReconstructRequest& request,
                                              QStringList& arguments, QString& cameraModeHint)
{
    const QString imageDirectory = QDir::cleanPath(request.imageDirectory);
    m_outputPath = m_runDirectory + QStringLiteral("/cloud.ply");

    arguments << QStringLiteral("--images") << imageDirectory;
    arguments << QStringLiteral("--output") << m_outputPath;
    arguments << QStringLiteral("--workspace") << (m_runDirectory + QStringLiteral("/workspace"));
    if (request.matcher == QLatin1String("sequential")) {
        arguments << QStringLiteral("--matcher") << QStringLiteral("sequential");
    }
    if (request.dense) {
        arguments << QStringLiteral("--dense") << QStringLiteral("--gpu");
    }

    cameraModeHint = QStringLiteral("estimated");
    if (!request.useCameraInfo) {
        return true;
    }

    const QString cameraInfoPath = m_runDirectory + QStringLiteral("/camera_info.json");
    QString error;
    const CameraInfoBuildResult cameraInfo =
        CameraInfoBuilder::build(imageDirectory, cameraInfoPath, error);
    if (!error.isEmpty()) {
        m_lastError = error;
        return false;
    }
    cameraModeHint = cameraInfo.cameraMode;
    if (cameraInfo.built) {
        arguments << QStringLiteral("--camera-info") << cameraInfoPath;
        return true;
    }

    // No usable capture metadata: leave --camera-info off entirely and let
    // COLMAP estimate the cameras, which is a normal outcome.
    m_cameraInfoNote = tr("Camera metadata not used (%1); COLMAP will estimate the cameras")
                           .arg(cameraInfo.skipReason);
    return true;
}

bool ReconstructionController::start(const ReconstructRequest& request)
{
    if (m_running) {
        m_lastError = tr("A reconstruction is already running");
        return false;
    }
    m_lastError.clear();
    m_cameraInfoNote.clear();
    m_lastOutputLine.clear();
    m_stdoutBuffer.clear();
    m_pointCloudPath.clear();
    m_cameraMode.clear();
    m_pointCount = 0;
    m_lastStage = 0;
    m_logOffset = 0;
    m_cancelled = false;
    m_dense = request.dense;

    if (!QFileInfo(request.imageDirectory).isDir()) {
        m_lastError = tr("Image directory does not exist: %1").arg(request.imageDirectory);
        return false;
    }
    if (CameraInfoBuilder::findImages(request.imageDirectory).size() < 2) {
        m_lastError = tr("At least two images are needed in %1").arg(request.imageDirectory);
        return false;
    }
    if (request.outputRoot.isEmpty()) {
        m_lastError = tr("No output directory was given");
        return false;
    }

    m_repoRoot = resolveRepoRoot();
    if (m_repoRoot.isEmpty()) {
        m_lastError = tr("Cannot locate core/reconstruction/reconstruct.py above %1")
                          .arg(QCoreApplication::applicationDirPath());
        return false;
    }

    QStringList prefixArgs;
    const QString python = resolvePython(prefixArgs);
    if (python.isEmpty()) {
        m_lastError = tr("No Python 3 interpreter was found. Install Python or set "
                         "reconstruction/pythonInterpreter.");
        return false;
    }

    m_runDirectory = createRunDirectory(request.outputRoot);
    if (m_runDirectory.isEmpty()) {
        m_lastError = tr("Cannot create a run directory under %1").arg(request.outputRoot);
        return false;
    }

    QString cameraModeHint;
    QStringList arguments;
    if (!buildArguments(request, arguments, cameraModeHint)) {
        return false;
    }

    arguments.prepend(m_repoRoot + QStringLiteral("/core/reconstruction/reconstruct.py"));
    arguments = prefixArgs + arguments;

    // Set program and arguments separately so paths with spaces or non-ASCII
    // characters never go through a shell.
    m_process->setProgram(python);
    m_process->setArguments(arguments);
    m_process->setWorkingDirectory(m_repoRoot);

    m_running = true;
    m_process->start();
    if (!m_process->waitForStarted(5000)) {
        const QString message = tr("Cannot start %1: %2").arg(python, m_process->errorString());
        m_running = false;
        reportFailure(message);
        return false;
    }

    m_pollTimer->start();
    emit started(m_runDirectory, cameraModeHint);
    emit progressChanged(0, m_dense ? kDenseStages : kSparseStages, tr("Starting"));
    if (!m_cameraInfoNote.isEmpty()) {
        emit logMessage(m_cameraInfoNote);
    }
    return true;
}

void ReconstructionController::cancel()
{
    if (!m_running || m_process == nullptr) {
        return;
    }
    m_cancelled = true;

    // Killing the Python process does not stop the colmap.exe it spawned, so the
    // whole tree has to go. taskkill walks the parent/child chain, which only
    // exists while Python is still alive: killing Python first would orphan
    // colmap.exe and leave it running, so taskkill has to finish first.
#if defined(Q_OS_WIN)
    const qint64 pid = m_process->processId();
    if (pid > 0) {
        QProcess::execute(QStringLiteral("taskkill"),
                          {QStringLiteral("/F"), QStringLiteral("/T"),
                           QStringLiteral("/PID"), QString::number(pid)});
    }
#endif
    m_process->kill();
}

void ReconstructionController::pollColmapLog()
{
    if (m_runDirectory.isEmpty()) {
        return;
    }
    QFile log(m_runDirectory + QStringLiteral("/workspace/colmap.log"));
    if (!log.open(QIODevice::ReadOnly)) {
        return;
    }
    const qint64 size = log.size();
    if (size <= m_logOffset || !log.seek(m_logOffset)) {
        return;
    }
    const QByteArray data = log.read(size - m_logOffset);
    const QString text = QString::fromUtf8(data);
    if (!text.endsWith(QLatin1Char('\n'))) {
        // A partially written line is picked up on the next poll.
        const int lastBreak = text.lastIndexOf(QLatin1Char('\n'));
        if (lastBreak < 0) {
            return;
        }
        m_logOffset += static_cast<qint64>(text.left(lastBreak + 1).toUtf8().size());
        return;
    }
    m_logOffset = size;

    static const QRegularExpression commandPattern(QStringLiteral("^\\s*\\$\\s*colmap\\s+(\\S+)"));
    for (const QString& rawLine : text.split(QLatin1Char('\n'))) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        emit logMessage(line);
        const QRegularExpressionMatch match = commandPattern.match(line);
        if (!match.hasMatch()) {
            continue;
        }
        const StageInfo info = stageFor(match.captured(1), m_dense);
        if (info.stage == 0 || info.stage == m_lastStage) {
            continue;
        }
        m_lastStage = info.stage;
        emit progressChanged(info.stage, m_dense ? kDenseStages : kSparseStages,
                             QString::fromUtf8(info.label));
    }
}

void ReconstructionController::parseStdout()
{
    if (m_process == nullptr) {
        return;
    }
    m_stdoutBuffer += QString::fromUtf8(m_process->readAllStandardOutput());
    int newline = m_stdoutBuffer.indexOf(QLatin1Char('\n'));
    while (newline >= 0) {
        const QString line = m_stdoutBuffer.left(newline);
        m_stdoutBuffer.remove(0, newline + 1);
        handleStdoutLine(line.trimmed());
        newline = m_stdoutBuffer.indexOf(QLatin1Char('\n'));
    }
}

void ReconstructionController::handleStdoutLine(const QString& line)
{
    if (line.isEmpty()) {
        return;
    }
    m_lastOutputLine = line;

    const QStringList markers{QStringLiteral("point cloud: "), QStringLiteral("points: "),
                              QStringLiteral("camera mode: "), QStringLiteral("manifest: ")};
    for (const QString& marker : markers) {
        if (!line.startsWith(marker)) {
            continue;
        }
        const QString value = line.mid(marker.size()).trimmed();
        if (marker.startsWith(QStringLiteral("point cloud"))) {
            m_pointCloudPath = value;
        } else if (marker.startsWith(QStringLiteral("points"))) {
            m_pointCount = value.toInt();
        } else if (marker.startsWith(QStringLiteral("camera mode"))) {
            m_cameraMode = value;
        }
        return;
    }
    emit logMessage(line);
}

void ReconstructionController::handleFinished(int exitCode, QProcess::ExitStatus status)
{
    if (!m_running) {
        return;
    }
    m_pollTimer->stop();

    parseStdout();
    if (!m_stdoutBuffer.trimmed().isEmpty()) {
        handleStdoutLine(m_stdoutBuffer.trimmed());
        m_stdoutBuffer.clear();
    }
    // The final stage may have been written just before the process exited.
    pollColmapLog();

    m_running = false;
    if (m_cancelled) {
        reportFailure(tr("Reconstruction cancelled"));
        return;
    }
    if (exitCode != 0 || status == QProcess::CrashExit) {
        reportFailure(m_lastOutputLine.isEmpty()
                          ? tr("Reconstruction failed with exit code %1").arg(exitCode)
                          : m_lastOutputLine);
        return;
    }

    emit finished(m_pointCloudPath.isEmpty() ? m_outputPath : m_pointCloudPath,
                  m_pointCount, m_cameraMode, m_dense);
}

void ReconstructionController::reportFailure(const QString& message)
{
    m_lastError = message;
    emit failed(message);
}