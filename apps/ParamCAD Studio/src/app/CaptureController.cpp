#include "app/CaptureController.h"

#include "app/ProjectPaths.h"
#include "ui/viewport/VTKViewer.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSize>

namespace
{
constexpr int kSchemaVersion = 1;

QJsonArray toJsonArray(const double values[3])
{
    QJsonArray array;
    for (int index = 0; index < 3; ++index) {
        array.append(values[index]);
    }
    return array;
}

QJsonObject cameraToJson(const CaptureShot& shot, const CameraParameters& camera)
{
    QJsonObject object;
    object.insert(QStringLiteral("position"), toJsonArray(camera.position));
    object.insert(QStringLiteral("target"), toJsonArray(camera.target));
    object.insert(QStringLiteral("up"), toJsonArray(camera.up));
    object.insert(QStringLiteral("azimuthDeg"), shot.azimuthDeg);
    object.insert(QStringLiteral("elevationDeg"), shot.elevationDeg);
    object.insert(QStringLiteral("distance"), shot.distance);
    object.insert(QStringLiteral("fieldOfViewDeg"), camera.fieldOfView);
    object.insert(QStringLiteral("projection"),
                  camera.parallelProjection ? QStringLiteral("orthographic")
                                            : QStringLiteral("perspective"));
    return object;
}

QString sidecarNameFor(const QString& imageName)
{
    QString name = imageName;
    const int dot = name.lastIndexOf(QLatin1Char('.'));
    if (dot >= 0) {
        name.truncate(dot);
    }
    return name + QStringLiteral(".json");
}

bool writeJsonObject(const QString& filePath, const QJsonObject& object, QString& error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        error = QObject::tr("Cannot open %1 for writing").arg(filePath);
        return false;
    }

    const QByteArray payload = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (file.write(payload) == -1) {
        error = QObject::tr("Cannot write %1").arg(filePath);
        return false;
    }
    return true;
}
}

CaptureController::CaptureController(VTKViewer* viewer, QObject* parent)
    : QObject(parent)
    , m_viewer(viewer)
{
    m_baseDirectory = ProjectPaths::capturesRoot();
}

void CaptureController::setBaseDirectory(const QString& directory)
{
    if (!directory.isEmpty()) {
        m_baseDirectory = directory;
    }
}

QString CaptureController::baseDirectory() const
{
    return m_baseDirectory;
}

double CaptureController::currentDistance() const
{
    return m_viewer == nullptr ? OrbitParameters{}.distance
                               : OrbitCamera::fromCamera(m_viewer->cameraParameters()).distance;
}

const std::vector<CaptureShot>& CaptureController::shots() const noexcept
{
    return m_shots;
}

const QString& CaptureController::lastError() const noexcept
{
    return m_lastError;
}

QString CaptureController::createGroupDirectory()
{
    QDir base(m_baseDirectory);
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));

    // Two captures within the same second must not collide.
    QString group = stamp;
    for (int suffix = 2; base.exists(group); ++suffix) {
        group = stamp + QStringLiteral("-") + QString::number(suffix);
    }

    if (!base.mkpath(group)) {
        m_lastError = tr("Cannot create output directory %1").arg(base.filePath(group));
        emit captureFailed(m_lastError);
        return QString();
    }
    return base.filePath(group);
}

bool CaptureController::captureSingle(int width, int height)
{
    if (m_viewer == nullptr || m_batchRunning) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        m_lastError = tr("Output size must be positive");
        emit captureFailed(m_lastError);
        return false;
    }

    const QString directory = createGroupDirectory();
    if (directory.isEmpty()) {
        return false;
    }

    const CameraParameters camera = m_viewer->cameraParameters();
    const OrbitParameters orbit = OrbitCamera::fromCamera(camera);

    CaptureShot shot;
    shot.index = static_cast<int>(m_shots.size()) + 1;
    shot.groupName = QDir(directory).dirName();
    shot.imageName = QStringLiteral("shot_000.png");
    shot.azimuthDeg = orbit.azimuthDeg;
    shot.elevationDeg = orbit.elevationDeg;
    shot.distance = orbit.distance;

    if (!m_viewer->captureImage(directory + QLatin1Char('/') + shot.imageName, width, height)) {
        m_lastError = tr("Failed to write %1").arg(shot.imageName);
        emit captureFailed(m_lastError);
        return false;
    }

    if (!writeSidecar(directory, shot, camera, width, height)) {
        emit captureFailed(m_lastError);
        return false;
    }

    m_shots.push_back(shot);
    emit shotsChanged();

    // A single photo is its own group, so the manifest always describes exactly
    // one directory's worth of shots.
    const std::vector<CaptureShot> group{shot};
    if (!writeManifest(directory, shot.groupName, QStringLiteral("single"), orbit, false,
                       {}, group, width, height)) {
        emit captureFailed(m_lastError);
        return false;
    }

    m_lastError.clear();
    emit progressChanged(tr("Captured 1 photo in %1").arg(shot.groupName));
    return true;
}

bool CaptureController::captureOrbit(const std::vector<OrbitRing>& rings, double distance,
                                     int width, int height)
{
    if (m_viewer == nullptr || m_batchRunning) {
        return false;
    }

    int totalCount = 0;
    for (const OrbitRing& ring : rings) {
        if (ring.count < 1) {
            m_lastError = tr("Capture count and output size must be positive");
            emit captureFailed(m_lastError);
            return false;
        }
        totalCount += ring.count;
    }
    if (totalCount < 1 || width <= 0 || height <= 0) {
        m_lastError = tr("Capture count and output size must be positive");
        emit captureFailed(m_lastError);
        return false;
    }

    const QString directory = createGroupDirectory();
    if (directory.isEmpty()) {
        return false;
    }
    const QString groupName = QDir(directory).dirName();

    // OrthogonalizeViewUp() already ran on this pose, so replaying it afterwards
    // restores the user's exact view.
    const CameraParameters original = m_viewer->cameraParameters();

    m_batchRunning = true;
    emit batchStateChanged(true);

    const int firstIndex = static_cast<int>(m_shots.size());
    std::vector<CaptureShot> groupShots;
    groupShots.reserve(static_cast<std::size_t>(totalCount));

    QString failure;
    int taken = 0;
    for (const OrbitRing& ring : rings) {
        const double step = 360.0 / static_cast<double>(ring.count);
        for (int i = 0; i < ring.count && failure.isEmpty(); ++i) {
            const double azimuth = step * static_cast<double>(i);
            const OrbitParameters orbit{distance, azimuth, ring.elevationDeg};

            m_viewer->applyCamera(OrbitCamera::toCamera(orbit, original, true));

            CaptureShot shot;
            shot.index = firstIndex + taken + 1;
            shot.groupName = groupName;
            shot.imageName = QStringLiteral("shot_%1.png").arg(taken, 3, 10, QLatin1Char('0'));
            shot.azimuthDeg = azimuth;
            shot.elevationDeg = ring.elevationDeg;
            shot.distance = distance;

            const CameraParameters camera = m_viewer->cameraParameters();
            if (!m_viewer->captureImage(directory + QLatin1Char('/') + shot.imageName,
                                        width, height)
                || !writeSidecar(directory, shot, camera, width, height)) {
                failure = tr("Failed to capture %1").arg(shot.imageName);
                break;
            }

            ++taken;
            m_shots.push_back(shot);
            groupShots.push_back(shot);
            emit shotsChanged();
            emit progressChanged(tr("Capturing %1/%2").arg(taken).arg(totalCount));

            // Rendering does not pump the event loop, so the progress label and
            // shot list only repaint if events are flushed here. User input stays
            // excluded so a click cannot re-enter this loop.
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
        if (!failure.isEmpty()) {
            break;
        }
    }

    m_viewer->applyCamera(original);

    // The legacy "orbit" block keeps describing the first ring, which is what a
    // single-ring run has always written; the rings array carries the rest.
    const double firstElevation = rings.empty() ? 0.0 : rings.front().elevationDeg;
    const OrbitParameters manifestOrbit{distance, 0.0, firstElevation};
    const bool manifestWritten = writeManifest(directory, groupName, QStringLiteral("orbit"),
                                               manifestOrbit, true, rings, groupShots,
                                               width, height);

    // Release the busy state before reporting, because the report opens a modal
    // dialog that runs a nested event loop.
    m_batchRunning = false;
    emit batchStateChanged(false);

    if (!failure.isEmpty() || !manifestWritten) {
        m_lastError = failure.isEmpty() ? tr("Failed to write manifest.json") : failure;
        emit captureFailed(m_lastError);
        return false;
    }

    m_lastError.clear();
    emit progressChanged(tr("Captured %1 photos in %2").arg(groupShots.size()).arg(groupName));
    return true;
}

bool CaptureController::writeSidecar(const QString& directory, const CaptureShot& shot,
                                     const CameraParameters& camera,
                                     int width, int height)
{
    QJsonObject root;
    root.insert(QStringLiteral("schema"), QStringLiteral("cadpc.capture.sidecar"));
    root.insert(QStringLiteral("version"), kSchemaVersion);
    root.insert(QStringLiteral("image"), shot.imageName);
    root.insert(QStringLiteral("sidecar"), sidecarNameFor(shot.imageName));
    root.insert(QStringLiteral("group"), shot.groupName);
    root.insert(QStringLiteral("capturedAt"), QDateTime::currentDateTime().toString(Qt::ISODate));
    root.insert(QStringLiteral("camera"), cameraToJson(shot, camera));

    QJsonObject output;
    output.insert(QStringLiteral("width"), width);
    output.insert(QStringLiteral("height"), height);
    root.insert(QStringLiteral("output"), output);

    // captureImage() stretches the render window to the requested output size,
    // so the horizontal and vertical focal lengths of the resulting image
    // differ. Recording the pre-stretch window size lets a reconstruction build
    // correct intrinsics; without it the two are assumed equal.
    const QSize source = m_viewer == nullptr ? QSize() : m_viewer->captureSourceSize();
    if (source.isValid()) {
        QJsonObject render;
        render.insert(QStringLiteral("width"), source.width());
        render.insert(QStringLiteral("height"), source.height());
        root.insert(QStringLiteral("render"), render);
    }

    QString error;
    const QString path = directory + QLatin1Char('/') + sidecarNameFor(shot.imageName);
    if (!writeJsonObject(path, root, error)) {
        m_lastError = error;
        return false;
    }
    return true;
}

bool CaptureController::writeManifest(const QString& directory, const QString& groupName,
                                      const QString& mode, const OrbitParameters& orbit,
                                      bool hasOrbit, const std::vector<OrbitRing>& rings,
                                      const std::vector<CaptureShot>& groupShots,
                                      int width, int height)
{
    QJsonArray shots;
    for (const CaptureShot& shot : groupShots) {
        QJsonObject entry;
        entry.insert(QStringLiteral("index"), shot.index);
        entry.insert(QStringLiteral("image"), shot.imageName);
        entry.insert(QStringLiteral("sidecar"), sidecarNameFor(shot.imageName));
        entry.insert(QStringLiteral("azimuthDeg"), shot.azimuthDeg);
        entry.insert(QStringLiteral("elevationDeg"), shot.elevationDeg);
        shots.append(entry);
    }

    QJsonObject root;
    root.insert(QStringLiteral("schema"), QStringLiteral("cadpc.capture.manifest"));
    root.insert(QStringLiteral("version"), kSchemaVersion);
    root.insert(QStringLiteral("group"), groupName);
    root.insert(QStringLiteral("createdAt"), QDateTime::currentDateTime().toString(Qt::ISODate));
    root.insert(QStringLiteral("mode"), mode);

    if (hasOrbit) {
        // The orbit block describes the first ring alone, so a run without the
        // under-side keeps the exact shape it has always had; the rings array
        // below is what distinguishes a two-ring group.
        const int firstCount = rings.empty() ? static_cast<int>(groupShots.size())
                                             : rings.front().count;
        QJsonObject orbitObject;
        orbitObject.insert(QStringLiteral("count"), firstCount);
        orbitObject.insert(QStringLiteral("startAzimuthDeg"), orbit.azimuthDeg);
        orbitObject.insert(QStringLiteral("stepAzimuthDeg"),
                           firstCount > 0 ? 360.0 / firstCount : 0.0);
        orbitObject.insert(QStringLiteral("elevationDeg"), orbit.elevationDeg);
        orbitObject.insert(QStringLiteral("distance"), orbit.distance);
        root.insert(QStringLiteral("orbit"), orbitObject);

        if (rings.size() > 1) {
            QJsonArray ringArray;
            int firstShot = 0;
            for (const OrbitRing& ring : rings) {
                QJsonObject entry;
                entry.insert(QStringLiteral("count"), ring.count);
                entry.insert(QStringLiteral("elevationDeg"), ring.elevationDeg);
                entry.insert(QStringLiteral("startAzimuthDeg"), 0.0);
                entry.insert(QStringLiteral("stepAzimuthDeg"), 360.0 / ring.count);
                entry.insert(QStringLiteral("firstShotNumber"), firstShot + 1);
                ringArray.append(entry);
                firstShot += ring.count;
            }
            root.insert(QStringLiteral("rings"), ringArray);
        }
    } else {
        root.insert(QStringLiteral("orbit"), QJsonValue::Null);
    }

    root.insert(QStringLiteral("shots"), shots);

    QJsonObject output;
    output.insert(QStringLiteral("width"), width);
    output.insert(QStringLiteral("height"), height);
    root.insert(QStringLiteral("output"), output);

    QString error;
    const QString path = directory + QStringLiteral("/manifest.json");
    if (!writeJsonObject(path, root, error)) {
        m_lastError = error;
        return false;
    }
    return true;
}