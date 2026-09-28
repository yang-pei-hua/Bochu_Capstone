#pragma once

#include "core/CameraController.h"
#include "core/OrbitCamera.h"

#include <QObject>
#include <QString>

#include <vector>

class VTKViewer;

// One captured shot: the image it produced plus the camera pose behind it.
struct CaptureShot
{
    int index = 0;              // 1-based position in this session's shot list
    QString groupName;          // timestamped sub-directory the image lives in
    QString imageName;
    double azimuthDeg = 0.0;
    double elevationDeg = 0.0;
    double distance = 0.0;
};

// One elevation ring of an orbit: how many azimuth steps are taken and the
// height they are taken from. A group usually holds one ring, but a second one
// below the horizon lets the same group also see the under-side of the model.
struct OrbitRing
{
    int count = 0;
    double elevationDeg = 0.0;
};

// Turns camera poses into a folder of images plus machine-readable metadata.
// The viewer stays the single source of truth for the camera; this class only
// drives it, records what it saw, and restores the pose afterwards.
class CaptureController final : public QObject
{
    Q_OBJECT

public:
    explicit CaptureController(VTKViewer* viewer, QObject* parent = nullptr);

    void setBaseDirectory(const QString& directory);
    QString baseDirectory() const;

    // Distance of the live camera from its target, used to seed the orbit UI.
    double currentDistance() const;

    const std::vector<CaptureShot>& shots() const noexcept;
    const QString& lastError() const noexcept;

    bool captureSingle(int width, int height);
    // Every ring lands in one timestamped group, with shot numbers and file
    // names running on across the rings so the group stays a single sequence.
    bool captureOrbit(const std::vector<OrbitRing>& rings, double distance,
                      int width, int height);

signals:
    void shotsChanged();
    void progressChanged(const QString& message);
    void batchStateChanged(bool running);
    void captureFailed(const QString& message);

private:
    // Creates a fresh timestamped sub-directory and returns its absolute path,
    // or an empty string after reporting the failure.
    QString createGroupDirectory();
    bool writeSidecar(const QString& directory, const CaptureShot& shot,
                      const CameraParameters& camera, int width, int height);
    bool writeManifest(const QString& directory, const QString& groupName,
                       const QString& mode, const OrbitParameters& orbit, bool hasOrbit,
                       const std::vector<OrbitRing>& rings,
                       const std::vector<CaptureShot>& groupShots,
                       int width, int height);

    VTKViewer* m_viewer = nullptr;
    QString m_baseDirectory;
    std::vector<CaptureShot> m_shots;
    QString m_lastError;
    bool m_batchRunning = false;
};