#pragma once

#include <QString>
#include <QStringList>

// Outcome of turning a capture folder into a COLMAP CameraInfo file. When the
// folder cannot be described by the capture metadata the caller leaves
// --camera-info off entirely and lets COLMAP estimate the cameras instead; that
// is a normal result, not a failure.
struct CameraInfoBuildResult
{
    bool built = false;
    int totalImages = 0;
    int posedImages = 0;
    QString cameraMode; // "provided" | "estimated"
    QString skipReason; // set when built is false
};

// Converts capture sidecars (camera centre, look-at target, up vector, vertical
// field of view) into the CameraInfo JSON that core/reconstruction consumes.
//
// COLMAP camera axes are +X right, +Y down, +Z forward, so the rotation stored
// as camera_to_world maps the camera basis onto the world basis.
class CameraInfoBuilder
{
public:
    // Image file names in directory, sorted; the extension set matches COLMAP.
    static QStringList findImages(const QString& directory);

    // Writes outputJsonPath when every image has a usable perspective sidecar
    // and there are at least two of them. Returns false only for a real error.
    static CameraInfoBuildResult build(const QString& imageDirectory,
                                       const QString& outputJsonPath,
                                       QString& error);
};