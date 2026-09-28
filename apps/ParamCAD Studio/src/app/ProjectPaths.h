#pragma once

#include <QString>

// Every artefact the client produces - captured images, the camera pose
// metadata written beside them, and reconstructed point clouds - belongs under
// the repository's outputs/ folder rather than beside the user's Pictures, so a
// session never scatters files outside the project.
//
// Layout:
//   outputs/captures/<yyyyMMdd_HHmmss>/         PNGs + aggregate manifest + sidecars
//   outputs/reconstructions/<yyyyMMdd_HHmmss>/  cloud.ply + camera_info.json + workspace
//   outputs/screenshots/<yyyyMMdd_HHmmss>.png   single-frame viewport capture
namespace ProjectPaths {

// Repository root: the folder holding core/reconstruction/reconstruct.py.
// Empty when it cannot be located.
QString projectRoot();

// <projectRoot>/outputs
QString outputsRoot();

// <projectRoot>/outputs/captures - one timestamped group per capture batch.
QString capturesRoot();

// <projectRoot>/outputs/reconstructions - one timestamped folder per run.
QString reconstructionsRoot();

// <projectRoot>/outputs/screenshots - one timestamped PNG per single-frame
// viewport capture.
QString screenshotsRoot();

} // namespace ProjectPaths
