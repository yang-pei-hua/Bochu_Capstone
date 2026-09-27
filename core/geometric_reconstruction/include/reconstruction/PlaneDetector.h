#pragma once

#include "reconstruction/SurfaceEvidence.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace reconstruction {

struct PlaneDetectionOptions {
    double distanceThreshold = 0.1;
    std::size_t minimumSupportPoints = 50;
    std::size_t maximumPlanes = 6;
    std::size_t ransacIterations = 800;
    std::uint32_t randomSeed = 1;

    // Dense reconstructions can contain several thin, parallel layers for one
    // physical surface. Suppress such layers without making the public plane
    // limit large enough for duplicates to crowd out weaker real faces.
    double duplicatePlaneAngularToleranceRadians =
        0.03490658503988659;  // 2 degrees
    double duplicatePlaneDistanceMultiplier = 3.0;
    std::size_t maximumCandidatePlaneMultiplier = 4;
};

// Detects disjoint planar support sets. RANSAC proposes each plane, then a
// covariance fit refines it before final inlier assignment.
bool detectPlanes(
    const PointStore& points,
    const PlaneDetectionOptions& options,
    std::vector<PlaneEvidence>& output,
    std::string& error);

}  // namespace reconstruction
