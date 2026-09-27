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
};

// Detects disjoint planar support sets. RANSAC proposes each plane, then a
// covariance fit refines it before final inlier assignment.
bool detectPlanes(
    const PointStore& points,
    const PlaneDetectionOptions& options,
    std::vector<PlaneEvidence>& output,
    std::string& error);

}  // namespace reconstruction
