#pragma once

#include "reconstruction/PrimitiveEvidence.h"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace reconstruction {

struct PlaneDetectionOptions {
    double distanceThreshold = 0.1;
    std::size_t minimumSupportPoints = 50;
    std::size_t maximumPlanes = 6;
    double probability = 0.01;
    double minimumNormalAlignment = 0.9;
    // Zero selects a scale-aware value from the input bounding box.
    double clusterEpsilon = 0.0;
    double duplicatePlaneAngularToleranceRadians =
        0.03490658503988659;  // 2 degrees
    double duplicatePlaneDistanceMultiplier = 3.0;
};

struct CylinderDetectionOptions {
    double distanceThreshold = 0.1;
    double minimumNormalAlignment = 0.9;
    std::size_t minimumSupportPoints = 100;
    std::size_t maximumCylinders = 4;
    double probability = 0.01;
    // Zero selects a scale-aware value from the input bounding box.
    double clusterEpsilon = 0.0;
    double minimumRadius = 0.0;
    double maximumRadius = std::numeric_limits<double>::infinity();
};

struct SphereDetectionOptions {
    double distanceThreshold = 0.1;
    double minimumNormalAlignment = 0.9;
    std::size_t minimumSupportPoints = 100;
    std::size_t maximumSpheres = 4;
    double probability = 0.01;
    double clusterEpsilon = 0.0;
    double minimumRadius = 0.0;
    double maximumRadius = std::numeric_limits<double>::infinity();
};

struct ConeDetectionOptions {
    double distanceThreshold = 0.1;
    double minimumNormalAlignment = 0.9;
    std::size_t minimumSupportPoints = 100;
    std::size_t maximumCones = 4;
    double probability = 0.01;
    double clusterEpsilon = 0.0;
    double minimumOpeningAngleRadians = 0.01;
    double maximumOpeningAngleRadians = 1.5607963267948966;
};

struct TorusDetectionOptions {
    double distanceThreshold = 0.1;
    double minimumNormalAlignment = 0.9;
    std::size_t minimumSupportPoints = 100;
    std::size_t maximumTori = 4;
    double probability = 0.01;
    double clusterEpsilon = 0.0;
    double minimumMajorRadius = 0.0;
    double maximumMajorRadius = std::numeric_limits<double>::infinity();
    double minimumMinorRadius = 0.0;
    double maximumMinorRadius = std::numeric_limits<double>::infinity();
};

struct PrimitiveDetectionOptions {
    bool detectPlanes = true;
    bool detectCylinders = true;
    bool detectSpheres = true;
    bool detectCones = true;
    bool detectTori = true;
    PlaneDetectionOptions plane{};
    CylinderDetectionOptions cylinder{};
    SphereDetectionOptions sphere{};
    ConeDetectionOptions cone{};
    TorusDetectionOptions torus{};
};

struct PrimitiveDetectionResult {
    std::vector<PrimitiveEvidence> evidence;
    std::vector<PointId> unassignedPointIds;
};

// Project-owned proposal boundary backed directly by CGAL Efficient RANSAC.
// No CGAL type crosses this API.
bool detectPrimitiveEvidence(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error);

}  // namespace reconstruction
