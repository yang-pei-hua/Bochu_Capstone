#include "detail/RansacEvidence.h"

namespace reconstruction::detail::cgal {
bool validPlaneOptions(const PlaneDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        options.minimumSupportPoints >= 3U && options.maximumPlanes > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.duplicatePlaneAngularToleranceRadians) &&
        options.duplicatePlaneAngularToleranceRadians > 0.0 &&
        options.duplicatePlaneAngularToleranceRadians < kPi / 2.0 &&
        std::isfinite(options.duplicatePlaneDistanceMultiplier) &&
        options.duplicatePlaneDistanceMultiplier >= 1.0;
}

bool validCylinderOptions(const CylinderDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 2U &&
        options.maximumCylinders > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumRadius) &&
        options.minimumRadius >= 0.0 &&
        options.maximumRadius > options.minimumRadius;
}

bool validSphereOptions(const SphereDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 3U && options.maximumSpheres > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumRadius) &&
        options.minimumRadius >= 0.0 &&
        options.maximumRadius > options.minimumRadius;
}

bool validConeOptions(const ConeDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 3U && options.maximumCones > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumOpeningAngleRadians) &&
        std::isfinite(options.maximumOpeningAngleRadians) &&
        options.minimumOpeningAngleRadians > 0.0 &&
        options.maximumOpeningAngleRadians < kPi / 2.0 &&
        options.maximumOpeningAngleRadians >
            options.minimumOpeningAngleRadians;
}

bool validTorusOptions(const TorusDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 4U && options.maximumTori > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumMajorRadius) &&
        options.minimumMajorRadius >= 0.0 &&
        options.maximumMajorRadius > options.minimumMajorRadius &&
        std::isfinite(options.minimumMinorRadius) &&
        options.minimumMinorRadius >= 0.0 &&
        options.maximumMinorRadius > options.minimumMinorRadius;
}
} // namespace reconstruction::detail::cgal
