#pragma once

#include "reconstruction/PointStore.h"

#include <cstdint>
#include <vector>

namespace reconstruction {

using SurfaceId = std::uint64_t;
inline constexpr SurfaceId kInvalidSurfaceId = 0;

struct PlaneEquation {
    modeling::Vec3 normal{0.0, 0.0, 1.0};
    double offset = 0.0;  // normal dot position + offset = 0
};

struct PlaneEvidence {
    SurfaceId id = kInvalidSurfaceId;
    PlaneEquation plane{};
    std::vector<PointId> supportPointIds;
    double meanAbsoluteResidual = 0.0;
    double maxAbsoluteResidual = 0.0;
    double coverage = 0.0;
    double confidence = 0.0;
};

struct CylinderEquation {
    // Closest point on the infinite axis to the world origin.
    modeling::Vec3 axisPoint{};
    modeling::Vec3 axisDirection{0.0, 0.0, 1.0};
    double radius = 0.0;
};

struct CylinderEvidence {
    SurfaceId id = kInvalidSurfaceId;
    CylinderEquation cylinder{};
    std::vector<PointId> supportPointIds;
    double axialMinimum = 0.0;
    double axialMaximum = 0.0;
    double angularCoverage = 0.0;
    double meanAbsoluteResidual = 0.0;
    double maxAbsoluteResidual = 0.0;
    double coverage = 0.0;
    double confidence = 0.0;
};

struct SphereEquation {
    modeling::Vec3 center{};
    double radius = 0.0;
};

struct SphereEvidence {
    SurfaceId id = kInvalidSurfaceId;
    SphereEquation sphere{};
    std::vector<PointId> supportPointIds;
    double meanAbsoluteResidual = 0.0;
    double maxAbsoluteResidual = 0.0;
    double coverage = 0.0;
    double confidence = 0.0;
};

struct ConeEquation {
    modeling::Vec3 apex{};
    modeling::Vec3 axisDirection{0.0, 0.0, 1.0};
    double openingAngleRadians = 0.0;
};

struct ConeEvidence {
    SurfaceId id = kInvalidSurfaceId;
    ConeEquation cone{};
    std::vector<PointId> supportPointIds;
    double axialMinimum = 0.0;
    double axialMaximum = 0.0;
    double angularCoverage = 0.0;
    double meanAbsoluteResidual = 0.0;
    double maxAbsoluteResidual = 0.0;
    double coverage = 0.0;
    double confidence = 0.0;
};

struct TorusEquation {
    modeling::Vec3 center{};
    modeling::Vec3 axisDirection{0.0, 0.0, 1.0};
    double majorRadius = 0.0;
    double minorRadius = 0.0;
};

struct TorusEvidence {
    SurfaceId id = kInvalidSurfaceId;
    TorusEquation torus{};
    std::vector<PointId> supportPointIds;
    double meanAbsoluteResidual = 0.0;
    double maxAbsoluteResidual = 0.0;
    double coverage = 0.0;
    double confidence = 0.0;
};

}  // namespace reconstruction
