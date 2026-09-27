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

}  // namespace reconstruction
