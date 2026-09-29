#pragma once

#include "modeling/PrimitiveParams.h"
#include "reconstruction/SurfaceEvidence.h"

#include <vector>

namespace reconstruction {

struct BoxCandidate {
    modeling::BoxPrimitiveParams primitive{};
    std::vector<PlaneEvidence> sourceSurfaces;
    double confidence = 0.0;
};

struct ThroughHoleCandidate {
    modeling::ThroughHolePrimitiveParams primitive{};
    CylinderEvidence sourceSurface{};
    // Distance between the two box boundary planes crossed by the hole.
    double bodyThickness = 0.0;
    double confidence = 0.0;
};

struct BoxWithThroughHolesCandidate {
    BoxCandidate box{};
    std::vector<ThroughHoleCandidate> throughHoles;
    double confidence = 0.0;
};

}  // namespace reconstruction
