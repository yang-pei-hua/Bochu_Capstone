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

}  // namespace reconstruction
