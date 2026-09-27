#pragma once

#include "modeling/PrimitiveParams.h"
#include "reconstruction/PointStore.h"

#include <cstddef>
#include <cstdint>

namespace reconstruction {

struct SyntheticBoxRequest {
    modeling::BoxPrimitiveParams box{};
    std::size_t samplesPerEdge = 10;
    double gaussianNoiseSigma = 0.0;
    std::size_t outlierCount = 0;
    double outlierPaddingFraction = 0.25;
    std::uint32_t randomSeed = 1;
};

PointStore makeSyntheticBoxSurface(const SyntheticBoxRequest& request);

}  // namespace reconstruction
