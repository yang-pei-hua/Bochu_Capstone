#pragma once

#include "modeling/PrimitiveParams.h"
#include "reconstruction/PointStore.h"

#include <cstddef>

namespace reconstruction {

struct SyntheticBoxRequest {
    modeling::BoxPrimitiveParams box{};
    std::size_t samplesPerEdge = 10;
};

PointStore makeSyntheticBoxSurface(const SyntheticBoxRequest& request);

}  // namespace reconstruction
