#pragma once

#include <cstdint>

namespace modeling {

using FeatureId = std::uint64_t;
using SketchEntityId = std::uint64_t;

inline constexpr FeatureId kInvalidFeatureId = 0;
inline constexpr SketchEntityId kInvalidSketchEntityId = 0;

}  // namespace modeling
