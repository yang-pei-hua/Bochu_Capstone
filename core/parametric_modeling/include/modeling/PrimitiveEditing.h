#pragma once
#include "modeling/FeatureParams.h"
#include <array>

namespace modeling {
bool validResizeAnchor(const FeatureParams& params, ResizeAnchor anchor);
// Values: box X/Y/Z lengths; hole diameter; cylinder radius/height. All in mm.
bool adjustedPrimitive(const FeatureParams& original, const std::array<double, 3>& values,
                       ResizeAnchor anchor, FeatureParams& adjusted, std::string& error);
std::array<Vec3, 3> boxAxes(const BoxPrimitiveParams& box);
// Rebase changed analytic geometry at its current bounds without moving the
// rendered body, preserving existing scale factors and truthful size fields.
bool refreshProfileBodyDimensions(ProfileBodyParams& params,std::string& error);
}
