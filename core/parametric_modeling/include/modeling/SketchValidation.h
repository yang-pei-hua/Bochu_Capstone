#pragma once

#include "modeling/FeatureParams.h"

#include <string>

namespace modeling {

// Validates whether a sketch currently defines a planar face suitable for an
// Extrude or Cut. Reference points are ignored. An open sketch is a valid
// document state, but this function returns false until a closed profile exists.
bool validateSketch(const SketchFeatureParams& params, std::string& error);

}  // namespace modeling
