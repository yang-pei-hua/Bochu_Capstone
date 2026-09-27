#pragma once

#include "reconstruction/ReconstructionCandidate.h"

#include <string>

namespace reconstruction {

// First vertical-slice recognizer. It deliberately accepts only one complete,
// axis-aligned box; its output contract remains valid when the implementation
// is replaced by plane RANSAC and relation analysis.
bool recognizeAxisAlignedBox(
    const PointStore& points,
    double distanceTolerance,
    BoxCandidate& output,
    std::string& error);

}  // namespace reconstruction
