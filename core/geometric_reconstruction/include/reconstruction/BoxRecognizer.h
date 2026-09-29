#pragma once

#include "reconstruction/PrimitiveDetector.h"
#include "reconstruction/ReconstructionCandidate.h"

#include <string>
#include <vector>

namespace reconstruction {

struct BoxRecognitionOptions {
    double angularToleranceRadians = 0.08726646259971647;  // 5 degrees
};

// Combines six fitted planes into three parallel pairs whose directions are
// mutually perpendicular, then derives an oriented semantic box.
bool recognizeBoxFromPlanes(
    const PointStore& points,
    const std::vector<PlaneEvidence>& planes,
    const BoxRecognitionOptions& options,
    BoxCandidate& output,
    std::string& error);

bool reconstructBox(
    const PointStore& points,
    const PlaneDetectionOptions& planeOptions,
    const BoxRecognitionOptions& boxOptions,
    BoxCandidate& output,
    std::string& error);

}  // namespace reconstruction
