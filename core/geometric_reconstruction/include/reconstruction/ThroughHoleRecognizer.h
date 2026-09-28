#pragma once

#include "reconstruction/BoxRecognizer.h"
#include "reconstruction/ReconstructionCandidate.h"

#include <string>
#include <vector>

namespace reconstruction {

struct ThroughHoleRecognitionOptions {
    double angularToleranceRadians = 0.08726646259971647;  // 5 degrees
    double minimumAngularCoverage = 0.70;
    double minimumAxialCoverage = 0.85;
    // Each observed cylinder end may be this fraction of the body thickness
    // away from its corresponding exterior face.
    double maximumEndpointGapFraction = 0.10;
    // Added to the hole radius when testing containment in the two transverse
    // box dimensions.
    double minimumBoundaryClearance = 0.0;
};

// Classifies cylinder surface evidence against an already recognized box. A
// cylinder is accepted only when it is aligned to one box axis, lies fully in
// the transverse bounds, and its observed side wall reaches both exterior
// faces. Short cylinders are therefore not mislabeled as through holes.
bool recognizeThroughHoles(
    const BoxCandidate& box,
    const std::vector<CylinderEvidence>& cylinders,
    const ThroughHoleRecognitionOptions& options,
    std::vector<ThroughHoleCandidate>& output,
    std::string& error);

// End-to-end box-plus-hole path used by the client. Plane and cylinder
// proposals are detected independently so their RANSAC assignments cannot
// hide one another.
bool reconstructBoxWithThroughHoles(
    const PointStore& points,
    const PlaneDetectionOptions& planeOptions,
    const CylinderDetectionOptions& cylinderOptions,
    const BoxRecognitionOptions& boxOptions,
    const ThroughHoleRecognitionOptions& holeOptions,
    BoxWithThroughHolesCandidate& output,
    std::string& error);

}  // namespace reconstruction
