#pragma once

#include "reconstruction/PrimitiveDetector.h"

#include <string>

namespace reconstruction {

// Adapts PointStore observations to CGAL Efficient RANSAC and maps detected
// shapes back to project-owned evidence records. Points without usable normals
// are intentionally left unassigned because Efficient RANSAC requires normals.
bool detectWithCgalEfficientRansac(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error);

}  // namespace reconstruction
