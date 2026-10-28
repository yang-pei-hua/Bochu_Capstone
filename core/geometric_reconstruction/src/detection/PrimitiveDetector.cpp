#include "reconstruction/detection/PrimitiveDetector.h"

#include "reconstruction/detection/CgalEfficientRansacDetector.h"

namespace reconstruction {

bool detectPrimitiveEvidence(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error) {
    return detectWithCgalEfficientRansac(points, options, output, error);
}

}  // namespace reconstruction
