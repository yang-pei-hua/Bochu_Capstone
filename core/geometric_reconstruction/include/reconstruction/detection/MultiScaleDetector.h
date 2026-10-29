#pragma once
#include "reconstruction/detection/PrimitiveDetector.h"
#include <functional>

namespace reconstruction {
struct MultiScaleDetectionReport {
    std::size_t detectionPasses=0,localRegions=0,recoveredSurfaces=0;
};
// Independent global families followed by normal-compatible local residual
// regions. Only observations and options enter this API; no CAD labels do.
bool detectMultiScalePrimitiveEvidence(const PointStore& points,const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,MultiScaleDetectionReport& report,std::string& error,
    const std::function<bool()>& cancelled={});
}
