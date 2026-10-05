#pragma once
#include "reconstruction/PrimitiveDetector.h"
#include "modeling/PrimitiveParams.h"
#include <optional>
namespace reconstruction {
struct CurvedSolidRecognitionOptions {
    double distanceTolerance = 0.1;
    double minimumSurfaceCoverage = 0.90;
    double minimumBoundaryCoverage = 0.90;
    double minimumNormalAlignment = 0.80;
    std::uint32_t randomSeed = 4500;
    std::optional<PrimitiveKind> requiredKind;
    std::vector<PrimitiveEvidence> seeds;
};
struct CurvedSolidCandidate {
    modeling::CurvedPrimitiveParams primitive;
    double surfaceCoverage = 0;
    double boundaryCoverage = 0;
    double meanResidual = 0;
};
// Recognizes one complete, independent curved solid. Caps must be observed;
// angular boundary coverage prevents completion of unsupported partial scans.
bool reconstructCurvedSolid(const PointStore &points, const CurvedSolidRecognitionOptions &options,
                            CurvedSolidCandidate &output, std::string &error);
bool matchesCurvedBoundary(const modeling::CurvedPrimitiveParams& primitive,
                          const PointSample& point, double tolerance);
} // namespace reconstruction
