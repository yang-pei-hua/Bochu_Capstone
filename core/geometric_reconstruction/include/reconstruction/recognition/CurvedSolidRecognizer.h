#pragma once
#include "reconstruction/detection/PrimitiveDetector.h"
#include "reconstruction/analysis/SurfaceAnalysis.h"
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

    // Quality gates and sampling budgets that were previously embedded in the
    // curved recognizer and its boundary assessment.
    double boundaryNormalCosine = 0.8;
    double capNormalCosine = 0.98;
    double assessDistanceMultiplier = 2;
    double apexToleranceMultiplier = 3;
    double capRadialToleranceMultiplier = 2;
    double capGroupToleranceMultiplier = 4;
    std::size_t minimumCapPoints = 12;
    std::size_t capPointDivisor = 1000;
    std::size_t sphereAngularBins = 12;
    std::size_t sphereAxialBins = 6;
    std::size_t torusAngularBins = 24;
    std::size_t torusTubeBins = 12;
    std::size_t curvedSideAngularBins = 24;
    std::size_t curvedSideAxialBins = 6;
    std::size_t capAngularBins = 8;
    std::size_t capRadialBins = 2;
    std::size_t curvedCylinderSupportFloor = 30;
    std::size_t curvedCylinderSupportDivisor = 8;
    double curvedCylinderProbability = 0.001;
    double coneStartToleranceMultiplier = 2;
    double residualScoreWeight = 1;
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
bool recognizeCurvedSolidFromAnalysis(const PointStore& points, const SurfaceAnalysisResult& analysis,
                                     const CurvedSolidRecognitionOptions& options,
                                     CurvedSolidCandidate& output, std::string& error);
bool matchesCurvedBoundary(const modeling::CurvedPrimitiveParams& primitive,
                          const PointSample& point, double tolerance,
                          double normalCosine = 0.8);
} // namespace reconstruction
