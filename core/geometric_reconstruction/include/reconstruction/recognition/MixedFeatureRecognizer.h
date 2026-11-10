#pragma once
#include "reconstruction/recognition/CompoundRecognizer.h"
namespace reconstruction {
struct RecognitionSettings;

// Quality gates, sampling budgets and scale multipliers used by the mixed
// recognizer. Defaults reproduce the values that were previously hard-coded.
struct MixedFeatureRecognitionOptions {
    double hostOpposingPlaneCosine = 0.995;
    double hostDuplicateSupportFraction = 0.2;
    double hostPlaneDistanceMultiplier = 4;
    double hostOutsidePlaneMultiplier = 3;
    double hostSideBandMultiplier = 2;
    std::size_t minimumHostPoints = 30;
    double maximumOpposingPointFraction = 0.01;
    double hostSideNormalCosine = 0.1;
    std::size_t minimumSidePoints = 20;
    double minimumSideFraction = 0.8;
    double hostSideAxisCosine = 0.995;
    double maximumThicknessRatio = 5;
    double hostConfidence = 0.95;
    std::size_t minimumEvidenceSupportPoints = 12;

    double holeInwardNormalFraction = 0.8;
    double aboveSideToleranceFraction = 0.1;
    double aboveSideNormalCosine = 0.9;
    double knownSurfaceToleranceMultiplier = 2;
    double knownSurfaceNormalCosine = 0.9;
    double connectivityToleranceMultiplier = 5;
    double connectivitySpacingMultiplier = 3.5;
    double connectivityAdaptiveMultiplier = 4;
    double coverageShiftToleranceMultiplier = 3;
    std::size_t minimumLocalPoints = 20;
    double capNormalCosine = 0.98;
    std::size_t minimumCapPoints = 20;
    double minimumSurfaceCoverage = 0.98;

    double bossAxisCosine = 0.995;
    double bossRadiusToleranceMultiplier = 2;
    double bossBandToleranceMultiplier = 2;
    double bossCapNormalCosine = 0.98;
    std::size_t bossMinimumCapPoints = 20;
    double bossMinimumHeightMultiplier = 4;
    double bossSideNormalCosine = 0.8;
    double bossReferenceAxisCosine = 0.9;
    std::size_t bossDepthBins = 6;
    std::size_t bossAngularBins = 24;
    std::size_t bossCapSectors = 8;
    std::size_t bossRadialBins = 2;
    double minimumBossCoverage = 0.9;
    double minimumBossBoundary = 0.8;

    double coplanarNormalCosine = 0.05;
    double coplanarAxisCosine = 0.995;
    double cutWallToleranceMultiplier = 2;
    std::size_t minimumWalls = 4;
    std::size_t maximumWalls = 12;
    double cutLevelAxisCosine = 0.995;
    double cutMinimumSizeMultiplier = 4;
    double cutSideNormalCosine = 0.8;
    double cutSpanToleranceMultiplier = 4;
};

// Bounded mixed-feature recognizer: an observed thin rectangular host, attached
// planar structures in independent local frames, vertical cylindrical/conical
// bosses, and round/rectangular through cuts. No template labels are consumed.
bool recognizeMixedFeatures(const PointStore& points, const SurfaceAnalysisResult& analysis,
                            CompoundCandidate& output, std::string& error,
                            std::vector<LocalAnalysisFailure>* failures = nullptr,
                            const PointStore* source = nullptr,
                            const RecognitionSettings* settings = nullptr);
} // namespace reconstruction
