#pragma once

#include "modeling/FeatureParams.h"
#include "reconstruction/data/PrimitiveEvidence.h"

#include <functional>
#include <optional>
#include <string>

namespace modeling { class ModelingCore; }

namespace reconstruction {

enum class ProjectionPath { Automatic, SingleAxis, TwoAxisIntersection, Segmented };

struct ProjectionRegionSupport {
    std::size_t minimumPoints = 12;
    double minimumFraction = .001; // 0.10% of distinct points in one projection.
};

struct ProjectionExtrusionOptions {
    double distanceTolerance = 0; // mm; zero estimates scale from the part.
    std::size_t maximumAxes = 12;
    std::size_t maximumFitPoints = 24000;
    std::size_t maximumScorePoints = 1800;
    std::size_t maximumOperations = 3;
    bool includeAreaDirections = false;
    bool retainRejectedProgram = false; // Diagnostics only.
    ProjectionPath path = ProjectionPath::Automatic;
    bool collectSketchInputs = false; // Diagnostics only.
    ProjectionRegionSupport intersectionRegionSupport;
    double relativeDistanceTolerance = 0.0025;
    double scoreVoxelScale = 2;
    std::size_t maximumEvidencePlanes = 16;
    std::size_t maximumEvidenceCylinders = 6;
    std::size_t evidenceSupportDivisor = 300;
    double axisAlignmentCosine = 0.9986;
    double sideMomentNormalCosine = 0.2;
    double sideMomentEigenGap = 0.03;
    std::size_t areaSamplePoints = 2000;
    std::size_t areaSampleDirections = 64;
    std::size_t maximumAreaDirections = 3;
    double extentTrimFraction = 0.002;
    std::size_t minimumCapPoints = 12;
    double capBandMultiplier = 3;
    double capNormalCosine = 0.9;
    double minimumDepthMultiplier = 3;
    double capAlphaSpacingMultiplier = 1.5;
    double capAlphaToleranceMultiplier = 3;
    double capSimplificationTolerance = 0;
    std::size_t capSpacingNeighbors = 4;
    std::size_t capSpacingSamples = 256;
    std::size_t maximumSketchEdges = 80;
    std::size_t minimumCircleVertices = 8;
    double minimumCircleRadiusMultiplier = 5;
    double maximumCircleGapDegrees = 60;
    double sideSnapNormalCosine = 0.08;
    double sideSnapReachMultiplier = 2;
    double intersectionAlphaRadius = 0;
    double intersectionAlphaScale = 1;
    double intersectionSimplificationTolerance = 0;
    double intersectionMaximumAxisCosine = 0.5;
    double innerWallNormalCosine = 0.3;
    double innerWallEndTrimFraction = 0.1;
    std::size_t innerWallDepthBands = 5;
    std::size_t innerWallMinimumBands = 3;
    std::size_t innerWallMinimumPointsPerBand = 2;
    double innerWallReachMultiplier = 2;
    double capWallSpacingMultiplier = 0.5;
    double cappedCylinderCoverage = 0.8;
    double cutCylinderCoverage = 0.65;
    double cylinderEndBandMultiplier = 4;
    double cylinderRadialNormalCosine = 0.85;
    double cylinderOpposingNormalFraction = 0.1;
    double exteriorCircleOutsideFraction = 0.05;
    std::size_t minimumOpeningPoints = 6;
    double blindHoleNormalCosine = 0.95;
    double throughHoleOpeningNormalCosine = 0.9;
    double throughHoleToleranceSpanFraction = 0.16666666666666666;
    double throughHoleOpeningBandMultiplier = 3;
    double throughHoleOpeningBandFraction = 0.25;
    double throughHoleCutMarginToleranceFraction = 0.1;
    double throughHoleCutMarginDepthFraction = 0.05;
    double slotWallNormalCosine = 0.95;
    double slotWallMinimumDepthFraction = 0.6;
    std::size_t slotWallMinimumPoints = 6;
    double slotWallDistanceMultiplier = 2;
    std::size_t minimumSlotSupportedEdges = 2;
    std::size_t maximumInteriorSections = 2;
    double interiorSectionEndMultiplier = 5;
    double interiorSectionMergeMultiplier = 4;
    std::size_t searchBeamWidth = 4;
    double minimumScoreImprovement = 0.01;
    double minimumInlierFraction = 0.9;
    double maximumP95Multiplier = 3;
    double maximumUnsupportedFaceFraction = 0.04;
    double scoreMaximumDistanceLoss = 8;
    double scoreNormalWeight = 0.5;
    double scoreFaceSupportNormalCosine = 0.8;
    std::size_t scoreMinimumFacePoints = 3;
    double scoreOperationPenalty = 0.015;
    double scoreCurvePenalty = 0.001;
    double minimumRingAreaMultiplier = 1;
    double cylinderOpeningClearanceMultiplier = 2;
    double cylinderMergeToleranceMultiplier = 2;
    double cylinderMergeSpacingMultiplier = 2;
    double throughHoleMinimumRadiusMultiplier = 2;
    double throughHoleMaximumResidualMultiplier = 1;
    double holeOpeningOuterRadiusMultiplier = 2;
    double cutOvertravelMultiplier = 1;
    double projectionDepthPaddingMultiplier = 1;
    double minimumBooleanVolumeMultiplier = 1;
};

struct ProjectionSketchInput {
    enum class Section { WholePart, FirstCap, LastCap };
    modeling::Plane3d plane;
    Section section = Section::WholePart;
    std::vector<modeling::Point2D> points;
    double alphaRadius = 0;
    std::optional<double> simplificationTolerance;
    PointId firstPointId = kInvalidPointId;
    ProjectionRegionSupport regionSupport{0, 0};
    double automaticAlphaScale = 1;
};

struct ProjectionSketchRegion {
    std::vector<modeling::Point2D> outer;
    std::vector<std::vector<modeling::Point2D>> holes;
};

struct ProjectionSketchBoundary {
    std::vector<ProjectionSketchRegion> regions;
    double alphaRadius = 0;
    double simplificationTolerance = 0;
    std::size_t discardedRegionCount = 0;
};

struct ProjectionOperation {
    modeling::SketchFeatureParams sketch;
    modeling::ExtrudeOperation operation = modeling::ExtrudeOperation::NewBody;
    double depth = 0;
    bool reverse = false;
    std::vector<PointId> supportPointIds;
};

struct ProjectionFitQuality {
    std::size_t scoredPoints = 0;
    double medianDistance = 0;
    double p95Distance = 0;
    double surfaceInlierFraction = 0;
    double unsupportedFaceAreaFraction = 0;
    double meanNormalErrorDegrees = 0;
    std::size_t normalComparisons = 0;
    double score = 0;
};

struct ProjectionAxisReport {
    modeling::Vec3 axis{};
    std::string source;
    double convexProjectionArea = 0;
    double convexEnvelopeVolume = 0;
    std::string status;
    std::optional<ProjectionFitQuality> bestBaseQuality;
};

struct ProjectionExtrusionResult {
    double distanceTolerance = 0;
    std::vector<ProjectionAxisReport> axes;
    std::vector<ProjectionOperation> operations;
    ProjectionFitQuality quality;
    std::vector<PointId> residualPointIds;
    std::string reason;
    // Index of the best program's base axis, including a best rejected program.
    std::optional<std::size_t> bestAxisIndex;
    std::vector<ProjectionOperation> rejectedOperations;
    std::vector<ProjectionSketchInput> sketchInputs;
};

// Prepares the same 2D observations as the requested path, without CAD fitting.
bool inspectProjectionSketches(const PointStore& source, const ProjectionExtrusionOptions& options,
    ProjectionExtrusionResult& result, const std::function<bool()>& cancelled = {});
bool inspectProjectionSketches(const PointStore& source, ProjectionExtrusionResult& result,
    const std::function<bool()>& cancelled = {});

// Returns the raw fitted regions before the CAD sketch's topology and wall checks.
ProjectionSketchBoundary fitProjectionSketchBoundary(const ProjectionSketchInput& input);

// Fits one metrically scaled component. Evidence, if supplied, must use the
// same point IDs and frame. Failure retains diagnostic axes and reason.
bool fitProjectionExtrusion(const PointStore& points,
    const ProjectionExtrusionOptions& options, ProjectionExtrusionResult& result,
    const std::vector<PrimitiveEvidence>& evidence = {},
    const std::function<bool()>& cancelled = {});
bool fitProjectionExtrusion(const PointStore& points, ProjectionExtrusionResult& result,
    const std::vector<PrimitiveEvidence>& evidence = {}, const std::function<bool()>& cancelled = {});

// Replays a complete program in isolation and publishes it atomically.
bool buildProjectionProgram(const std::vector<ProjectionOperation>& operations,
    modeling::ModelingCore& model, std::string& error);

} // namespace reconstruction
