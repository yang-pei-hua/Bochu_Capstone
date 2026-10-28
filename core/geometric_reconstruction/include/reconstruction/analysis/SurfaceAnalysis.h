#pragma once

#include "reconstruction/detection/PrimitiveDetector.h"
#include <functional>
#include <optional>

namespace reconstruction {

// IDs are scoped to one analysis fingerprint. Equations and finite, connected
// observation patches are deliberately separate: a plane may have many patches.
using PatchId = std::uint64_t;
enum class SurfaceRelationKind {
    Adjacent, Parallel, Perpendicular, Coplanar, Coaxial, SameRadius,
    Intersect, Tangent, AxisNormalToPlane, AxisParallelToPlane
};
const char* surfaceRelationName(SurfaceRelationKind kind) noexcept;
struct SurfaceRelation {
    PatchId first = 0, second = 0;
    SurfaceRelationKind kind = SurfaceRelationKind::Adjacent;
    double error = 0, confidence = 0;
    bool automatic = true; // A hypothesis, not an equality imposed on the model.
};
struct SurfacePatch {
    PatchId id = 0;
    SurfaceId surfaceId = 0;
    std::vector<PointId> supportPointIds;
    modeling::Vec3 lower{}, upper{}, centroid{}, materialNormal{};
    bool orientationKnown = false;
};
struct RelationSurface {
    PatchId id = 0;
    std::optional<PrimitiveEvidence> evidence; // Empty/stale fits still admit adjacency.
    std::vector<PointId> supportPointIds;
};
struct SurfaceRelationOptions {
    double distanceTolerance = 0.1;
    double angularToleranceRadians = 0.08726646259971647;
};
struct SurfaceRelationHint {
    std::vector<PointId> firstPoints, secondPoints;
    SurfaceRelationKind kind = SurfaceRelationKind::Adjacent;
};
struct SurfaceAnalysisOptions {
    PrimitiveDetectionOptions detection;
    SurfaceRelationOptions relations;
    double connectivityDistance = 0; // Zero: estimate from each surface's density.
    bool independentFamilies = true;
};
enum class LocalFailureReason { InsufficientSupport, MissingEndFace, CandidateConflict, InvalidRelation };
const char* localFailureReasonName(LocalFailureReason reason) noexcept;
struct LocalAnalysisFailure {
    LocalFailureReason reason = LocalFailureReason::InsufficientSupport;
    std::string message;
    std::vector<PointId> pointIds, recoveredPointIds;
    std::vector<PrimitiveEvidence> candidates, repairedCandidates;
    std::vector<SurfaceRelation> rejectedRelations, repairedRelations;
    std::vector<SurfacePatch> repairedPatches;
    std::uint64_t localFingerprint = 0;
    std::uint64_t sourceFingerprint = 0;
    std::size_t detectionPasses = 0;
    bool repaired = false;
};
struct RelationOptimizationReport {
    std::vector<SurfaceRelation> accepted, rejected;
    double beforeError = 0, afterError = 0;
};
struct SurfaceAnalysisResult {
    std::uint64_t pointFingerprint = 0;
    LengthUnit unit = LengthUnit::Arbitrary;
    SurfaceAnalysisOptions options;
    std::vector<PrimitiveEvidence> surfaces;
    std::vector<SurfacePatch> patches;
    std::vector<SurfaceRelation> relations;
    std::vector<PointId> unassignedPointIds;
    std::size_t detectionPasses = 0;
    std::vector<LocalAnalysisFailure> localFailures;
};

std::uint64_t pointStoreFingerprint(const PointStore& points) noexcept;
bool analysisMatches(const PointStore& points, const SurfaceAnalysisResult& analysis) noexcept;
bool analysisSettingsMatch(const SurfaceAnalysisOptions& a, const SurfaceAnalysisOptions& b);
bool hasSurfaceRelation(const SurfaceAnalysisResult& analysis, SurfaceId first, SurfaceId second,
                        SurfaceRelationKind kind) noexcept;
std::vector<std::vector<PointId>> connectedPointComponents(const PointStore& points,
    const std::vector<PointId>& ids, double distance, double maximumAdaptiveDistance = 0);
double surfaceResidual(const PrimitiveEvidence& evidence, modeling::Vec3 point);
// Geometry-only relation inference, shared by the client and batch pipeline.
bool inferSurfaceRelations(const PointStore& points, const std::vector<RelationSurface>& nodes,
                           const SurfaceRelationOptions& options,
                           std::vector<SurfaceRelation>& output, std::string& error);
// Manual relationships are checked against current observed patches before
// entering recognition. They do not move geometry or silently override data.
bool applySurfaceRelationHints(const PointStore& points, const std::vector<SurfaceRelationHint>& hints,
                              SurfaceAnalysisResult& analysis, std::string& error);
// Build from detected/manual evidence, without invoking a detector. Overlapping
// candidate supports are retained; this function does not force unique labels.
bool indexSurfaceEvidence(const PointStore& points, const std::vector<PrimitiveEvidence>& evidence,
                          const SurfaceAnalysisOptions& options, SurfaceAnalysisResult& output,
                          std::string& error);
// Transactional, observation-bounded optimization of a feasible relation subset.
bool optimizeSurfaceRelations(const PointStore& points, SurfaceAnalysisResult& analysis,
    const std::vector<SurfaceRelation>& proposals, RelationOptimizationReport& report, std::string& error);
// Source must use the same coordinate frame and units as the working cloud.
// Only actual source samples inside the failed region (plus tolerance halo) are retrieved.
bool recoverLocalSurfaces(const PointStore& working, const PointStore& source,
    const SurfaceAnalysisResult& analysis, LocalAnalysisFailure& failure,
    PointStore& local, SurfaceAnalysisResult& repaired, std::string& error);
bool analyzeSurfaces(const PointStore& points, const SurfaceAnalysisOptions& options,
                     SurfaceAnalysisResult& output, std::string& error,
                     const std::vector<PrimitiveEvidence>& seeds = {},
                     const std::function<bool()>& cancelled = {});
// Shared defaults for production recognition. Multi-family proposals are gathered
// independently, so one rejected family cannot consume another family's points.
SurfaceAnalysisOptions recognitionAnalysisOptions(const PointStore& points,
                                                 const PlaneDetectionOptions& planes,
                                                 const CylinderDetectionOptions& cylinders);
} // namespace reconstruction
