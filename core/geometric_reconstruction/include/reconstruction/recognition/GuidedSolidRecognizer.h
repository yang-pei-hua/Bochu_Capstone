#pragma once
#include "reconstruction/recognition/CompoundRecognizer.h"
#include <functional>
#include <optional>

namespace reconstruction {
enum class TemplateKind { Plane, Cylinder, Sphere, Cone, Torus, Box };
struct TemplateHint
{
    int regionId = 0;
    TemplateKind kind = TemplateKind::Plane;
    std::vector<PointId> points;
};
struct TemplateFit
{
    int regionId = 0;
    std::optional<PrimitiveEvidence> surface;
    std::optional<BoxCandidate> box;
    std::optional<CurvedSolidCandidate> solid;
};
struct GuidedSolidResult
{
    CompoundCandidate candidate;
    SurfaceAnalysisResult analysis;
    bool analysisReused = false;
    std::vector<TemplateFit> fits;
    int failedRegionId = 0;
};
// Fit explicit groups first, use their equations/normals as whole-body seeds,
// recover remaining material, then check every hint against finite CAD faces.
// Partial surface hints never invent a cap or a thickness.
struct RecognitionSettings;
// Quality gates and sampling budgets used by explicit template fitting.
struct GuidedRecognitionOptions {
    std::size_t minimumTemplatePoints = 20;
    std::size_t templateBoxMinimumSupportPoints = 20;
    double boundaryToleranceMultiplier = 3;
    double boundaryNormalCosine = 0.8;
    double minimumSupportFraction = 0.9;
    double relationToleranceMultiplier = 4;
    std::size_t minimumExpandedPoints = 20;
    double minimumSurfaceSupportFraction = 0.7;
    std::size_t templateDetectionMinimumSupportPoints = 20;
    double capNormalCosine = 0.98;
    double sideNormalCosine = 0.8;
    double capMarginFraction = 0.1;
    double minimumMatchFraction = 0.9;
    std::size_t minimumSeedPoints = 20;
};
bool reconstructGuidedSolid(const PointStore& points, const PlaneDetectionOptions& planes,
                            const CylinderDetectionOptions& cylinders,
                            const std::vector<TemplateHint>& hints, GuidedSolidResult& output,
                            std::string& error, const std::function<bool()>& cancelled = {},
                            const SurfaceAnalysisResult* sharedAnalysis = nullptr,
                            const std::vector<SurfaceRelationHint>& relationHints = {},
                            const PointStore* source = nullptr,
                            const RecognitionSettings* settings = nullptr);
} // namespace reconstruction
