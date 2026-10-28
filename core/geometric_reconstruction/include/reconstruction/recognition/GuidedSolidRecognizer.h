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
bool reconstructGuidedSolid(const PointStore& points, const PlaneDetectionOptions& planes,
                            const CylinderDetectionOptions& cylinders,
                            const std::vector<TemplateHint>& hints, GuidedSolidResult& output,
                            std::string& error, const std::function<bool()>& cancelled = {},
                            const SurfaceAnalysisResult* sharedAnalysis = nullptr,
                            const std::vector<SurfaceRelationHint>& relationHints = {},
                            const PointStore* source = nullptr);
} // namespace reconstruction
