#pragma once
#include "reconstruction/recognition/GuidedSolidRecognizer.h"
#include "reconstruction/recognition/BoxRecognizer.h"
#include "common/Math3.h"

namespace reconstruction::detail::guided {
struct TemplateRegions {
    std::vector<PointSample> adjustedPoints;
    std::vector<PrimitiveEvidence> seeds;
    std::vector<CurvedSolidCandidate> closed;
    std::vector<TemplateFit> fits;
    int failedRegionId = 0;
};
bool fitTemplateRegions(const PointStore& points, const PlaneDetectionOptions& planes,
    const CylinderDetectionOptions& cylinders, const std::vector<TemplateHint>& hints,
    TemplateRegions& output, std::string& error, const std::function<bool()>& cancelled);

PrimitiveDetectionOptions detectionFor(TemplateKind kind, double tolerance);

std::size_t supportSize(const PrimitiveEvidence& e);

std::vector<PointSample> expandCurvedSupport(const PrimitiveEvidence& evidence,
                                             const std::vector<PointSample>& samples,
                                             double tolerance);

bool onBox(const modeling::BoxPrimitiveParams& box, const PointSample& p, double tolerance);

bool verifyHints(const PointStore& points, const std::vector<TemplateHint>& hints,
                 const CompoundCandidate& candidate, const std::vector<TemplateFit>& fits,
                 double tolerance, int& failed, std::string& error,
                 const std::function<bool()>& cancelled);
} // namespace reconstruction::detail::guided
