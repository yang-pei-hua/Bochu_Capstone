#include "detail/AnalysisInternal.h"
#include "reconstruction/detection/MultiScaleDetector.h"

namespace reconstruction {
using namespace detail::analysis;

bool analyzeSurfaces(const PointStore& points, const SurfaceAnalysisOptions& options,
                     SurfaceAnalysisResult& output, std::string& error,
                     const std::vector<PrimitiveEvidence>& seeds, const std::function<bool()>& cancelled) {
    output = {}; error.clear();
    if (!valid(options)) { error = "Invalid surface analysis options"; return false; }
    std::vector<PrimitiveEvidence> evidence;
    std::size_t passes = 0;
    const auto& d = options.detection;
    const std::array<bool,5> enabled{d.detectPlanes,d.detectCylinders,d.detectSpheres,d.detectCones,d.detectTori};
    if (std::none_of(enabled.begin(),enabled.end(),[](bool x){return x;})) { error = "No surface family enabled"; return false; }
    std::string failure;
    if(options.recoverResidualSurfaces) {
        PrimitiveDetectionResult result;MultiScaleDetectionReport report;
        if(!detectMultiScalePrimitiveEvidence(points,d,result,report,failure,cancelled)) {
            if(failure=="Cancelled"){error=failure;return false;}
        }
        evidence=std::move(result.evidence);passes=report.detectionPasses;
    } else for (int family = 0; family < (options.independentFamilies ? 5 : 1); ++family) {
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        if (options.independentFamilies && !enabled[family]) continue;
        auto detection = d;
        if (options.independentFamilies) {
            detection.detectPlanes = family == 0; detection.detectCylinders = family == 1;
            detection.detectSpheres = family == 2; detection.detectCones = family == 3; detection.detectTori = family == 4;
        }
        PrimitiveDetectionResult result; ++passes;
        if (detectPrimitiveEvidence(points,detection,result,failure))
            evidence.insert(evidence.end(),result.evidence.begin(),result.evidence.end());
    }
    // Manual evidence replaces only same-family observations with substantial
    // source overlap, never unrelated nearby surfaces sharing an equation.
    for (const auto& seed : seeds) {
        std::unordered_set<PointId> fixed(ids(seed).begin(),ids(seed).end());
        evidence.erase(std::remove_if(evidence.begin(),evidence.end(),[&](const auto& e) {
            if (primitiveKind(e) != primitiveKind(seed)) return false;
            std::size_t shared = 0; for (auto id : ids(e)) shared += fixed.count(id);
            return shared > 0 && ids(e).size() <= fixed.size()*1.25 &&
                shared >= 0.7*std::min(ids(e).size(),fixed.size());
        }),evidence.end());
        evidence.push_back(seed);
    }
    if (cancelled && cancelled()) { error = "Cancelled"; return false; }
    if (evidence.empty()) { error = failure.empty() ? "No supported surface evidence" : failure; return false; }
    if (!indexSurfaceEvidence(points,evidence,options,output,error)) return false;
    output.detectionPasses = passes;
    return true;
}

SurfaceAnalysisOptions recognitionAnalysisOptions(const PointStore& points, const PlaneDetectionOptions& planes,
                                                 const CylinderDetectionOptions& cylinders) {
    SurfaceAnalysisOptions o; auto& d = o.detection;
    d.randomSeed = planes.randomSeed; d.plane = planes; d.cylinder = cylinders;
    d.plane.maximumPlanes = 128; d.cylinder.maximumCylinders = 32;
    // A small wall/cap is not less real because a much larger host is present.
    // Candidate validation, rather than scene-size quotas, rejects accidental fits.
    d.plane.minimumSupportPoints = std::min<std::size_t>(planes.minimumSupportPoints,30);
    d.cylinder.minimumSupportPoints = std::min<std::size_t>(cylinders.minimumSupportPoints,30);
    const auto minimum = std::max<std::size_t>(30,std::min<std::size_t>(128,planes.minimumSupportPoints));
    d.sphere.minimumSupportPoints = d.cone.minimumSupportPoints = d.torus.minimumSupportPoints = minimum;
    d.sphere.distanceThreshold = d.cone.distanceThreshold = d.torus.distanceThreshold = planes.distanceThreshold;
    d.sphere.probability = d.cone.probability = d.torus.probability = 0.001;
    d.sphere.maximumSpheres = d.cone.maximumCones = d.torus.maximumTori = 32;
    o.relations.distanceTolerance = planes.distanceThreshold*4;
    return o;
}
} // namespace reconstruction
