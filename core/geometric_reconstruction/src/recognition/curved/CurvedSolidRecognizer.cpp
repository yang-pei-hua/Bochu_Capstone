#include "detail/CurvedInternal.h"

namespace reconstruction {
using namespace detail::curved;

bool reconstructCurvedSolid(const PointStore &points, const CurvedSolidRecognitionOptions &options,
                            CurvedSolidCandidate &output, std::string &error) {
    output = {};
    if (points.empty() || points.unit() != LengthUnit::Millimeter ||
        !std::isfinite(options.distanceTolerance) || options.distanceTolerance <= 0 ||
        !(options.minimumSurfaceCoverage > 0 && options.minimumSurfaceCoverage <= 1) ||
        !(options.minimumBoundaryCoverage > 0 && options.minimumBoundaryCoverage <= 1) ||
        !(options.minimumNormalAlignment > 0 && options.minimumNormalAlignment <= 1)) {
        error = "Curved solid requires metric points and valid recognition tolerances";
        return false;
    }
    for (const auto &p : points.points())
        if (!p.normal || length(*p.normal) < 1e-12) {
            error = "Curved solid requires consistently outward surface normals";
            return false;
        }
    PlaneDetectionOptions planes;
    planes.distanceThreshold = options.distanceTolerance;
    planes.randomSeed = options.randomSeed;
    CylinderDetectionOptions cylinders;
    cylinders.distanceThreshold = options.distanceTolerance;
    cylinders.minimumSupportPoints = std::max<std::size_t>(30, points.size()/8);
    cylinders.probability = 0.001;
    auto settings = recognitionAnalysisOptions(points, planes, cylinders);
    settings.detection.detectPlanes = false;
    if (options.requiredKind) {
        settings.detection.detectCylinders = *options.requiredKind == PrimitiveKind::Cylinder;
        settings.detection.detectSpheres = *options.requiredKind == PrimitiveKind::Sphere;
        settings.detection.detectCones = *options.requiredKind == PrimitiveKind::Cone;
        settings.detection.detectTori = *options.requiredKind == PrimitiveKind::Torus;
    }
    SurfaceAnalysisResult analysis;
    if (!analyzeSurfaces(points, settings, analysis, error, options.seeds)) return false;
    return recognizeCurvedSolidFromAnalysis(points, analysis, options, output, error);
}

bool recognizeCurvedSolidFromAnalysis(const PointStore& points, const SurfaceAnalysisResult& analysis,
                                     const CurvedSolidRecognitionOptions& options,
                                     CurvedSolidCandidate& output, std::string& error) {
    output = {};
    if (!analysisMatches(points, analysis)) { error = "Stale surface analysis"; return false; }
    if (points.empty() || points.unit() != LengthUnit::Millimeter ||
        !std::isfinite(options.distanceTolerance) || options.distanceTolerance <= 0 ||
        !(options.minimumSurfaceCoverage > 0 && options.minimumSurfaceCoverage <= 1) ||
        !(options.minimumBoundaryCoverage > 0 && options.minimumBoundaryCoverage <= 1) ||
        !(options.minimumNormalAlignment > 0 && options.minimumNormalAlignment <= 1)) {
        error = "Curved solid requires metric points and valid recognition tolerances"; return false;
    }
    for (const auto& p : points.points()) if (!p.normal || !p.normalOrientationKnown) {
        error = "Curved solid requires outward normals"; return false;
    }
    bool found = false;
    double score = -INFINITY;
    for (const auto &e : analysis.surfaces) {
        if (options.requiredKind && primitiveKind(e) != *options.requiredKind) continue;
        CurvedSolidCandidate candidate;
        bool constructed = false;
        std::visit(
            [&](const auto &evidence) {
                using E = std::decay_t<decltype(evidence)>;
                if constexpr (std::is_same_v<E, SphereEvidence>) {
                    auto s = evidence.sphere;
                    Eigen::VectorXd x(4);
                    x << s.center.x, s.center.y, s.center.z, s.radius;
                    if (!refine(x, support(points, evidence), options.distanceTolerance,
                                sphereResidual))
                        return;
                    candidate.primitive = modeling::SpherePrimitiveParams{vec(x, 0), x[3]};
                    constructed = true;
                } else if constexpr (std::is_same_v<E, CylinderEvidence>) {
                    auto c = evidence.cylinder;
                    double t0, t1;
                    if (!ends(caps(points, c.axisPoint, c.axisDirection, options.distanceTolerance,
                                   [&](double) { return c.radius; }),
                              t0, t1, false))
                        return;
                    candidate.primitive = modeling::CylinderPrimitiveParams{
                        add(c.axisPoint, scale(c.axisDirection, t0)), c.axisDirection, c.radius,
                        t1 - t0};
                    constructed = true;
                } else if constexpr (std::is_same_v<E, ConeEvidence>) {
                    auto c = evidence.cone;
                    Eigen::VectorXd x(7);
                    x << c.apex.x, c.apex.y, c.apex.z, c.axisDirection.x, c.axisDirection.y,
                        c.axisDirection.z, c.openingAngleRadians;
                    if (!refine(x, support(points, evidence), options.distanceTolerance,
                                coneResidual))
                        return;
                    const auto axis = normalized(vec(x, 3)), apex = vec(x, 0);
                    const double slope = std::tan(x[6]);
                    double t0, t1;
                    if (!ends(caps(points, apex, axis, options.distanceTolerance,
                                   [&](double t) { return std::max(0.0, t * slope); }),
                              t0, t1, true) ||
                        t0 < -2 * options.distanceTolerance)
                        return;
                    t0 = std::max(0.0, t0);
                    candidate.primitive = modeling::ConePrimitiveParams{
                        add(apex, scale(axis, t0)), axis, t0 * slope, t1 * slope, t1 - t0};
                    constructed = true;
                } else if constexpr (std::is_same_v<E, TorusEvidence>) {
                    auto t = evidence.torus;
                    Eigen::VectorXd x(8);
                    x << t.center.x, t.center.y, t.center.z, t.axisDirection.x, t.axisDirection.y,
                        t.axisDirection.z, t.majorRadius, t.minorRadius;
                    if (!refine(x, support(points, evidence), options.distanceTolerance,
                                torusResidual))
                        return;
                    candidate.primitive = modeling::TorusPrimitiveParams{
                        vec(x, 0), normalized(vec(x, 3)), x[6], x[7]};
                    constructed = true;
                }
            },
            e);
        if (!constructed || !assess(points, options, candidate))
            continue;
        const double value = candidate.surfaceCoverage + candidate.boundaryCoverage -
                             candidate.meanResidual / options.distanceTolerance;
        if (value > score) {
            score = value;
            output = candidate;
            found = true;
        }
    }
    if (!found) {
        error = "No complete curved solid: missing caps, incomplete angular coverage or "
                "inconsistent boundary/normals";
        return false;
    }
    error.clear();
    return true;
}

bool matchesCurvedBoundary(const modeling::CurvedPrimitiveParams& primitive,
                          const PointSample& point, double tolerance) {
    const auto b = boundary(primitive,point.position);
    return b.distance <= tolerance && point.normal &&
        dot(b.normal,normalized(*point.normal)) >= 0.8;
}
} // namespace reconstruction
