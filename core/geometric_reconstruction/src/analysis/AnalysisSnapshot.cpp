#include "detail/AnalysisInternal.h"

namespace reconstruction {
using namespace detail::analysis;

std::uint64_t pointStoreFingerprint(const PointStore& points) noexcept {
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](const auto& value) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(value); ++i) h = (h ^ bytes[i])*1099511628211ULL;
    };
    const auto unit = static_cast<int>(points.unit()); mix(unit);
    for (const auto& p : points.points()) {
        mix(p.id); mix(p.position.x); mix(p.position.y); mix(p.position.z);
        const bool has = p.normal.has_value(); mix(has); mix(p.normalOrientationKnown);
        if (p.normal) { mix(p.normal->x); mix(p.normal->y); mix(p.normal->z); }
        mix(p.confidence);
    }
    return h;
}

bool analysisMatches(const PointStore& p, const SurfaceAnalysisResult& a) noexcept {
    return a.unit == p.unit() && a.pointFingerprint == pointStoreFingerprint(p);
}

bool analysisSettingsMatch(const SurfaceAnalysisOptions& a,const SurfaceAnalysisOptions& b) {
    auto signature=[](const SurfaceAnalysisOptions& o) {
        const auto& d=o.detection;
        std::vector<double> v{double(d.randomSeed),double(d.detectPlanes),double(d.detectCylinders),
            double(d.detectSpheres),double(d.detectCones),double(d.detectTori),o.connectivityDistance,
            double(o.independentFamilies),o.relations.distanceTolerance,o.relations.angularToleranceRadians};
        auto add=[&](const auto& p,double maximum) {
            v.insert(v.end(),{p.distanceThreshold,double(p.minimumSupportPoints),maximum,p.probability,
                             p.minimumNormalAlignment,p.clusterEpsilon});
        };
        add(d.plane,double(d.plane.maximumPlanes));add(d.cylinder,double(d.cylinder.maximumCylinders));
        add(d.sphere,double(d.sphere.maximumSpheres));add(d.cone,double(d.cone.maximumCones));add(d.torus,double(d.torus.maximumTori));
        v.insert(v.end(),{d.plane.duplicatePlaneAngularToleranceRadians,d.plane.duplicatePlaneDistanceMultiplier,
            d.cylinder.minimumRadius,d.cylinder.maximumRadius,d.sphere.minimumRadius,d.sphere.maximumRadius,
            d.cone.minimumOpeningAngleRadians,d.cone.maximumOpeningAngleRadians,
            d.torus.minimumMajorRadius,d.torus.maximumMajorRadius,d.torus.minimumMinorRadius,d.torus.maximumMinorRadius});
        return v;
    };
    return signature(a)==signature(b);
}
} // namespace reconstruction
