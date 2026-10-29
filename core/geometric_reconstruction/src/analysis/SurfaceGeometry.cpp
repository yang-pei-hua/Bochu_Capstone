#include "detail/AnalysisInternal.h"

namespace reconstruction::detail::analysis {
const std::vector<PointId>& ids(const PrimitiveEvidence& e) {
    return std::visit([](const auto& x) -> const std::vector<PointId>& { return x.supportPointIds; }, e);
}

double confidence(const PrimitiveEvidence& e) {
    return std::visit([](const auto& x) { return x.confidence; }, e);
}

std::unordered_map<PointId, const PointSample*> pointMap(const PointStore& points) {
    std::unordered_map<PointId, const PointSample*> result;
    result.reserve(points.size());
    for (const auto& p : points.points()) result.emplace(p.id, &p);
    return result;
}

Vec3 axis(const PrimitiveEvidence& e) {
    return std::visit([](const auto& s) -> Vec3 {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, PlaneEvidence>) return normalized(s.plane.normal);
        else if constexpr (std::is_same_v<T, CylinderEvidence>) return normalized(s.cylinder.axisDirection);
        else if constexpr (std::is_same_v<T, ConeEvidence>) return normalized(s.cone.axisDirection);
        else if constexpr (std::is_same_v<T, TorusEvidence>) return normalized(s.torus.axisDirection);
        else return {};
    }, e);
}

Vec3 center(const PrimitiveEvidence& e) {
    return std::visit([](const auto& s) -> Vec3 {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, CylinderEvidence>) return s.cylinder.axisPoint;
        else if constexpr (std::is_same_v<T, ConeEvidence>) return s.cone.apex;
        else if constexpr (std::is_same_v<T, SphereEvidence>) return s.sphere.center;
        else if constexpr (std::is_same_v<T, TorusEvidence>) return s.torus.center;
        else return {};
    }, e);
}

double radius(const PrimitiveEvidence& e) {
    if (const auto* c = std::get_if<CylinderEvidence>(&e)) return c->cylinder.radius;
    if (const auto* s = std::get_if<SphereEvidence>(&e)) return s->sphere.radius;
    return -1;
}

bool axial(PrimitiveKind k) { return k == PrimitiveKind::Cylinder || k == PrimitiveKind::Cone || k == PrimitiveKind::Torus; }

bool valid(const SurfaceAnalysisOptions& o) {
    return std::isfinite(o.connectivityDistance) && o.connectivityDistance >= 0 &&
        std::isfinite(o.relations.distanceTolerance) && o.relations.distanceTolerance > 0 &&
        std::isfinite(o.relations.angularToleranceRadians) && o.relations.angularToleranceRadians > 0 &&
        o.relations.angularToleranceRadians <= 0.5235987755982988;
}
} // namespace reconstruction::detail::analysis

namespace reconstruction {
using namespace detail::analysis;
double surfaceResidual(const PrimitiveEvidence& e, Vec3 p) {
    return std::visit([&](const auto& s) -> double {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, PlaneEvidence>) return std::abs(dot(s.plane.normal, p)+s.plane.offset);
        else if constexpr (std::is_same_v<T, SphereEvidence>) return std::abs(length(subtract(p, s.sphere.center))-s.sphere.radius);
        else {
            const Vec3 a = axis(e), d = subtract(p, center(e));
            const double t = dot(d, a), r = length(subtract(d, scale(a, t)));
            if constexpr (std::is_same_v<T, CylinderEvidence>) return std::abs(r-s.cylinder.radius);
            else if constexpr (std::is_same_v<T, ConeEvidence>) {
                if (t < 0) return INFINITY;
                return std::abs(r*std::cos(s.cone.openingAngleRadians)-t*std::sin(s.cone.openingAngleRadians));
            } else return std::abs(std::hypot(r-s.torus.majorRadius, t)-s.torus.minorRadius);
        }
    }, e);
}
double surfaceNormalAlignment(const PrimitiveEvidence& e,Vec3 p,Vec3 normal) {
    if(!std::isfinite(length(normal)) || length(normal)<1e-12)return 0;
    normal=normalized(normal);
    return std::visit([&](const auto& s)->double {
        using T=std::decay_t<decltype(s)>;Vec3 n;
        if constexpr(std::is_same_v<T,PlaneEvidence>)n=s.plane.normal;
        else if constexpr(std::is_same_v<T,SphereEvidence>)n=subtract(p,s.sphere.center);
        else {
            const auto a=axis(e),d=subtract(p,center(e));const double t=dot(d,a);
            const auto radial=subtract(d,scale(a,t));const double r=length(radial);
            if(r<1e-12)return 0;
            if constexpr(std::is_same_v<T,CylinderEvidence>)n=radial;
            else if constexpr(std::is_same_v<T,ConeEvidence>)n=subtract(scale(radial,std::cos(s.cone.openingAngleRadians)/r),scale(a,std::sin(s.cone.openingAngleRadians)));
            else n=subtract(d,scale(radial,s.torus.majorRadius/r));
        }
        return length(n)>1e-12?std::abs(dot(normal,normalized(n))):0;
    },e);
}
} // namespace reconstruction
