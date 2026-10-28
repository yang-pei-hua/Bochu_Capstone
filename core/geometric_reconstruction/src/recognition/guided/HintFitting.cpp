#include "detail/GuidedInternal.h"
#include <algorithm>
#include <cmath>

namespace reconstruction::detail::guided {
PrimitiveDetectionOptions detectionFor(TemplateKind kind, double tolerance)
{
    PrimitiveDetectionOptions o;
    o.detectPlanes = kind == TemplateKind::Plane;
    o.detectCylinders = kind == TemplateKind::Cylinder;
    o.detectSpheres = kind == TemplateKind::Sphere;
    o.detectCones = kind == TemplateKind::Cone;
    o.detectTori = kind == TemplateKind::Torus;
    o.plane.distanceThreshold = o.cylinder.distanceThreshold = o.sphere.distanceThreshold =
        o.cone.distanceThreshold = o.torus.distanceThreshold = tolerance;
    o.plane.minimumSupportPoints = o.cylinder.minimumSupportPoints = o.sphere.minimumSupportPoints =
        o.cone.minimumSupportPoints = o.torus.minimumSupportPoints = 20;
    o.plane.maximumPlanes = o.cylinder.maximumCylinders = o.sphere.maximumSpheres =
        o.cone.maximumCones = o.torus.maximumTori = 1;
    return o;
}

std::size_t supportSize(const PrimitiveEvidence& e)
{
    return std::visit([](const auto& surface) { return surface.supportPointIds.size(); }, e);
}

std::vector<PointSample> expandCurvedSupport(const PrimitiveEvidence& evidence,
                                             const std::vector<PointSample>& samples,
                                             double tolerance)
{
    using namespace math3;
    return std::visit(
        [&](const auto& surface) {
            using S = std::decay_t<decltype(surface)>;
            std::vector<PointSample> expanded;
            if constexpr (std::is_same_v<S, SphereEvidence>) {
                modeling::CurvedPrimitiveParams params =
                    modeling::SpherePrimitiveParams{surface.sphere.center, surface.sphere.radius};
                for (const auto& p : samples)
                    if (matchesCurvedBoundary(params, p, tolerance * 3))
                        expanded.push_back(p);
            } else if constexpr (std::is_same_v<S, TorusEvidence>) {
                modeling::CurvedPrimitiveParams params = modeling::TorusPrimitiveParams{
                    surface.torus.center, surface.torus.axisDirection, surface.torus.majorRadius,
                    surface.torus.minorRadius};
                for (const auto& p : samples)
                    if (matchesCurvedBoundary(params, p, tolerance * 3))
                        expanded.push_back(p);
            } else if constexpr (std::is_same_v<S, CylinderEvidence> ||
                                 std::is_same_v<S, ConeEvidence>) {
                modeling::Vec3 center, axis;
                double radius, slope;
                if constexpr (std::is_same_v<S, CylinderEvidence>) {
                    center = surface.cylinder.axisPoint;
                    axis = normalized(surface.cylinder.axisDirection);
                    radius = surface.cylinder.radius;
                    slope = 0;
                } else {
                    center = surface.cone.apex;
                    axis = normalized(surface.cone.axisDirection);
                    radius = 0;
                    slope = std::tan(surface.cone.openingAngleRadians);
                }
                double minimum = INFINITY, maximum = -INFINITY;
                std::vector<std::pair<PointSample, double>> disks;
                for (const auto& p : samples) {
                    if (!p.normal)
                        continue;
                    const auto delta = subtract(p.position, center);
                    const double axial = dot(delta, axis), expectedRadius = radius + axial * slope;
                    const auto radial = subtract(delta, scale(axis, axial));
                    const double rho = length(radial);
                    if (expectedRadius < 0)
                        continue;
                    const auto normal = normalized(*p.normal);
                    if (std::abs(dot(normal, axis)) > 0.98 && rho <= expectedRadius + tolerance * 3)
                        disks.push_back({p, axial});
                    if (rho <= 1e-12)
                        continue;
                    const auto outward =
                        normalized(subtract(scale(radial, 1 / rho), scale(axis, slope)));
                    if (std::abs(rho - expectedRadius) / std::sqrt(1 + slope * slope) >
                            tolerance * 3 ||
                        dot(normal, outward) < 0.8)
                        continue;
                    expanded.push_back(p);
                    minimum = std::min(minimum, axial);
                    maximum = std::max(maximum, axial);
                }
                if (!expanded.empty()) {
                    const double margin = (maximum - minimum) * 0.1 + tolerance * 3;
                    for (const auto& disk : disks)
                        if (disk.second >= minimum - margin && disk.second <= maximum + margin)
                            expanded.push_back(disk.first);
                }
            }
            return expanded;
        },
        evidence);
}

bool onBox(const modeling::BoxPrimitiveParams& box, const PointSample& p, double tolerance)
{
    using namespace math3;
    const auto x = normalized(box.pose.xDirection), z = normalized(box.pose.zDirection),
               y = cross(z, x);
    const auto d = subtract(p.position, box.pose.origin);
    const std::array<modeling::Vec3, 3> axes{x, y, z};
    const std::array<double, 3> sizes{box.sizeX, box.sizeY, box.sizeZ};
    bool boundary = false;
    for (int i = 0; i < 3; ++i) {
        const double c = dot(d, axes[i]);
        if (c < -tolerance || c > sizes[i] + tolerance)
            return false;
        boundary |= std::abs(c) <= tolerance || std::abs(c - sizes[i]) <= tolerance;
    }
    return boundary;
}
} // namespace reconstruction::detail::guided
