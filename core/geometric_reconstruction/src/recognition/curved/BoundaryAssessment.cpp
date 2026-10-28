#include "detail/CurvedInternal.h"

namespace reconstruction::detail::curved {
Boundary boundary(const Params &params, Vec3 p) {
    return std::visit(
        [&](const auto &shape) -> Boundary {
            using P = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>) {
                const auto d = subtract(p, shape.center);
                const double n = length(d);
                return {std::abs(n - shape.radius), n > 1e-12 ? scale(d, 1 / n) : Vec3{}};
            } else if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>) {
                const auto axis = normalized(shape.axisDirection), d = subtract(p, shape.center);
                const auto radial = subtract(d, scale(axis, dot(d, axis)));
                const double rho = length(radial);
                if (rho < 1e-12)
                    return {};
                const auto tube = subtract(d, scale(radial, shape.majorRadius / rho));
                const double n = length(tube);
                return {std::abs(n - shape.minorRadius), n > 1e-12 ? scale(tube, 1 / n) : Vec3{}};
            } else {
                const auto axis = normalized(shape.axisDirection),
                           d = subtract(p, shape.baseCenter);
                const double t = dot(d, axis), rho = radialLength(d, axis);
                double r0, r1;
                if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>)
                    r0 = r1 = shape.radius;
                else {
                    r0 = shape.baseRadius;
                    r1 = shape.topRadius;
                }
                const double slope = (r1 - r0) / shape.height;
                const double sideT =
                    std::clamp((t + slope * (rho - r0)) / (1 + slope * slope), 0.0, shape.height);
                const auto radial = subtract(d, scale(axis, t));
                Boundary best{std::hypot(t - sideT, rho - (r0 + slope * sideT)),
                              rho > 1e-12
                                  ? normalized(subtract(scale(radial, 1 / rho), scale(axis, slope)))
                                  : Vec3{}};
                for (auto cap :
                     std::array<std::pair<double, double>, 2>{{{0, r0}, {shape.height, r1}}}) {
                    if (cap.second <= 1e-12)
                        continue;
                    const double distance =
                        std::hypot(t - cap.first, std::max(0.0, rho - cap.second));
                    if (distance < best.distance)
                        best = {distance, scale(axis, cap.first == 0 ? -1 : 1)};
                }
                return best;
            }
        },
        params);
}

bool assess(const PointStore &points, const CurvedSolidRecognitionOptions &options,
            CurvedSolidCandidate &result) {
    std::string text;
    if (!modeling::validateCurvedPrimitiveParams(result.primitive, text))
        return false;
    std::vector<const PointSample *> fitted;
    double residual = 0;
    for (const auto &p : points.points()) {
        const auto b = boundary(result.primitive, p.position);
        if (b.distance <= 2 * options.distanceTolerance &&
            dot(b.normal, normalized(*p.normal)) >= options.minimumNormalAlignment) {
            fitted.push_back(&p);
            residual += b.distance;
        }
    }
    result.surfaceCoverage = double(fitted.size()) / points.size();
    if (result.surfaceCoverage < options.minimumSurfaceCoverage)
        return false;
    result.meanResidual = residual / fitted.size();
    result.boundaryCoverage = std::visit(
        [&](const auto &shape) {
            using P = std::decay_t<decltype(shape)>;
            std::set<int> bins;
            std::array<std::set<int>, 2> capBins;
            Vec3 axis{0, 0, 1}, center;
            if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>)
                center = shape.center;
            else {
                axis = normalized(shape.axisDirection);
                if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                    center = shape.center;
                else
                    center = shape.baseCenter;
            }
            const auto u = transverse(axis), v = cross(axis, u);
            for (auto p : fitted) {
                const auto d = subtract(p->position, center);
                const double z = dot(d, axis), x = dot(d, u), y = dot(d, v), rho = std::hypot(x, y);
                if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>) {
                    bins.insert(angleBin(y, x, 12) * 6 + unitBin((z / length(d) + 1) / 2, 6));
                } else if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>) {
                    bins.insert(angleBin(y, x, 24) * 12 + angleBin(z, rho - shape.majorRadius, 12));
                } else {
                    const double alignment = dot(normalized(*p->normal), axis);
                    if (std::abs(alignment) > 0.98) {
                        double radius;
                        if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>)
                            radius = shape.radius;
                        else
                            radius = alignment < 0 ? shape.baseRadius : shape.topRadius;
                        if (radius > options.distanceTolerance)
                            capBins[alignment < 0 ? 0 : 1].insert(angleBin(y, x, 8) * 2 +
                                                                  unitBin(rho / radius, 2));
                    } else
                        bins.insert(angleBin(y, x, 24) * 6 + unitBin(z / shape.height, 6));
                }
            }
            if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>)
                return double(bins.size()) / 72;
            else if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                return double(bins.size()) / 288;
            else {
                double coverage = double(bins.size()) / 144;
                double r0, r1;
                if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>)
                    r0 = r1 = shape.radius;
                else {
                    r0 = shape.baseRadius;
                    r1 = shape.topRadius;
                }
                if (r0 > options.distanceTolerance)
                    coverage = std::min(coverage, double(capBins[0].size()) / 16);
                if (r1 > options.distanceTolerance)
                    coverage = std::min(coverage, double(capBins[1].size()) / 16);
                // A pointed cone must have observations near the apex itself.
                if (std::min(r0, r1) == 0) {
                    auto apex = add(center, scale(axis, r0 == 0 ? 0 : shape.height));
                    bool observed = false;
                    // At an apex the geometric surface normal is not unique.
                    for (const auto &p : points.points())
                        observed |=
                            length(subtract(p.position, apex)) <= 3 * options.distanceTolerance;
                    if (!observed)
                        return 0.0;
                }
                return coverage;
            }
        },
        result.primitive);
    return result.boundaryCoverage >= options.minimumBoundaryCoverage;
}
} // namespace reconstruction::detail::curved
