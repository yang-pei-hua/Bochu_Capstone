#include "reconstruction/CurvedSolidRecognizer.h"
#include "Math3.h"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <set>
#include <type_traits>

namespace reconstruction {
namespace {
using modeling::Vec3;
using namespace math3;
constexpr double pi = 3.14159265358979323846;
using Params = modeling::CurvedPrimitiveParams;
Vec3 vec(const Eigen::VectorXd &x, int i) { return {x[i], x[i + 1], x[i + 2]}; }
double radialLength(Vec3 d, Vec3 axis) { return length(subtract(d, scale(axis, dot(d, axis)))); }
Vec3 transverse(Vec3 axis) {
    return normalized(cross(axis, std::abs(axis.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}));
}
int angleBin(double y, double x, int count) {
    double a = std::atan2(y, x);
    if (a < 0)
        a += 2 * pi;
    return std::clamp(int(a / (2 * pi) * count), 0, count - 1);
}
int unitBin(double value, int count) { return std::clamp(int(value * count), 0, count - 1); }

// Robust nonlinear least squares on source observations. Numerical derivatives
// keep sphere/cone/torus parameter refinement separate from CGAL implementation.
bool refine(Eigen::VectorXd &x, const std::vector<const PointSample *> &data, double tolerance,
            const std::function<double(const Eigen::VectorXd &, Vec3)> &residual) {
    if (data.size() < std::size_t(x.size() * 4))
        return false;
    auto cost = [&](const Eigen::VectorXd &p) -> double {
        double sum = 0;
        for (auto point : data) {
            const double r = residual(p, point->position);
            if (!std::isfinite(r))
                return INFINITY;
            sum += std::abs(r) <= tolerance ? r * r
                                            : 2 * tolerance * std::abs(r) - tolerance * tolerance;
        }
        return sum;
    };
    for (int iteration = 0; iteration < 15; ++iteration) {
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(x.size(), x.size());
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(x.size());
        for (auto point : data) {
            const double r = residual(x, point->position);
            if (!std::isfinite(r))
                return false;
            Eigen::VectorXd gradient(x.size());
            for (int j = 0; j < x.size(); ++j) {
                const double delta = 1e-5 * std::max(1.0, std::abs(x[j]));
                auto plus = x, minus = x;
                plus[j] += delta;
                minus[j] -= delta;
                gradient[j] = (residual(plus, point->position) - residual(minus, point->position)) /
                              (2 * delta);
            }
            const double weight = std::min(1.0, tolerance / std::max(std::abs(r), 1e-12));
            matrix += weight * gradient * gradient.transpose();
            rhs -= weight * gradient * r;
        }
        matrix.diagonal().array() += 1e-8;
        const Eigen::VectorXd step = matrix.ldlt().solve(rhs);
        if (!step.allFinite())
            return false;
        const double before = cost(x);
        bool moved = false;
        for (double fraction = 1; fraction >= 1.0 / 64; fraction /= 2) {
            const Eigen::VectorXd next = x + fraction * step;
            if (cost(next) < before) {
                x = next;
                moved = true;
                break;
            }
        }
        if (!moved || step.norm() < 1e-9)
            break;
    }
    return x.allFinite();
}
template <class Evidence>
std::vector<const PointSample *> support(const PointStore &points, const Evidence &e) {
    std::set<PointId> ids(e.supportPointIds.begin(), e.supportPointIds.end());
    std::vector<const PointSample *> data;
    for (const auto &p : points.points())
        if (ids.count(p.id))
            data.push_back(&p);
    std::sort(data.begin(), data.end(), [](auto a, auto b) { return a->id < b->id; });
    return data;
}
double sphereResidual(const Eigen::VectorXd &x, Vec3 p) {
    return length(subtract(p, vec(x, 0))) - x[3];
}
double coneResidual(const Eigen::VectorXd &x, Vec3 p) {
    if (length(vec(x, 3)) < 1e-12 || x[6] <= 0 || x[6] >= pi / 2)
        return INFINITY;
    auto axis = normalized(vec(x, 3)), d = subtract(p, vec(x, 0));
    return radialLength(d, axis) * std::cos(x[6]) - dot(d, axis) * std::sin(x[6]);
}
double torusResidual(const Eigen::VectorXd &x, Vec3 p) {
    if (length(vec(x, 3)) < 1e-12 || x[7] <= 0 || x[6] <= x[7])
        return INFINITY;
    auto axis = normalized(vec(x, 3)), d = subtract(p, vec(x, 0));
    return std::hypot(radialLength(d, axis) - x[6], dot(d, axis)) - x[7];
}

struct Cap {
    double t = 0;
    int sign = 0;
    std::vector<const PointSample *> data;
};
std::vector<Cap> caps(const PointStore &points, Vec3 origin, Vec3 axis, double tolerance,
                      const std::function<double(double)> &radius) {
    struct Item {
        double t;
        const PointSample *p;
        int sign;
    };
    std::vector<Item> items;
    for (const auto &p : points.points()) {
        const double projection = dot(normalized(*p.normal), axis);
        if (std::abs(projection) < 0.98)
            continue;
        auto d = subtract(p.position, origin);
        const double t = dot(d, axis);
        if (radialLength(d, axis) > radius(t) + 2 * tolerance)
            continue;
        items.push_back({t, &p, projection > 0 ? 1 : -1});
    }
    std::sort(items.begin(), items.end(), [](auto a, auto b) { return a.t < b.t; });
    std::vector<Cap> result;
    for (std::size_t i = 0; i < items.size();) {
        auto end = i + 1;
        while (end < items.size() && items[end].t - items[i].t <= 4 * tolerance)
            ++end;
        for (int sign : {-1, 1}) {
            Cap cap;
            cap.sign = sign;
            for (auto j = i; j < end; ++j)
                if (items[j].sign == sign)
                    cap.data.push_back(items[j].p);
            if (cap.data.size() >= std::max<std::size_t>(12, points.size() / 1000)) {
                std::sort(cap.data.begin(), cap.data.end(),
                          [](auto a, auto b) { return a->id < b->id; });
                for (auto p : cap.data)
                    cap.t += dot(subtract(p->position, origin), axis);
                cap.t /= cap.data.size();
                result.push_back(std::move(cap));
            }
        }
        i = end;
    }
    return result;
}
bool ends(const std::vector<Cap> &planes, double &start, double &end, bool pointed) {
    const Cap *low = nullptr;
    const Cap *high = nullptr;
    for (const auto &cap : planes) {
        auto *&chosen = cap.sign < 0 ? low : high;
        if (!chosen || cap.data.size() > chosen->data.size())
            chosen = &cap;
    }
    if (!high || (!pointed && !low))
        return false;
    start = low ? low->t : 0;
    end = high->t;
    return end > start;
}

struct Boundary {
    double distance = INFINITY;
    Vec3 normal{};
};
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
} // namespace
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
    PrimitiveDetectionOptions detection;
    detection.randomSeed = options.randomSeed;
    detection.detectPlanes = false;
    const auto minimum = std::max<std::size_t>(30, points.size() / 8);
    detection.cylinder.distanceThreshold = detection.sphere.distanceThreshold =
        detection.cone.distanceThreshold = detection.torus.distanceThreshold =
            options.distanceTolerance;
    detection.cylinder.minimumSupportPoints = detection.sphere.minimumSupportPoints =
        detection.cone.minimumSupportPoints = detection.torus.minimumSupportPoints = minimum;
    detection.cylinder.probability = detection.sphere.probability = detection.cone.probability =
        detection.torus.probability = 0.001;
    PrimitiveDetectionResult proposals;
    proposals.evidence = options.seeds;
    // Fit each family independently. In a joint run a nearly cylindrical cone
    // or torus can consume the cylinder's observations before parameter checks
    // reject that competing proposal. Complete-solid scoring decides the family.
    for (int family = 0; family < 4; ++family) {
        const auto kind = static_cast<PrimitiveKind>(family + 1);
        if (options.requiredKind && *options.requiredKind != kind) continue;
        detection.detectCylinders = family == 0;
        detection.detectSpheres = family == 1;
        detection.detectCones = family == 2;
        detection.detectTori = family == 3;
        PrimitiveDetectionResult single;
        if (detectPrimitiveEvidence(points, detection, single, error))
            proposals.evidence.insert(proposals.evidence.end(), single.evidence.begin(),
                                      single.evidence.end());
    }
    if (proposals.evidence.empty()) {
        error = "CGAL found no supported curved primitive family";
        return false;
    }
    bool found = false;
    double score = -INFINITY;
    for (const auto &e : proposals.evidence) {
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
