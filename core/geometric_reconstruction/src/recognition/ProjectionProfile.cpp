#include "ProjectionInternal.h"

#include "preprocessing/detail/KdTree.h"
#include "recognition/assembly/BoundaryGeometry.h"

#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>

namespace reconstruction::detail::projection {
using namespace modeling;
using assembly::BoundaryLoop;
using assembly::boundaryArea;

namespace {
std::optional<Circle2D> fitCircle(const BoundaryLoop& loop, double tolerance) {
    if (loop.size() < 8) return {};
    Eigen::MatrixXd design(loop.size(), 3);
    Eigen::VectorXd rhs(loop.size());
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const auto p = loop[i];
        design.row(i) << 2*p.x, 2*p.y, 1;
        rhs[i] = p.x*p.x+p.y*p.y;
    }
    const auto qr = design.colPivHouseholderQr();
    if (qr.rank() < 3) return {};
    const Eigen::Vector3d fitted = qr.solve(rhs);
    const auto radius = std::sqrt(fitted.z()+fitted.head<2>().squaredNorm());
    if (!std::isfinite(radius) || radius < 5*tolerance) return {};
    std::vector<double> angles;
    for (const auto p : loop) {
        if (std::abs(std::hypot(p.x-fitted.x(), p.y-fitted.y())-radius) > tolerance) return {};
        angles.push_back(std::atan2(p.y-fitted.y(), p.x-fitted.x()));
    }
    std::sort(angles.begin(), angles.end());
    angles.push_back(angles.front()+2*pi);
    for (std::size_t i = 1; i < angles.size(); ++i)
        if (angles[i]-angles[i-1] > pi/3) return {};
    return Circle2D{fitted.x(), fitted.y(), radius};
}

bool hasInnerWall(const BoundaryLoop& loop, const Frame& frame, const Cap& first,
    const Cap& last, const std::vector<PointSample>& points, double reach) {
    const double depth = last.depth-first.depth;
    if (depth <= 0) return false;
    std::array<std::size_t,5> bands{};
    for (const auto& p : points) {
        if (!p.normal || std::abs(dot(*p.normal,frame.axis)) > .3) continue;
        const double position = (frame.depth(p.position)-first.depth)/depth;
        if (position <= .1 || position >= .9) continue;
        const auto uv = frame.project(p.position);
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const auto a = loop[i], b = loop[(i+1)%loop.size()];
            const double x = b.x-a.x, y = b.y-a.y, edge = x*x+y*y;
            if (edge <= 0) continue;
            const double t = std::clamp(((uv.x-a.x)*x+(uv.y-a.y)*y)/edge,0.,1.);
            if (std::hypot(uv.x-a.x-t*x,uv.y-a.y-t*y) > reach) continue;
            ++bands[std::size_t(position*bands.size())];
            break;
        }
    }
    return std::count_if(bands.begin(),bands.end(),[](auto count) { return count>=2; }) >= 3;
}

double capSpacing(const Cap& cap) {
    const detail::preprocessing::KdTree tree(cap.points);
    std::vector<double> distances;
    const auto stride = std::max<std::size_t>(1, cap.points.size()/256);
    for (std::size_t i = 0; i < cap.points.size(); i += stride) {
        const auto neighbors = tree.nearest(i, 4);
        if (!neighbors.empty()) distances.push_back(std::sqrt(neighbors.back().first));
    }
    const auto median = distances.begin()+distances.size()/2;
    std::nth_element(distances.begin(), median, distances.end());
    return *median;
}

} // namespace

std::vector<SketchEntity> recoverProfile(const Cap& cap, const Frame& frame, double tolerance,
                                  const std::vector<PrimitiveEvidence>& evidence,
                                  const Cap& first, const Cap& last,
                                  const std::vector<PointSample>& points) {
    if (cap.points.size() < minimumCapPoints) return {};
    BoundaryLoop projected;
    for (const auto& p : cap.points) projected.push_back(frame.project(p.position));
    const double spacing = capSpacing(cap);
    BoundaryRecoveryDiagnostics diagnostics;
    auto loops = detail::assembly::alphaLoops(projected, std::max(2*spacing, 3*tolerance),
                                               tolerance, diagnostics);
    std::vector<detail::assembly::BoundaryLine> sideLines;
    for (const auto& primitive : evidence) {
        const auto* plane = std::get_if<PlaneEvidence>(&primitive);
        if (!plane || std::abs(dot(plane->plane.normal, frame.axis)) > .08) continue;
        const auto normal = plane->plane.normal;
        sideLines.push_back({dot(normal,frame.x), dot(normal,frame.y),
            dot(normal,frame.plane(cap.depth).origin)+plane->plane.offset,
            int(sideLines.size()+1)});
    }
    auto snapped = loops;
    for (auto& loop : snapped) {
        std::size_t count = 0;
        loop = detail::assembly::snapBoundary(loop, sideLines, std::max(spacing,tolerance),
                                               2*std::max(spacing,tolerance), diagnostics, count);
    }
    if (detail::assembly::sameBoundaryTopology(loops, snapped)) loops = std::move(snapped);
    std::vector<Circle2D> holes;
    for (const auto& primitive : evidence) {
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < .8 || loops.empty() ||
            std::abs(dot(wall->cylinder.axisDirection,frame.axis)) < axisAlignment) continue;
        const auto& cylinder = wall->cylinder;
        const auto min = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMinimum)));
        const auto max = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMaximum)));
        if (std::abs(std::min(min,max)-first.depth) > 4*tolerance ||
            std::abs(std::max(min,max)-last.depth) > 4*tolerance) continue;
        const auto center = frame.project(cylinder.axisPoint);
        const auto distance = [&](Point2D p) { return std::hypot(p.x-center.x,p.y-center.y); };
        const double radius = cylinder.radius;
        // The opening must lie inside the outer profile, with material
        // observed around it and no conflicting cap samples in its interior.
        bool inside = false;
        const auto& outer = loops.front();
        for (std::size_t i = 0; i < outer.size(); ++i) {
            const auto a = outer[i], b = outer[(i+1)%outer.size()];
            if ((a.y > center.y) != (b.y > center.y) &&
                center.x < a.x+(b.x-a.x)*(center.y-a.y)/(b.y-a.y)) inside = !inside;
        }
        if (!inside || std::any_of(outer.begin(),outer.end(),[&](auto p) {
            return distance(p) < radius+2*tolerance;
        })) continue;
        const auto blocks = [&](const Cap& end) {
            return std::any_of(end.points.begin(),end.points.end(),[&](const auto& p) {
                return distance(frame.project(p.position)) < radius-2*tolerance;
            });
        };
        if (blocks(first) || blocks(last)) continue;
        if (std::any_of(holes.begin(),holes.end(),[&](const auto& hole) {
            return distance({hole.x,hole.y}) < 2*tolerance && std::abs(hole.radius-radius)<2*tolerance;
        })) continue;
        loops.erase(std::remove_if(loops.begin()+1,loops.end(),[&](const auto& loop) {
            return std::all_of(loop.begin(),loop.end(),[&](auto p) {
                return std::abs(distance(p)-radius) < 2*spacing+2*tolerance;
            });
        }),loops.end());
        holes.push_back({center.x,center.y,radius});
    }
    std::vector<SketchEntity> entities;
    for (std::size_t index = 0; index < loops.size(); ++index) {
        const auto& loop = loops[index];
        // An interior sampling gap only becomes a hole with lateral support
        // at several depths. Keep separate islands for the CAD validity check.
        if (index && boundaryArea(loop)*boundaryArea(loops.front()) < 0 &&
            !hasInnerWall(loop,frame,first,last,points,std::max(2*tolerance,.5*spacing))) continue;
        if (const auto circle = fitCircle(loop, tolerance)) {
            entities.push_back({SketchEntityId(entities.size()+1), *circle});
            continue;
        }
        if (loop.size() > 80) return {};
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const auto a = loop[i], b = loop[(i+1)%loop.size()];
            entities.push_back({SketchEntityId(entities.size()+1), Line2D{a.x,a.y,b.x,b.y}});
        }
    }
    for (const auto& hole : holes) entities.push_back({SketchEntityId(entities.size()+1),hole});
    return entities;
}

} // namespace reconstruction::detail::projection
