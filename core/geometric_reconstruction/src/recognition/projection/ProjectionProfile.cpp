#include "ProjectionInternal.h"

#include "preprocessing/detail/KdTree.h"
#include "recognition/assembly/BoundaryGeometry.h"

#include <contour_fitting/ContourFitting.h>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>

namespace reconstruction::detail::projection {
using namespace modeling;
using assembly::BoundaryLoop;
using assembly::boundaryArea;

namespace {
struct FittedLoops {
    std::vector<BoundaryLoop> rings; // One outer ring followed by its holes.
    double simplificationTolerance = 0;
};

FittedLoops fitProfileLoops(const ProjectionSketchInput& input, const ProjectionExtrusionOptions& options) {
    auto fitted = fitProjectionSketchBoundary(input);
    FittedLoops result{{}, fitted.simplificationTolerance};
    const double minimumArea = options.minimumRingAreaMultiplier*fitted.simplificationTolerance*fitted.simplificationTolerance;
    const auto append = [&](BoundaryLoop& loop) {
        const double area = boundaryArea(loop);
        if (std::abs(area) <= minimumArea) return false;
        // The sketch builder treats each wire as a face before assigning holes.
        if (area < 0) std::reverse(loop.begin(),loop.end());
        result.rings.push_back(std::move(loop));
        return true;
    };
    for (auto& contour : fitted.regions) {
        if (!append(contour.outer)) continue;
        // The CAD sketch accepts one connected region per extrusion.
        if (result.rings.size() > 1) return {{}, fitted.simplificationTolerance};
        for (auto& hole : contour.holes) append(hole);
    }
    return result;
}

std::optional<Circle2D> fitCircle(const BoundaryLoop& loop, double tolerance, const ProjectionExtrusionOptions& options) {
    if (loop.size() < options.minimumCircleVertices) return {};
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
    if (!std::isfinite(radius) || radius < options.minimumCircleRadiusMultiplier*tolerance) return {};
    std::vector<double> angles;
    for (const auto p : loop) {
        if (std::abs(std::hypot(p.x-fitted.x(), p.y-fitted.y())-radius) > tolerance) return {};
        angles.push_back(std::atan2(p.y-fitted.y(), p.x-fitted.x()));
    }
    std::sort(angles.begin(), angles.end());
    angles.push_back(angles.front()+2*pi);
    for (std::size_t i = 1; i < angles.size(); ++i)
        if (angles[i]-angles[i-1] > options.maximumCircleGapDegrees*pi/180) return {};
    return Circle2D{fitted.x(), fitted.y(), radius};
}

bool hasInnerWall(const BoundaryLoop& loop, const Frame& frame, const Cap& first,
    const Cap& last, const std::vector<PointSample>& points, double reach, const ProjectionExtrusionOptions& options) {
    const double depth = last.depth-first.depth;
    if (depth <= 0) return false;
    std::vector<std::size_t> bands(options.innerWallDepthBands);
    for (const auto& p : points) {
        if (!p.normal || std::abs(dot(*p.normal,frame.axis)) > options.innerWallNormalCosine) continue;
        const double position = (frame.depth(p.position)-first.depth)/depth;
        if (position <= options.innerWallEndTrimFraction || position >= 1-options.innerWallEndTrimFraction) continue;
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
    return std::size_t(std::count_if(bands.begin(),bands.end(),[&](auto count) {
        return count >= options.innerWallMinimumPointsPerBand;
    })) >= options.innerWallMinimumBands;
}

} // namespace

double capSpacing(const Cap& cap, const ProjectionExtrusionOptions& options) {
    const detail::preprocessing::KdTree tree(cap.points);
    std::vector<double> distances;
    const auto stride = std::max<std::size_t>(1, cap.points.size()/options.capSpacingSamples);
    for (std::size_t i = 0; i < cap.points.size(); i += stride) {
        const auto neighbors = tree.nearest(i, options.capSpacingNeighbors);
        if (!neighbors.empty()) distances.push_back(std::sqrt(neighbors.back().first));
    }
    const auto median = distances.begin()+distances.size()/2;
    std::nth_element(distances.begin(), median, distances.end());
    return *median;
}

namespace {

std::vector<SketchEntity> profileEntities(const std::vector<BoundaryLoop>& rings, const Frame& frame,
    const Cap& first, const Cap& last, const std::vector<PointSample>& points, double tolerance, double wallReach,
    const ProjectionExtrusionOptions& options) {
    std::vector<SketchEntity> entities;
    for (std::size_t index = 0; index < rings.size(); ++index) {
        const auto& loop = rings[index];
        // Interior sampling gaps only become holes with wall support at several depths.
        if (index && !hasInnerWall(loop,frame,first,last,points,wallReach,options)) continue;
        if (const auto circle = fitCircle(loop, tolerance, options)) {
            entities.push_back({SketchEntityId(entities.size()+1), *circle});
            continue;
        }
        if (loop.size() > options.maximumSketchEdges) return {};
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const auto a = loop[i], b = loop[(i+1)%loop.size()];
            entities.push_back({SketchEntityId(entities.size()+1), Line2D{a.x,a.y,b.x,b.y}});
        }
    }
    return entities;
}

} // namespace

ProjectionSketchInput projectCap(const Cap& cap, const Frame& frame, double spacing, double tolerance,
    const ProjectionExtrusionOptions& options) {
    ProjectionSketchInput input;
    input.plane = frame.plane(cap.depth);
    input.alphaRadius = std::max(options.capAlphaSpacingMultiplier*spacing, options.capAlphaToleranceMultiplier*tolerance);
    input.simplificationTolerance = options.capSimplificationTolerance > 0 ? options.capSimplificationTolerance : tolerance;
    input.points.reserve(cap.points.size());
    for (const auto& sample : cap.points) input.points.push_back(frame.project(sample.position));
    if (!cap.points.empty()) input.firstPointId = cap.points.front().id;
    return input;
}

ProjectionSketchInput projectEnvelope(const std::vector<PointSample>& points,
    const Frame& frame, double start, double end, double tolerance, const ProjectionExtrusionOptions& options) {
    ProjectionSketchInput input;
    input.plane = frame.plane(start);
    input.regionSupport = options.intersectionRegionSupport;
    input.alphaRadius = options.intersectionAlphaRadius;
    input.automaticAlphaScale = options.intersectionAlphaScale;
    if (options.intersectionSimplificationTolerance > 0)
        input.simplificationTolerance = options.intersectionSimplificationTolerance;
    input.points.reserve(points.size());
    const double padding = options.projectionDepthPaddingMultiplier*tolerance;
    for (const auto& sample : points) {
        const double depth = frame.depth(sample.position);
        if (depth < start-padding || depth > end+padding) continue;
        input.points.push_back(frame.project(sample.position));
    }
    return input;
}

std::optional<ProjectionOperation> projectionEnvelope(const std::vector<PointSample>& points,
    const Frame& frame, double start, double end, double tolerance, const ProjectionExtrusionOptions& options) {
    if (end-start <= options.minimumDepthMultiplier*tolerance) return {};
    const auto input = projectEnvelope(points,frame,start,end,tolerance,options);
    const auto fitted = fitProfileLoops(input,options);
    const double localTolerance = fitted.simplificationTolerance;
    auto entities = profileEntities(fitted.rings,frame,Cap{start,{}},Cap{end,{}},points,
        localTolerance,options.innerWallReachMultiplier*localTolerance,options);
    if (entities.empty()) return {};
    ProjectionOperation operation;
    operation.sketch = {input.plane,std::move(entities)};
    operation.depth = end-start;
    return operation;
}

std::vector<SketchEntity> recoverProfile(const Cap& cap, const Frame& frame, double tolerance,
                                  const std::vector<PrimitiveEvidence>& evidence,
                                  const Cap& first, const Cap& last,
                                  const std::vector<PointSample>& points, const ProjectionExtrusionOptions& options) {
    if (cap.points.size() < options.minimumCapPoints) return {};
    const double spacing = capSpacing(cap,options);
    BoundaryRecoveryDiagnostics diagnostics;
    auto loops = fitProfileLoops(projectCap(cap,frame,spacing,tolerance,options),options).rings;
    if (loops.empty()) return {};
    std::vector<detail::assembly::BoundaryLine> sideLines;
    for (const auto& primitive : evidence) {
        const auto* plane = std::get_if<PlaneEvidence>(&primitive);
        if (!plane || std::abs(dot(plane->plane.normal, frame.axis)) > options.sideSnapNormalCosine) continue;
        const auto normal = plane->plane.normal;
        sideLines.push_back({dot(normal,frame.x), dot(normal,frame.y),
            dot(normal,frame.plane(cap.depth).origin)+plane->plane.offset,
            int(sideLines.size()+1)});
    }
    auto snapped = loops;
    for (auto& loop : snapped) {
        std::size_t count = 0;
        loop = detail::assembly::snapBoundary(loop, sideLines, std::max(spacing,tolerance),
                                               options.sideSnapReachMultiplier*std::max(spacing,tolerance), diagnostics, count);
    }
    if (detail::assembly::sameBoundaryTopology(loops, snapped)) loops = std::move(snapped);
    std::vector<Circle2D> holes;
    for (const auto& primitive : evidence) {
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < options.cappedCylinderCoverage || loops.empty() ||
            std::abs(dot(wall->cylinder.axisDirection,frame.axis)) < options.axisAlignmentCosine) continue;
        const auto& cylinder = wall->cylinder;
        const auto min = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMinimum)));
        const auto max = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMaximum)));
        if (std::abs(std::min(min,max)-first.depth) > options.cylinderEndBandMultiplier*tolerance ||
            std::abs(std::max(min,max)-last.depth) > options.cylinderEndBandMultiplier*tolerance) continue;
        const auto center = frame.project(cylinder.axisPoint);
        const auto distance = [&](Point2D p) { return std::hypot(p.x-center.x,p.y-center.y); };
        const double radius = cylinder.radius;
        const double clearance = options.cylinderOpeningClearanceMultiplier*tolerance;
        const double mergeTolerance = options.cylinderMergeToleranceMultiplier*tolerance;
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
            return distance(p) < radius+clearance;
        })) continue;
        const auto blocks = [&](const Cap& end) {
            return std::any_of(end.points.begin(),end.points.end(),[&](const auto& p) {
                return distance(frame.project(p.position)) < radius-clearance;
            });
        };
        if (blocks(first) || blocks(last)) continue;
        if (std::any_of(holes.begin(),holes.end(),[&](const auto& hole) {
            return distance({hole.x,hole.y}) < mergeTolerance && std::abs(hole.radius-radius) < mergeTolerance;
        })) continue;
        loops.erase(std::remove_if(loops.begin()+1,loops.end(),[&](const auto& loop) {
            return std::all_of(loop.begin(),loop.end(),[&](auto p) {
                return std::abs(distance(p)-radius) < options.cylinderMergeSpacingMultiplier*spacing+mergeTolerance;
            });
        }),loops.end());
        holes.push_back({center.x,center.y,radius});
    }
    auto entities = profileEntities(loops,frame,first,last,points,tolerance,
        std::max(options.innerWallReachMultiplier*tolerance,options.capWallSpacingMultiplier*spacing),options);
    if (entities.empty()) return {};
    for (const auto& hole : holes) entities.push_back({SketchEntityId(entities.size()+1),hole});
    return entities;
}

} // namespace reconstruction::detail::projection

namespace reconstruction {
ProjectionSketchBoundary fitProjectionSketchBoundary(const ProjectionSketchInput& input) {
    std::vector<contour_fitting::Point2D> points;
    points.reserve(input.points.size());
    for (const auto& point : input.points) points.push_back({point.x,point.y});
    const auto fitted = contour_fitting::fitContours(points,{input.alphaRadius,input.simplificationTolerance,
        input.regionSupport.minimumPoints,input.regionSupport.minimumFraction,input.automaticAlphaScale});
    ProjectionSketchBoundary boundary{{},fitted.alphaRadius,fitted.simplificationTolerance,fitted.discardedRegionCount};
    boundary.regions.reserve(fitted.contours.size());
    const auto convert = [](const contour_fitting::Ring& ring) {
        std::vector<modeling::Point2D> result;
        result.reserve(ring.size());
        for (const auto& point : ring) result.push_back({point.x,point.y});
        return result;
    };
    for (const auto& contour : fitted.contours) {
        ProjectionSketchRegion region{convert(contour.outer),{}};
        for (const auto& hole : contour.holes) region.holes.push_back(convert(hole));
        boundary.regions.push_back(std::move(region));
    }
    return boundary;
}
} // namespace reconstruction
