#include "ProjectionInternal.h"

#include <algorithm>
#include <cmath>

namespace reconstruction::detail::projection {
using namespace modeling;
using math3::normalized;

namespace {
std::optional<Circle2D> exteriorCircle(const Frame& frame, const Cap& section,
    double start, double end, const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance,
    const ProjectionExtrusionOptions& options) {
    for (const auto& primitive : evidence) {
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < options.cappedCylinderCoverage) continue;
        const auto& cylinder = wall->cylinder;
        if (cylinder.radius <= options.minimumDepthMultiplier*tolerance || std::abs(dot(cylinder.axisDirection,frame.axis)) < options.axisAlignmentCosine)
            continue;
        const double first = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMinimum)));
        const double last = frame.depth(add(cylinder.axisPoint,scale(cylinder.axisDirection,wall->axialMaximum)));
        if (std::abs(std::min(first,last)-start) > options.cylinderEndBandMultiplier*tolerance ||
            std::abs(std::max(first,last)-end) > options.cylinderEndBandMultiplier*tolerance) continue;
        const auto center = frame.project(cylinder.axisPoint);
        const auto outside = [&](const PointSample& point) {
            const auto uv = frame.project(point.position);
            return std::hypot(uv.x-center.x,uv.y-center.y) > cylinder.radius+options.cylinderOpeningClearanceMultiplier*tolerance;
        };
        if (std::size_t(std::count_if(section.points.begin(),section.points.end(),outside)) > section.points.size()*options.exteriorCircleOutsideFraction)
            continue;
        std::size_t outward = 0, inward = 0;
        for (const auto& p : points) {
            const double depth = frame.depth(p.position);
            if (!p.normal || !p.normalOrientationKnown || depth <= start || depth >= end) continue;
            const auto uv = frame.project(p.position);
            const double x = uv.x-center.x, y = uv.y-center.y, radius = std::hypot(x,y);
            if (std::abs(radius-cylinder.radius) > tolerance) continue;
            const double alignment = dot(*p.normal,add(scale(frame.x,x/radius),scale(frame.y,y/radius)));
            if (alignment > options.cylinderRadialNormalCosine) ++outward;
            if (alignment < -options.cylinderRadialNormalCosine) ++inward;
        }
        if (outward >= options.minimumCapPoints && inward <= outward*options.cylinderOpposingNormalFraction)
            return Circle2D{center.x,center.y,cylinder.radius};
    }
    return {};
}

std::optional<ProjectionOperation> slotCut(const Cap& floor, const Cap& opening,
    const Frame& frame, const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance,
    const ProjectionExtrusionOptions& options) {
    auto entities = recoverProfile(floor,frame,tolerance,evidence,floor,floor,points,options);
    if (entities.size() != 4) return {};
    const double depth = std::abs(opening.depth-floor.depth);
    const double sign = opening.depth > floor.depth ? 1 : -1;
    std::vector<PointId> support;
    unsigned supportedEdges = 0;
    for (const auto& entity : entities) {
        const auto* edge = std::get_if<Line2D>(&entity.geometry);
        if (!edge) return {};
        const double dx = edge->x2-edge->x1, dy = edge->y2-edge->y1;
        const double length = std::hypot(dx,dy);
        if (length <= tolerance) return {};
        const auto normal = add(scale(frame.x,-dy/length),scale(frame.y,dx/length));
        std::vector<PointId> wall;
        double low = depth, high = 0;
        for (const auto& p : points) {
            if (!p.normal || std::abs(dot(*p.normal,normal)) < options.slotWallNormalCosine) continue;
            const double t = sign*(frame.depth(p.position)-floor.depth);
            if (t < 0 || t > depth) continue;
            const auto uv = frame.project(p.position);
            const double x = uv.x-edge->x1, y = uv.y-edge->y1;
            const double along = (x*dx+y*dy)/length;
            if (along < 0 || along > length || std::abs((y*dx-x*dy)/length) > options.slotWallDistanceMultiplier*tolerance)
                continue;
            low = std::min(low,t);
            high = std::max(high,t);
            wall.push_back(p.id);
        }
        if (wall.size() >= options.slotWallMinimumPoints && high-low > options.slotWallMinimumDepthFraction*depth) {
            ++supportedEdges;
            support.insert(support.end(),wall.begin(),wall.end());
        }
    }
    if (supportedEdges < options.minimumSlotSupportedEdges) return {};
    for (const auto& p : floor.points) support.push_back(p.id);
    ProjectionOperation cut;
    cut.sketch = {frame.plane(opening.depth+sign*options.cutOvertravelMultiplier*tolerance),std::move(entities)};
    cut.operation = ExtrudeOperation::Cut;
    cut.depth = depth+options.cutOvertravelMultiplier*tolerance;
    cut.reverse = sign > 0;
    cut.supportPointIds = std::move(support);
    return cut;
}
} // namespace

std::vector<ProjectionOperation> blindHoles(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance,
    const ProjectionExtrusionOptions& options) {
    std::vector<ProjectionOperation> cuts;
    for (const auto& primitive : evidence) {
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < options.cutCylinderCoverage) continue;
        const auto& cylinder = wall->cylinder;
        const auto frame = makeFrame(cylinder.axisPoint, normalized(cylinder.axisDirection));
        const double radius = cylinder.radius;
        for (const double sign : {-1., 1.}) {
            const double floor = sign > 0 ? wall->axialMinimum : wall->axialMaximum;
            const double opening = sign > 0 ? wall->axialMaximum : wall->axialMinimum;
            double floorDepth = 0, openingDepth = 0;
            std::size_t floorCount = 0, openingCount = 0;
            std::vector<PointId> support = wall->supportPointIds;
            for (const auto& p : points) {
                if (!p.normal || std::abs(dot(*p.normal, frame.axis)) < options.blindHoleNormalCosine) continue;
                const double t = frame.depth(p.position);
                const auto uv = frame.project(p.position);
                const auto r = std::hypot(uv.x, uv.y);
                if (r < radius-tolerance && std::abs(t-floor) < options.cylinderEndBandMultiplier*tolerance) {
                    floorDepth += t;
                    ++floorCount;
                    support.push_back(p.id);
                }
                if (r > radius+tolerance && r < options.holeOpeningOuterRadiusMultiplier*radius && std::abs(t-opening) < options.cylinderEndBandMultiplier*tolerance) {
                    openingDepth += t;
                    ++openingCount;
                    support.push_back(p.id);
                }
            }
            if (floorCount < options.minimumOpeningPoints || openingCount < options.minimumOpeningPoints) continue;
            floorDepth /= floorCount;
            openingDepth /= openingCount;
            const double depth = sign*(openingDepth-floorDepth);
            if (depth <= options.minimumDepthMultiplier*tolerance) continue;
            ProjectionOperation cut;
            cut.sketch.plane = frame.plane(openingDepth+sign*options.cutOvertravelMultiplier*tolerance);
            cut.sketch.entities = {{1, Circle2D{0,0,radius}}};
            cut.depth = depth+options.cutOvertravelMultiplier*tolerance;
            cut.reverse = sign > 0;
            cut.operation = ExtrudeOperation::Cut;
            cut.supportPointIds = std::move(support);
            cuts.push_back(std::move(cut));
        }
    }
    return cuts;
}

std::vector<ProjectionOperation> throughHoles(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance,
    const std::function<bool()>& cancelled, const ProjectionExtrusionOptions& options) {
    std::vector<ProjectionOperation> cuts;
    for (const auto& primitive : evidence) {
        if (cancelled && cancelled()) break;
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < options.cutCylinderCoverage) continue;
        const auto& cylinder = wall->cylinder;
        const double radius = cylinder.radius;
        const double low = wall->axialMinimum, high = wall->axialMaximum;
        const double span = high-low;
        if (span <= 0) continue;
        // The body tolerance grows with the component diagonal. For a thin
        // drilled wall, keep the two opening neighborhoods separate.
        const double featureTolerance = std::min(tolerance,span*options.throughHoleToleranceSpanFraction);
        if (radius <= options.throughHoleMinimumRadiusMultiplier*featureTolerance ||
            wall->meanAbsoluteResidual > options.throughHoleMaximumResidualMultiplier*featureTolerance) continue;
        const auto frame = makeFrame(cylinder.axisPoint,normalized(cylinder.axisDirection));
        const double endBand = std::min(options.throughHoleOpeningBandMultiplier*featureTolerance,span*options.throughHoleOpeningBandFraction);
        std::array<std::vector<double>, 2> openings;
        std::vector<PointId> support;
        std::size_t inward = 0, outward = 0;
        bool blocked = false;
        for (const auto& p : points) {
            if (!p.normal || !p.normalOrientationKnown) continue;
            const double depth = frame.depth(p.position);
            const auto uv = frame.project(p.position);
            const double r = std::hypot(uv.x,uv.y);
            const double axialNormal = dot(*p.normal,frame.axis);
            for (std::size_t end = 0; end < 2; ++end) {
                const double direction = end == 0 ? -1 : 1;
                const double boundary = end == 0 ? low : high;
                if (std::abs(depth-boundary) > endBand || direction*axialNormal < options.throughHoleOpeningNormalCosine) continue;
                if (r < radius-featureTolerance) blocked = true;
                if (r >= radius+featureTolerance && r <= options.holeOpeningOuterRadiusMultiplier*radius+featureTolerance) {
                    openings[end].push_back(depth);
                    support.push_back(p.id);
                }
            }
            if (depth <= low || depth >= high || std::abs(r-radius) > featureTolerance) continue;
            const auto radial = add(scale(frame.x,uv.x/r),scale(frame.y,uv.y/r));
            const double alignment = dot(*p.normal,radial);
            if (alignment < -options.cylinderRadialNormalCosine) {
                ++inward;
                support.push_back(p.id);
            }
            if (alignment > options.cylinderRadialNormalCosine) ++outward;
        }
        if (blocked || inward < options.minimumCapPoints || outward > inward*options.cylinderOpposingNormalFraction ||
            openings[0].size() < options.minimumOpeningPoints || openings[1].size() < options.minimumOpeningPoints) continue;
        for (auto& opening : openings) std::sort(opening.begin(),opening.end());
        const double start = openings[0][openings[0].size()/2];
        const double end = openings[1][openings[1].size()/2];
        if (end-start <= options.minimumDepthMultiplier*featureTolerance) continue;
        const double margin = std::min(options.throughHoleCutMarginToleranceFraction*tolerance,(end-start)*options.throughHoleCutMarginDepthFraction);
        ProjectionOperation cut;
        cut.sketch = {frame.plane(start-margin),{{1,Circle2D{0,0,radius}}}};
        cut.operation = ExtrudeOperation::Cut;
        cut.depth = end-start+2*margin;
        cut.supportPointIds = std::move(support);
        cuts.push_back(std::move(cut));
    }
    return cuts;
}

bool proposeAxisOperations(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, const Frame& frame,
    const Cap& first, const Cap& last, double tolerance, bool includeSecondary,
    std::vector<AxisProposal>& proposals, const std::function<bool()>& cancelled,
    const EndProfiles* endProfiles, std::vector<OperationSequence>* segmented,
    const ProjectionExtrusionOptions& options) {
    proposals.clear();
    auto makeBase = [&](const Cap& section, const Cap& lowCap, const Cap& highCap,
                        const std::vector<SketchEntity>* profile = nullptr) {
        ProjectionOperation operation;
        operation.sketch.plane = frame.plane(lowCap.depth);
        operation.sketch.entities = profile ? *profile : recoverProfile(section, frame, tolerance, evidence, lowCap, highCap, points, options);
        operation.depth = highCap.depth-lowCap.depth;
        for (const auto& point : section.points) operation.supportPointIds.push_back(point.id);
        return operation;
    };
    const auto addProposal = [&](ProjectionOperation operation, bool allowJoin) {
        if (!operation.sketch.entities.empty()) proposals.push_back({std::move(operation),allowJoin});
    };
    addProposal(makeBase(first,first,last,endProfiles ? &(*endProfiles)[0] : nullptr),false);
    addProposal(makeBase(last,first,last,endProfiles ? &(*endProfiles)[1] : nullptr),false);
    if (!includeSecondary || !options.maximumInteriorSections) return true;

    std::vector<double> interior;
    for (const auto& primitive : evidence) {
        if (cancelled && cancelled()) return false;
        const auto* plane = std::get_if<PlaneEvidence>(&primitive);
        if (!plane) continue;
        const double alignment = dot(plane->plane.normal, frame.axis);
        if (std::abs(alignment) < options.axisAlignmentCosine) continue;
        const double depth = -(dot(plane->plane.normal,frame.origin)+plane->plane.offset)/alignment;
        if (depth-first.depth < options.interiorSectionEndMultiplier*tolerance || last.depth-depth < options.interiorSectionEndMultiplier*tolerance ||
            std::any_of(interior.begin(),interior.end(),[&](double t) {
                return std::abs(t-depth)<options.interiorSectionMergeMultiplier*tolerance;
            })) continue;
        const auto middle = capAt(points,frame,depth,tolerance,options);
        if (!middle) continue;
        interior.push_back(depth);
        auto base = makeBase(first,first,*middle);
        auto top = makeBase(last,*middle,last);
        if (segmented) {
            addProposal(base,true);
            addProposal(top,true);
            top.operation = ExtrudeOperation::Join;
            if (!base.sketch.entities.empty() && !top.sketch.entities.empty())
                segmented->push_back({base,top});
            const auto lowerCircle = exteriorCircle(frame,first,first.depth,middle->depth,points,evidence,tolerance,options);
            const auto upperCircle = exteriorCircle(frame,last,middle->depth,last.depth,points,evidence,tolerance,options);
            if (lowerCircle || upperCircle) {
                if (lowerCircle) base.sketch.entities = {{1,*lowerCircle}};
                if (upperCircle) top.sketch.entities = {{1,*upperCircle}};
                if (!base.sketch.entities.empty() && !top.sketch.entities.empty())
                    segmented->push_back({std::move(base),std::move(top)});
            }
        } else {
            addProposal(std::move(base),true);
            addProposal(std::move(top),true);
        }
        for (const auto* opening : {&first,&last})
            if (auto cut = slotCut(*middle,*opening,frame,points,evidence,tolerance,options))
                proposals.push_back({std::move(*cut), false});
        if (interior.size()==options.maximumInteriorSections) break;
    }
    return true;
}
} // namespace reconstruction::detail::projection
