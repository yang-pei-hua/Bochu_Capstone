#include "ProjectionInternal.h"

#include <algorithm>
#include <cmath>

namespace reconstruction::detail::projection {
using namespace modeling;
using math3::normalized;

namespace {
std::optional<ProjectionOperation> slotCut(const Cap& floor, const Cap& opening,
    const Frame& frame, const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance) {
    auto entities = recoverProfile(floor,frame,tolerance,evidence,floor,floor,points);
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
            if (!p.normal || std::abs(dot(*p.normal,normal)) < .95) continue;
            const double t = sign*(frame.depth(p.position)-floor.depth);
            if (t < 0 || t > depth) continue;
            const auto uv = frame.project(p.position);
            const double x = uv.x-edge->x1, y = uv.y-edge->y1;
            const double along = (x*dx+y*dy)/length;
            if (along < 0 || along > length || std::abs((y*dx-x*dy)/length) > 2*tolerance)
                continue;
            low = std::min(low,t);
            high = std::max(high,t);
            wall.push_back(p.id);
        }
        if (wall.size() >= 6 && high-low > .6*depth) {
            ++supportedEdges;
            support.insert(support.end(),wall.begin(),wall.end());
        }
    }
    if (supportedEdges < 2) return {};
    for (const auto& p : floor.points) support.push_back(p.id);
    ProjectionOperation cut;
    cut.sketch = {frame.plane(opening.depth+sign*tolerance),std::move(entities)};
    cut.operation = ExtrudeOperation::Cut;
    cut.depth = depth+tolerance;
    cut.reverse = sign > 0;
    cut.supportPointIds = std::move(support);
    return cut;
}
} // namespace

std::vector<ProjectionOperation> blindHoles(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance) {
    std::vector<ProjectionOperation> cuts;
    for (const auto& primitive : evidence) {
        const auto* wall = std::get_if<CylinderEvidence>(&primitive);
        if (!wall || wall->angularCoverage < .65) continue;
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
                if (!p.normal || std::abs(dot(*p.normal, frame.axis)) < .95) continue;
                const double t = frame.depth(p.position);
                const auto uv = frame.project(p.position);
                const auto r = std::hypot(uv.x, uv.y);
                if (r < radius-tolerance && std::abs(t-floor) < 4*tolerance) {
                    floorDepth += t;
                    ++floorCount;
                    support.push_back(p.id);
                }
                if (r > radius+tolerance && r < 2*radius && std::abs(t-opening) < 4*tolerance) {
                    openingDepth += t;
                    ++openingCount;
                    support.push_back(p.id);
                }
            }
            if (floorCount < 6 || openingCount < 6) continue;
            floorDepth /= floorCount;
            openingDepth /= openingCount;
            const double depth = sign*(openingDepth-floorDepth);
            if (depth <= 3*tolerance) continue;
            ProjectionOperation cut;
            cut.sketch.plane = frame.plane(openingDepth+sign*tolerance);
            cut.sketch.entities = {{1, Circle2D{0,0,radius}}};
            cut.depth = depth+tolerance;
            cut.reverse = sign > 0;
            cut.operation = ExtrudeOperation::Cut;
            cut.supportPointIds = std::move(support);
            cuts.push_back(std::move(cut));
        }
    }
    return cuts;
}

bool proposeAxisOperations(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, const Frame& frame,
    const Cap& first, const Cap& last, double tolerance, bool includeSecondary,
    std::vector<AxisProposal>& proposals, const std::function<bool()>& cancelled,
    const EndProfiles* endProfiles) {
    proposals.clear();
    auto proposeBase = [&](const Cap& section, const Cap& lowCap, const Cap& highCap, bool allowJoin,
                           const std::vector<SketchEntity>* profile = nullptr) {
        ProjectionOperation operation;
        operation.sketch.plane = frame.plane(lowCap.depth);
        operation.sketch.entities = profile ? *profile : recoverProfile(section, frame, tolerance, evidence, lowCap, highCap, points);
        if (operation.sketch.entities.empty()) return;
        operation.depth = highCap.depth-lowCap.depth;
        for (const auto& point : section.points) operation.supportPointIds.push_back(point.id);
        proposals.push_back({std::move(operation), allowJoin});
    };
    proposeBase(first, first, last, false, endProfiles ? &(*endProfiles)[0] : nullptr);
    proposeBase(last, first, last, false, endProfiles ? &(*endProfiles)[1] : nullptr);
    if (!includeSecondary) return true;

    std::vector<double> interior;
    for (const auto& primitive : evidence) {
        if (cancelled && cancelled()) return false;
        const auto* plane = std::get_if<PlaneEvidence>(&primitive);
        if (!plane) continue;
        const double alignment = dot(plane->plane.normal, frame.axis);
        if (std::abs(alignment) < axisAlignment) continue;
        const double depth = -(dot(plane->plane.normal,frame.origin)+plane->plane.offset)/alignment;
        if (depth-first.depth < 5*tolerance || last.depth-depth < 5*tolerance ||
            std::any_of(interior.begin(),interior.end(),[&](double t) {
                return std::abs(t-depth)<4*tolerance;
            })) continue;
        const auto middle = capAt(points,frame,depth,tolerance);
        if (!middle) continue;
        interior.push_back(depth);
        proposeBase(first,first,*middle,true);
        proposeBase(last,*middle,last,true);
        for (const auto* opening : {&first,&last})
            if (auto cut = slotCut(*middle,*opening,frame,points,evidence,tolerance))
                proposals.push_back({std::move(*cut), false});
        if (interior.size()==2) break;
    }
    return true;
}
} // namespace reconstruction::detail::projection
