#include "reconstruction/recognition/ProjectionExtrusion.h"

#include "common/Math3.h"
#include "modeling/ModelingCore.h"
#include "preprocessing/detail/KdTree.h"
#include "recognition/assembly/BoundaryGeometry.h"
#include "reconstruction/detection/PrimitiveDetector.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <tuple>
#include <unordered_set>

namespace reconstruction {
namespace {
using namespace math3;
using namespace modeling;
using detail::assembly::BoundaryLoop;
using detail::assembly::boundaryArea;

constexpr double pi = 3.141592653589793;
constexpr double axisAlignment = 0.9986; // Three degrees.
constexpr std::size_t minimumCapPoints = 12;

bool finite(Vec3 p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

Eigen::Vector3d vector(Vec3 p) { return {p.x, p.y, p.z}; }
Vec3 point(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }

std::vector<PointSample> sample(const std::vector<PointSample>& points, std::size_t limit) {
    if (points.size() <= limit) return points;
    auto ordered = points;
    std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
        return std::tie(a.position.x, a.position.y, a.position.z, a.id) <
            std::tie(b.position.x, b.position.y, b.position.z, b.id);
    });
    std::vector<PointSample> result;
    result.reserve(limit);
    for (std::size_t i = 0; i < limit; ++i)
        result.push_back(ordered[std::size_t((i + .5) * ordered.size() / limit)]);
    return result;
}

struct Frame {
    Vec3 origin, axis, x, y;
    Point2D project(Vec3 p) const {
        const auto delta = subtract(p, origin);
        return {dot(delta, x), dot(delta, y)};
    }
    double depth(Vec3 p) const { return dot(subtract(p, origin), axis); }
    Plane3d plane(double offset) const { return {add(origin, scale(axis, offset)), axis, x}; }
};

Frame makeFrame(Vec3 origin, Vec3 axis) {
    const Vec3 seed = std::abs(axis.x) < .8 ? Vec3{1,0,0} : Vec3{0,1,0};
    const auto x = normalized(cross(seed, axis));
    return {origin, axis, x, cross(axis, x)};
}

double convexArea(BoundaryLoop points) {
    std::sort(points.begin(), points.end(), [](auto a, auto b) {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
    });
    auto turn = [](auto a, auto b, auto c) {
        return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
    };
    BoundaryLoop hull;
    for (const auto& p : points) {
        while (hull.size() >= 2 && turn(hull[hull.size()-2], hull.back(), p) <= 0)
            hull.pop_back();
        hull.push_back(p);
    }
    const auto lower = hull.size();
    for (auto i = points.rbegin()+1; i != points.rend(); ++i) {
        while (hull.size() > lower && turn(hull[hull.size()-2], hull.back(), *i) <= 0)
            hull.pop_back();
        hull.push_back(*i);
    }
    return std::abs(boundaryArea(hull));
}

void addAxis(std::vector<ProjectionAxisReport>& axes, Vec3 axis, const char* source,
             std::size_t limit) {
    if (axes.size() >= limit || !finite(axis) || length(axis) < 1e-8) return;
    axis = normalized(axis);
    if (std::any_of(axes.begin(), axes.end(), [&](const auto& entry) {
        return std::abs(dot(entry.axis, axis)) >= axisAlignment;
    })) return;
    axes.push_back({axis, source, 0, 0, {}});
}

std::vector<PrimitiveEvidence> detect(const PointStore& points, double tolerance) {
    PrimitiveDetectionOptions options;
    options.detectSpheres = options.detectCones = options.detectTori = false;
    options.plane.distanceThreshold = options.cylinder.distanceThreshold = tolerance;
    options.plane.minimumSupportPoints = options.cylinder.minimumSupportPoints =
        std::max<std::size_t>(minimumCapPoints, points.size()/300);
    options.plane.maximumPlanes = 16;
    options.cylinder.maximumCylinders = 6;
    PrimitiveDetectionResult output;
    std::string error;
    detectPrimitiveEvidence(points, options, output, error);
    return std::move(output.evidence);
}

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

struct Cap {
    double depth;
    std::vector<PointSample> points;
};

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

std::optional<Cap> capAt(const std::vector<PointSample>& points, const Frame& frame,
                         double boundary, double tolerance) {
    Cap cap{0, {}};
    std::vector<double> depths;
    for (const auto& p : points) {
        const auto depth = frame.depth(p.position);
        if (std::abs(depth-boundary) > 3*tolerance ||
            (p.normal && std::abs(dot(*p.normal, frame.axis)) < .9)) continue;
        cap.points.push_back(p);
        depths.push_back(depth);
    }
    if (depths.size() < minimumCapPoints) return {};
    const auto median = depths.begin()+depths.size()/2;
    std::nth_element(depths.begin(), median, depths.end());
    cap.depth = *median;
    return cap;
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

std::vector<SketchEntity> profile(const Cap& cap, const Frame& frame, double tolerance,
                                  const std::vector<PrimitiveEvidence>& evidence,
                                  const Cap& first, const Cap& last,
                                  const std::vector<PointSample>& points) {
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

double volume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

struct Fit {
    ProjectionFitQuality quality;
    std::vector<PointId> residuals;
    double volume = 0;
};

std::optional<Fit> evaluate(const ModelingCore& model, const std::vector<PointSample>& observations,
                            double tolerance, std::size_t operations, std::size_t curves,
                            const std::function<bool()>& cancelled) {
    const auto& body = model.bodyShape();
    TopoDS_Compound faces;
    BRep_Builder builder;
    builder.MakeCompound(faces);
    NCollection_IndexedMap<TopoDS_Shape,TopTools_ShapeMapHasher> faceIndex;
    TopExp::MapShapes(body, TopAbs_FACE, faceIndex);
    for (int i = 1; i <= faceIndex.Extent(); ++i) builder.Add(faces, faceIndex(i));
    std::vector<std::size_t> support(faceIndex.Extent()+1);
    Fit fit;
    fit.volume = volume(body);
    std::vector<double> distances;
    double loss = 0, normalError = 0;
    BRepExtrema_DistShapeShape distance;
    distance.LoadS2(faces);
    for (const auto& p : observations) {
        if (cancelled && distances.size()%64 == 0 && cancelled()) return {};
        const auto vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(p.position.x,p.position.y,p.position.z)).Vertex();
        distance.LoadS1(vertex);
        distance.Perform();
        if (!distance.IsDone() || !distance.NbSolution()) return {};
        const double error = distance.Value();
        distances.push_back(error);
        loss += std::min(error/tolerance, 8.0);
        if (error > tolerance) fit.residuals.push_back(p.id);
        if (distance.SupportTypeShape2(1) != BRepExtrema_IsInFace) continue;
        const auto face = TopoDS::Face(distance.SupportOnShape2(1));
        const auto faceId = faceIndex.FindIndex(face);
        if (!p.normal) {
            if (error <= tolerance) ++support[faceId];
            continue;
        }
        double u, v;
        distance.ParOnFaceS2(1, u, v);
        gp_Pnt center;
        gp_Vec du, dv;
        BRepAdaptor_Surface(face).D1(u, v, center, du, dv);
        const auto crossProduct = du.Crossed(dv);
        if (crossProduct.SquareMagnitude() < 1e-24) continue;
        const auto normal = crossProduct.Normalized();
        double alignment = dot(*p.normal, {normal.X(),normal.Y(),normal.Z()});
        if (face.Orientation() == TopAbs_REVERSED) alignment = -alignment;
        if (!p.normalOrientationKnown) alignment = std::abs(alignment);
        alignment = std::clamp(alignment, -1.0, 1.0);
        if (error <= tolerance && alignment >= .8) ++support[faceId];
        normalError += std::acos(alignment)*180/pi;
        loss += .5*(1-alignment);
        ++fit.quality.normalComparisons;
    }
    std::sort(distances.begin(), distances.end());
    const auto count = distances.size();
    fit.quality.scoredPoints = count;
    fit.quality.medianDistance = distances[count/2];
    fit.quality.p95Distance = distances[std::size_t(.95*(count-1))];
    fit.quality.surfaceInlierFraction = 1-double(fit.residuals.size())/count;
    if (fit.quality.normalComparisons)
        fit.quality.meanNormalErrorDegrees = normalError/fit.quality.normalComparisons;
    double area = 0, unsupported = 0;
    for (int i = 1; i <= faceIndex.Extent(); ++i) {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(faceIndex(i),properties);
        area += properties.Mass();
        if (support[i] < 3) unsupported += properties.Mass();
    }
    fit.quality.unsupportedFaceAreaFraction = area > 0 ? unsupported/area : 1;
    fit.quality.score = loss/count + fit.quality.unsupportedFaceAreaFraction +
        .015*operations + .001*curves;
    return fit;
}

struct Program {
    std::vector<ProjectionOperation> operations;
    Fit fit;
};

std::optional<Program> score(std::vector<ProjectionOperation> operations,
    const std::vector<PointSample>& observations, double tolerance,
    const std::function<bool()>& cancelled) {
    try {
        ModelingCore model;
        std::string error;
        if (!buildProjectionProgram(operations, model, error)) return {};
        const auto curves = std::accumulate(operations.begin(), operations.end(), std::size_t(0),
            [](auto sum, const auto& op) { return sum+op.sketch.entities.size(); });
        auto fit = evaluate(model, observations, tolerance, operations.size(), curves, cancelled);
        if (fit) return Program{std::move(operations), std::move(*fit)};
    } catch (const Standard_Failure&) {
        // Invalid candidate geometry should leave the remaining candidates testable.
    }
    return {};
}

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

std::optional<ProjectionOperation> slotCut(const Cap& floor, const Cap& opening,
    const Frame& frame, const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance) {
    auto entities = profile(floor,frame,tolerance,evidence,floor,floor,points);
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

bool buildProjectionProgram(const std::vector<ProjectionOperation>& operations,
                            ModelingCore& model, std::string& error) {
    ModelingCore staged;
    if (operations.empty() || operations.front().operation != ExtrudeOperation::NewBody) {
        error = "Projection program requires a base extrusion";
        return false;
    }
    FeatureId target = kInvalidFeatureId;
    for (const auto& operation : operations) {
        const auto sketch = staged.execute(AddFeatureCommand{operation.sketch,true});
        if (!sketch.success) { error = sketch.error; return false; }
        const auto added = staged.execute(AddFeatureCommand{
            ExtrudeFeatureParams{sketch.featureId,operation.depth,operation.reverse,operation.operation,target},true});
        if (!added.success) { error = added.error; return false; }
        target = added.featureId;
        if (!staged.rebuild()) { error = staged.lastError(); return false; }
        const auto& shape = staged.bodyShape();
        TopExp_Explorer solids(shape, TopAbs_SOLID);
        if (!solids.More() || !BRepCheck_Analyzer(shape).IsValid() || volume(shape) <= 0) {
            error = "Projection operation did not produce a valid solid";
            return false;
        }
        solids.Next();
        if (solids.More()) {
            error = "Projection operation produced multiple solids";
            return false;
        }
    }
    model.adoptPrepared(std::move(staged));
    error.clear();
    return true;
}

bool fitProjectionExtrusion(const PointStore& source, const ProjectionExtrusionOptions& options,
    ProjectionExtrusionResult& result, const std::vector<PrimitiveEvidence>& supplied,
    const std::function<bool()>& cancelled) {
    result = {};
    if (source.unit() != LengthUnit::Millimeter || source.size() < 2*minimumCapPoints ||
        options.maximumFitPoints < 2*minimumCapPoints || options.maximumScorePoints == 0 ||
        options.maximumAxes == 0 || options.maximumOperations == 0 || options.distanceTolerance < 0 ||
        !std::isfinite(options.distanceTolerance)) {
        result.reason = "Expected one metric component and valid projection fit options";
        return false;
    }
    auto stopped = [&] {
        if (!cancelled || !cancelled()) return false;
        result.reason = "Cancelled";
        return true;
    };
    if (stopped()) return false;
    std::unordered_set<PointId> ids;
    Eigen::Vector3d low = vector(source.points().front().position), high = low;
    for (const auto& p : source.points()) {
        if (!finite(p.position) || !p.id || !ids.insert(p.id).second) {
            result.reason = "Invalid position or duplicate point ID";
            return false;
        }
        low = low.cwiseMin(vector(p.position));
        high = high.cwiseMax(vector(p.position));
    }
    const double span = (high-low).norm();
    if (span <= 0 || !std::isfinite(span)) {
        result.reason = "Unresolved component extent";
        return false;
    }
    const double tolerance = options.distanceTolerance > 0 ? options.distanceTolerance : span*.0025;
    result.distanceTolerance = tolerance;
    auto points = sample(source.points(), options.maximumFitPoints);
    for (auto& p : points) {
        if (p.normal && finite(*p.normal) && length(*p.normal) > 1e-8)
            p.normal = normalized(*p.normal);
        else p.normal.reset();
    }
    if (std::any_of(points.begin(), points.end(), [](const auto& p) { return !p.normal; })) {
        PointCloudPreprocessingOptions normals;
        PointCloudPreprocessingReport report;
        detail::preprocessing::estimateNormalsAndCurvature(points,normals,report);
    }
    const auto evenlySpaced = detail::preprocessing::voxelDownsample(
        points, span/(2*std::sqrt(double(options.maximumScorePoints))), {});
    const auto observations = sample(evenlySpaced, options.maximumScorePoints);
    auto evidence = supplied;
    auto detected = detect(PointStore(points), tolerance);
    evidence.insert(evidence.end(),std::make_move_iterator(detected.begin()),std::make_move_iterator(detected.end()));
    if (stopped()) return false;
    const auto center = (high+low)*.5;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : points) mean += vector(p.position);
    mean /= double(points.size());
    for (const auto& p : points) {
        const Eigen::Vector3d delta = vector(p.position)-mean;
        covariance += delta*delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> pca(covariance);
    std::vector<Vec3> normals;
    for (const auto& primitive : evidence) {
        if (const auto* cylinder = std::get_if<CylinderEvidence>(&primitive))
            addAxis(result.axes, cylinder->cylinder.axisDirection, "cylinder_axis", options.maximumAxes);
        if (const auto* plane = std::get_if<PlaneEvidence>(&primitive)) {
            const auto n = plane->plane.normal;
            if (std::none_of(normals.begin(), normals.end(), [&](auto other) {
                return std::abs(dot(n,other)) >= axisAlignment;
            })) normals.push_back(n);
        }
    }
    for (const auto n : normals) addAxis(result.axes,n,"plane_normal",options.maximumAxes);
    for (int i = 2; i >= 0; --i)
        addAxis(result.axes,point(pca.eigenvectors().col(i)),"coordinate_pca",options.maximumAxes);
    for (std::size_t i = 0; i < normals.size(); ++i)
        for (std::size_t j = i+1; j < normals.size(); ++j)
            addAxis(result.axes,cross(normals[i],normals[j]),"side_plane_pair",options.maximumAxes);
    const auto seeds = result.axes;
    for (const auto& seed : seeds) {
        Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
        for (const auto& p : observations) {
            if (!p.normal || std::abs(dot(*p.normal,seed.axis)) > .2) continue;
            const auto n = vector(*p.normal);
            moment += n*n.transpose();
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> fit(moment);
        const auto eigenvalues = fit.eigenvalues();
        if (eigenvalues[1]-eigenvalues[0] > .03*eigenvalues[2])
            addAxis(result.axes,point(fit.eigenvectors().col(0)),"side_normal_moment",options.maximumAxes);
    }
    if (options.includeAreaDirections) {
        // A small deterministic screen; the metric is the area of the convexified projection.
        const auto areaPoints = sample(points, 2000);
        std::vector<std::pair<double, Vec3>> ranked;
        for (int i = 0; i < 64; ++i) {
            if (stopped()) return false;
            const double z = (i+.5)/64;
            const double r = std::sqrt(1-z*z), angle = i*pi*(3-std::sqrt(5.));
            const Vec3 direction{r*std::cos(angle), r*std::sin(angle), z};
            const auto frame = makeFrame(point(center), direction);
            BoundaryLoop projected;
            for (const auto& p : areaPoints) projected.push_back(frame.project(p.position));
            ranked.emplace_back(convexArea(std::move(projected)),direction);
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        const auto limit = std::min(options.maximumAxes,result.axes.size()+3);
        for (const auto& item : ranked) addAxis(result.axes,item.second,"small_convex_projection",limit);
    }

    std::vector<Program> programs;
    auto operations = options.maximumOperations > 1 ? blindHoles(points,evidence,tolerance) :
        std::vector<ProjectionOperation>{};
    for (auto& report : result.axes) {
        if (stopped()) return false;
        const auto frame = makeFrame(point(center), report.axis);
        BoundaryLoop projected;
        std::vector<double> depths;
        for (const auto& p : points) {
            projected.push_back(frame.project(p.position));
            depths.push_back(frame.depth(p.position));
        }
        std::sort(depths.begin(), depths.end());
        const auto margin = depths.size()/500;
        const double start = depths[margin], end = depths[depths.size()-1-margin];
        report.convexProjectionArea = convexArea(std::move(projected));
        report.convexEnvelopeVolume = report.convexProjectionArea*(end-start);
        report.status = "missing_end_face_support";
        const auto first = capAt(points,frame,start,tolerance);
        const auto last = capAt(points,frame,end,tolerance);
        if (!first || !last || last->depth-first->depth < 3*tolerance) continue;
        report.status = "invalid_profile_or_solid";
        auto propose = [&](const Cap& section, const Cap& lowCap, const Cap& highCap,
                           bool addJoin) {
            ProjectionOperation base;
            base.sketch.plane = frame.plane(lowCap.depth);
            base.sketch.entities = profile(section,frame,tolerance,evidence,lowCap,highCap,points);
            if (base.sketch.entities.empty()) return;
            base.depth = highCap.depth-lowCap.depth;
            for (const auto& p : section.points) base.supportPointIds.push_back(p.id);
            if (auto program = score({base}, observations, tolerance, cancelled)) {
                report.status = "evaluated";
                programs.push_back(std::move(*program));
                if (addJoin) {
                    base.operation = ExtrudeOperation::Join;
                    operations.push_back(std::move(base));
                }
            }
        };
        propose(*first,*first,*last,false);
        propose(*last,*first,*last,false);
        if (options.maximumOperations == 1) continue;
        std::vector<double> interior;
        for (const auto& primitive : evidence) {
            if (stopped()) return false;
            const auto* plane = std::get_if<PlaneEvidence>(&primitive);
            if (!plane) continue;
            const double alignment = dot(plane->plane.normal, frame.axis);
            if (std::abs(alignment) < axisAlignment) continue;
            const double depth = -(dot(plane->plane.normal,frame.origin)+plane->plane.offset)/alignment;
            if (depth-first->depth < 5*tolerance || last->depth-depth < 5*tolerance ||
                std::any_of(interior.begin(),interior.end(),[&](double t) {
                    return std::abs(t-depth)<4*tolerance;
                })) continue;
            const auto middle = capAt(points,frame,depth,tolerance);
            if (!middle) continue;
            interior.push_back(depth);
            propose(*first,*first,*middle,true);
            propose(*last,*middle,*last,true);
            for (const auto* opening : {&*first,&*last})
                if (auto cut = slotCut(*middle,*opening,frame,points,evidence,tolerance))
                    operations.push_back(std::move(*cut));
            if (interior.size()==2) break;
        }
    }
    if (stopped()) return false;
    if (programs.empty()) {
        result.reason = "No axis produced a valid profile with two supported end faces";
        return false;
    }
    auto rank = [](auto& states) {
        std::stable_sort(states.begin(), states.end(), [](const auto& a, const auto& b) {
            return a.fit.quality.score < b.fit.quality.score;
        });
        if (states.size() > 4) states.resize(4);
    };
    rank(programs);
    for (std::size_t step = 1; step < options.maximumOperations && !operations.empty(); ++step) {
        auto next = programs;
        bool improved = false;
        for (const auto& program : programs) for (const auto& operation : operations) {
            if (stopped()) return false;
            auto extended = program.operations;
            if (extended.size() != step) continue;
            extended.push_back(operation);
            auto candidate = score(std::move(extended), observations, tolerance, cancelled);
            if (!candidate) continue;
            const double volumeChange = candidate->fit.volume-program.fit.volume;
            const double signedChange = operation.operation == ExtrudeOperation::Cut ? -volumeChange : volumeChange;
            if (signedChange > std::pow(tolerance,3) &&
                candidate->fit.quality.score+.01 < program.fit.quality.score &&
                candidate->fit.quality.surfaceInlierFraction >= program.fit.quality.surfaceInlierFraction) {
                next.push_back(std::move(*candidate));
                improved = true;
            }
        }
        if (!improved) break;
        rank(next);
        programs = std::move(next);
    }
    if (stopped()) return false;
    auto acceptable = [&](const Program& program) {
        return program.fit.quality.surfaceInlierFraction >= .9 &&
            program.fit.quality.p95Distance <= 3*tolerance &&
            program.fit.quality.unsupportedFaceAreaFraction <= .04;
    };
    const auto firstAccepted = std::find_if(programs.begin(),programs.end(),acceptable);
    auto& best = firstAccepted == programs.end() ? programs.front() : *firstAccepted;
    // Check shorter programs after the full geometry is available.
    for (std::size_t i = best.operations.size(); i-- > 1;) {
        auto shorter = best.operations;
        shorter.erase(shorter.begin()+i);
        if (auto pruned = score(std::move(shorter),observations,tolerance,cancelled);
            pruned && acceptable(*pruned) && pruned->fit.quality.score <= best.fit.quality.score &&
            pruned->fit.quality.surfaceInlierFraction >= best.fit.quality.surfaceInlierFraction)
            best = std::move(*pruned);
    }
    if (stopped()) return false;
    result.quality = best.fit.quality;
    result.residualPointIds = std::move(best.fit.residuals);
    if (!acceptable(best)) {
        result.reason = "Best valid program did not explain enough observed surface";
        return false;
    }
    result.operations = std::move(best.operations);
    return true;
}

} // namespace reconstruction
