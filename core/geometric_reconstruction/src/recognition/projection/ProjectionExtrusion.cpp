#include "reconstruction/recognition/ProjectionExtrusion.h"

#include "ProjectionInternal.h"
#include "reconstruction/recognition/ProjectionSettings.h"
#include "modeling/ModelingCore.h"
#include "preprocessing/detail/KdTree.h"
#include "recognition/assembly/BoundaryGeometry.h"
#include "reconstruction/detection/PrimitiveDetector.h"

#include <BRepCheck_Analyzer.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <tuple>
#include <unordered_set>

namespace reconstruction {
namespace {
using namespace math3;
using namespace modeling;
using namespace detail::projection;
using detail::assembly::BoundaryLoop;
using detail::assembly::boundaryArea;

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

BoundaryLoop convexHull(BoundaryLoop points) {
    if (points.size() < 3) return {};
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
    hull.pop_back(); // The reverse pass repeats the first vertex.
    return hull;
}

double convexArea(BoundaryLoop points) {
    return std::abs(boundaryArea(convexHull(std::move(points))));
}

bool hasAxis(const std::vector<ProjectionAxisReport>& axes, Vec3 axis, double alignment) {
    return std::any_of(axes.begin(), axes.end(), [&](const auto& entry) {
        return std::abs(dot(entry.axis, axis)) >= alignment;
    });
}

void addAxis(std::vector<ProjectionAxisReport>& axes, Vec3 axis, const char* source,
             std::size_t limit, double alignment) {
    if (axes.size() >= limit || !finite(axis) || length(axis) < 1e-8) return;
    axis = normalized(axis);
    if (!hasAxis(axes, axis, alignment)) axes.push_back({axis, source});
}

std::vector<PrimitiveEvidence> detect(const PointStore& points, double tolerance, const ProjectionExtrusionOptions& tuning) {
    PrimitiveDetectionOptions options;
    options.detectSpheres = options.detectCones = options.detectTori = false;
    options.plane.distanceThreshold = options.cylinder.distanceThreshold = tolerance;
    options.plane.minimumSupportPoints = options.cylinder.minimumSupportPoints =
        std::max<std::size_t>(tuning.minimumCapPoints, points.size()/tuning.evidenceSupportDivisor);
    options.plane.maximumPlanes = tuning.maximumEvidencePlanes;
    options.cylinder.maximumCylinders = tuning.maximumEvidenceCylinders;
    PrimitiveDetectionResult output;
    std::string error;
    detectPrimitiveEvidence(points, options, output, error);
    return std::move(output.evidence);
}

} // namespace

namespace detail::projection {
std::optional<Fit> trialProgram(const std::vector<ProjectionOperation>& operations,
    const std::vector<PointSample>& observations, double tolerance,
    const std::function<bool()>& cancelled, const ProjectionExtrusionOptions& options) {
    try {
        ModelingCore model;
        std::string error;
        if (!buildProjectionProgram(operations, model, error)) return {};
        const auto curves = std::accumulate(operations.begin(), operations.end(), std::size_t(0),
            [](auto sum, const auto& op) { return sum+op.sketch.entities.size(); });
        return scoreShape(model.bodyShape(), observations, tolerance, operations.size(), curves, cancelled, options);
    } catch (const Standard_Failure&) {
        // Invalid candidate geometry should leave the remaining candidates testable.
    }
    return {};
}

bool prepareProjection(const PointStore& source, const ProjectionExtrusionOptions& options,
    const std::vector<PrimitiveEvidence>& supplied, ProjectionWork& work, std::string& error,
    const std::function<bool()>& cancelled) {
    work = {};
    error.clear();
    try { validateProjectionOptions(options); }
    catch (const std::invalid_argument& failure) { error = failure.what(); return false; }
    if (source.unit() != LengthUnit::Millimeter || source.size() < 2*options.minimumCapPoints) {
        error = "Expected one metric component and valid projection fit options";
        return false;
    }
    if (cancelled && cancelled()) {
        error = "Cancelled";
        return false;
    }
    std::unordered_set<PointId> ids;
    Eigen::Vector3d low = vector(source.points().front().position), high = low;
    for (const auto& p : source.points()) {
        if (!finite(p.position) || !p.id || !ids.insert(p.id).second) {
            error = "Invalid position or duplicate point ID";
            return false;
        }
        low = low.cwiseMin(vector(p.position));
        high = high.cwiseMax(vector(p.position));
    }
    const double span = (high-low).norm();
    if (span <= 0 || !std::isfinite(span)) {
        error = "Unresolved component extent";
        return false;
    }
    const double tolerance = options.distanceTolerance > 0 ? options.distanceTolerance : span*options.relativeDistanceTolerance;
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
        points, span/(options.scoreVoxelScale*std::sqrt(double(options.maximumScorePoints))), {});
    auto observations = sample(evenlySpaced, options.maximumScorePoints);
    auto evidence = supplied;
    auto detected = detect(PointStore(points), tolerance, options);
    evidence.insert(evidence.end(),std::make_move_iterator(detected.begin()),std::make_move_iterator(detected.end()));
    work = {std::move(points), std::move(observations), std::move(evidence), point((high+low)*.5), tolerance};
    return true;
}

Frame makeFrame(Vec3 origin, Vec3 axis) {
    const Vec3 seed = std::abs(axis.x) < .8 ? Vec3{1,0,0} : Vec3{0,1,0};
    const auto x = normalized(cross(seed, axis));
    return {origin, axis, x, cross(axis, x)};
}

std::optional<Cap> capAt(const std::vector<PointSample>& points, const Frame& frame,
                         double boundary, double tolerance, const ProjectionExtrusionOptions& options) {
    Cap cap{0, {}};
    std::vector<double> depths;
    for (const auto& p : points) {
        const auto depth = frame.depth(p.position);
        if (std::abs(depth-boundary) > options.capBandMultiplier*tolerance ||
            (p.normal && std::abs(dot(*p.normal, frame.axis)) < options.capNormalCosine)) continue;
        cap.points.push_back(p);
        depths.push_back(depth);
    }
    if (depths.size() < options.minimumCapPoints) return {};
    const auto median = depths.begin()+depths.size()/2;
    std::nth_element(depths.begin(), median, depths.end());
    cap.depth = *median;
    return cap;
}

Interval inspectInterval(const std::vector<PointSample>& points, const Frame& frame, double tolerance,
                         const ProjectionExtrusionOptions& options) {
    if (points.empty()) return {};
    BoundaryLoop projected;
    std::vector<double> depths;
    for (const auto& p : points) {
        projected.push_back(frame.project(p.position));
        depths.push_back(frame.depth(p.position));
    }
    std::sort(depths.begin(), depths.end());
    const auto margin = std::size_t(depths.size()*options.extentTrimFraction);
    const double start = depths[margin], end = depths[depths.size()-1-margin];
    return {start, end, convexArea(std::move(projected)),
        capAt(points, frame, start, tolerance, options), capAt(points, frame, end, tolerance, options)};
}

bool generateAxes(const std::vector<PointSample>& points,
    const std::vector<PointSample>& directionObservations, const std::vector<PrimitiveEvidence>& evidence,
    Vec3 center, const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    const std::function<bool()>& cancelled, std::array<Vec3, 3>* principalAxes) {
    axes.clear();
    if (points.empty()) return true;
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
            addAxis(axes, cylinder->cylinder.axisDirection, "cylinder_axis", options.maximumAxes, options.axisAlignmentCosine);
        if (const auto* plane = std::get_if<PlaneEvidence>(&primitive)) {
            const auto n = plane->plane.normal;
            if (std::none_of(normals.begin(), normals.end(), [&](auto other) {
                return std::abs(dot(n,other)) >= options.axisAlignmentCosine;
            })) normals.push_back(n);
        }
    }
    for (const auto n : normals) addAxis(axes,n,"plane_normal",options.maximumAxes, options.axisAlignmentCosine);
    const std::array pcaAxes{point(pca.eigenvectors().col(2)), point(pca.eigenvectors().col(1)),
                             point(pca.eigenvectors().col(0))};
    if (principalAxes) *principalAxes = pcaAxes;
    const auto pcaEnd = pcaAxes.begin() + std::min(options.maximumAxes, pcaAxes.size());
    // Keep evidence priority and reuse a slot when it already covers a PCA direction.
    while (axes.size() + std::count_if(pcaAxes.begin(), pcaEnd, [&](Vec3 axis) {
        return !hasAxis(axes, axis, options.axisAlignmentCosine);
    }) > options.maximumAxes) axes.pop_back();
    for (auto axis = pcaAxes.begin(); axis != pcaEnd; ++axis)
        addAxis(axes, *axis, "coordinate_pca", options.maximumAxes, options.axisAlignmentCosine);
    for (std::size_t i = 0; i < normals.size(); ++i)
        for (std::size_t j = i+1; j < normals.size(); ++j)
            addAxis(axes,cross(normals[i],normals[j]),"side_plane_pair",options.maximumAxes, options.axisAlignmentCosine);
    const auto seeds = axes;
    for (const auto& seed : seeds) {
        Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
        for (const auto& p : directionObservations) {
            if (!p.normal || std::abs(dot(*p.normal,seed.axis)) > options.sideMomentNormalCosine) continue;
            const auto n = vector(*p.normal);
            moment += n*n.transpose();
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> fit(moment);
        const auto eigenvalues = fit.eigenvalues();
        if (eigenvalues[1]-eigenvalues[0] > options.sideMomentEigenGap*eigenvalues[2])
            addAxis(axes,point(fit.eigenvectors().col(0)),"side_normal_moment",options.maximumAxes, options.axisAlignmentCosine);
    }
    if (options.includeAreaDirections) {
        // A small deterministic screen; the metric is the area of the convexified projection.
        const auto areaPoints = sample(points, options.areaSamplePoints);
        std::vector<std::pair<double, Vec3>> ranked;
        for (std::size_t i = 0; i < options.areaSampleDirections; ++i) {
            if (cancelled && cancelled()) return false;
            const double z = (i+.5)/options.areaSampleDirections;
            const double r = std::sqrt(1-z*z), angle = i*pi*(3-std::sqrt(5.));
            const Vec3 direction{r*std::cos(angle), r*std::sin(angle), z};
            const auto frame = makeFrame(center, direction);
            BoundaryLoop projected;
            for (const auto& p : areaPoints) projected.push_back(frame.project(p.position));
            ranked.emplace_back(convexArea(std::move(projected)),direction);
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        const auto limit = std::min(options.maximumAxes,axes.size()+options.maximumAreaDirections);
        for (const auto& item : ranked) addAxis(axes,item.second,"small_convex_projection",limit,options.axisAlignmentCosine);
    }

    return true;
}
} // namespace detail::projection

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

namespace {
bool prepareProjectionAxes(const PointStore& source, const ProjectionExtrusionOptions& options,
    ProjectionExtrusionResult& result, const std::vector<PrimitiveEvidence>& supplied,
    ProjectionWork& work, std::array<Vec3, 3>& principalAxes, const std::function<bool()>& cancelled) {
    result = {};
    if (!prepareProjection(source, options, supplied, work, result.reason, cancelled)) return false;
    result.distanceTolerance = work.tolerance;
    if ((cancelled && cancelled()) || !generateAxes(work.points, work.scoringPoints, work.evidence,
            work.center, options, result.axes, cancelled, &principalAxes)) {
        result.reason = "Cancelled";
        return false;
    }
    return true;
}
} // namespace

bool inspectProjectionSketches(const PointStore& source, const ProjectionExtrusionOptions& options,
    ProjectionExtrusionResult& result, const std::function<bool()>& cancelled) {
    ProjectionWork work;
    std::array<Vec3, 3> principalAxes{};
    if (!prepareProjectionAxes(source,options,result,{},work,principalAxes,cancelled)) return false;
    collectSketchInputs(work,principalAxes,options,result.axes,result.sketchInputs,cancelled);
    if (cancelled && cancelled()) {
        result.sketchInputs.clear();
        result.reason = "Cancelled";
        return false;
    }
    if (!result.sketchInputs.empty()) return true;
    result.reason = "No supported projection samples for this path";
    return false;
}

bool fitProjectionExtrusion(const PointStore& source, const ProjectionExtrusionOptions& options,
    ProjectionExtrusionResult& result, const std::vector<PrimitiveEvidence>& supplied,
    const std::function<bool()>& cancelled) {
    ProjectionWork work;
    std::array<Vec3, 3> principalAxes{};
    if (!prepareProjectionAxes(source,options,result,supplied,work,principalAxes,cancelled)) return false;
    const auto& observations = work.scoringPoints;
    const auto tolerance = work.tolerance;
    if (options.collectSketchInputs)
        collectSketchInputs(work,principalAxes,options,result.axes,result.sketchInputs,cancelled);

    const ProgramEvaluator evaluate = [&](const std::vector<ProjectionOperation>& operations) {
        if (cancelled && cancelled()) return std::optional<Fit>{};
        return trialProgram(operations, observations, tolerance, cancelled, options);
    };
    auto search = searchProjectionPaths(work, principalAxes, options, result.axes, evaluate, cancelled);
    if (search.status == SearchStatus::Cancelled) {
        result.reason = "Cancelled";
        return false;
    }
    if (search.status == SearchStatus::NoValidBase) {
        result.reason = "No projection path produced a valid extrusion program";
        return false;
    }
    result.quality = search.best.fit.quality;
    result.residualPointIds = std::move(search.best.fit.residuals);
    const auto& base = search.best.operations.front();
    if (const auto* plane = std::get_if<Plane3d>(&base.sketch.plane)) {
        const auto axis = std::find_if(result.axes.begin(), result.axes.end(), [&](const auto& report) {
            return dot(report.axis, plane->normal) >= options.axisAlignmentCosine;
        });
        if (axis != result.axes.end()) result.bestAxisIndex = std::size_t(axis - result.axes.begin());
    }
    if (search.status == SearchStatus::InsufficientSupport) {
        result.reason = "Best valid program did not explain enough observed surface";
        if (options.retainRejectedProgram) result.rejectedOperations = std::move(search.best.operations);
        return false;
    }
    result.operations = std::move(search.best.operations);
    return true;
}

bool inspectProjectionSketches(const PointStore& source, ProjectionExtrusionResult& result,
    const std::function<bool()>& cancelled) {
    try {
        auto options = loadProjectionOptions();
        options.path = ProjectionPath::SingleAxis;
        return inspectProjectionSketches(source, options, result, cancelled);
    } catch (const std::exception& error) {
        result = {};
        result.reason = error.what();
        return false;
    }
}

bool fitProjectionExtrusion(const PointStore& source, ProjectionExtrusionResult& result,
    const std::vector<PrimitiveEvidence>& supplied, const std::function<bool()>& cancelled) {
    try {
        return fitProjectionExtrusion(source, loadProjectionOptions(), result, supplied, cancelled);
    } catch (const std::exception& error) {
        result = {};
        result.reason = error.what();
        return false;
    }
}

} // namespace reconstruction
