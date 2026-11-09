#pragma once

#include <array>

#include "common/Math3.h"
#include "reconstruction/recognition/ProjectionExtrusion.h"

class TopoDS_Shape;

namespace reconstruction::detail::projection {
inline constexpr double pi = 3.141592653589793;

using modeling::Vec3;
using modeling::Point2D;
using modeling::Plane3d;
using math3::add;
using math3::dot;
using math3::scale;
using math3::subtract;

struct Frame {
    Vec3 origin, axis, x, y;
    Point2D project(Vec3 p) const {
        const auto delta = subtract(p, origin);
        return {dot(delta, x), dot(delta, y)};
    }
    double depth(Vec3 p) const { return dot(subtract(p, origin), axis); }
    Plane3d plane(double offset) const { return {add(origin, scale(axis, offset)), axis, x}; }
};

// Axes are unit vectors in the observation frame.
Frame makeFrame(Vec3 origin, Vec3 axis);

struct ProjectionWork {
    std::vector<PointSample> points;
    std::vector<PointSample> scoringPoints;
    std::vector<PrimitiveEvidence> evidence;
    Vec3 center{};
    double tolerance = 0;
};

bool prepareProjection(const PointStore& source, const ProjectionExtrusionOptions& options,
    const std::vector<PrimitiveEvidence>& supplied, ProjectionWork& work, std::string& error,
    const std::function<bool()>& cancelled = {});

bool generateAxes(const std::vector<PointSample>& points,
    const std::vector<PointSample>& directionObservations,
    const std::vector<PrimitiveEvidence>& evidence, Vec3 center,
    const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    const std::function<bool()>& cancelled, std::array<Vec3, 3>* principalAxes = nullptr);

struct Cap {
    double depth;
    std::vector<PointSample> points;
};

struct Interval {
    double start = 0;
    double end = 0;
    double convexProjectionArea = 0;
    std::optional<Cap> first;
    std::optional<Cap> last;
};

Interval inspectInterval(const std::vector<PointSample>& points, const Frame& frame, double tolerance,
    const ProjectionExtrusionOptions& options = {});
std::optional<Cap> capAt(const std::vector<PointSample>& points, const Frame& frame,
    double boundary, double tolerance, const ProjectionExtrusionOptions& options = {});
double capSpacing(const Cap& cap, const ProjectionExtrusionOptions& options = {});
ProjectionSketchInput projectCap(const Cap& cap, const Frame& frame, double spacing, double tolerance,
    const ProjectionExtrusionOptions& options = {});
ProjectionSketchInput projectEnvelope(const std::vector<PointSample>& points,
    const Frame& frame, double start, double end, double tolerance, const ProjectionExtrusionOptions& options = {});
void collectSketchInputs(const ProjectionWork& work, const std::array<Vec3, 3>& principalAxes,
    const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    std::vector<ProjectionSketchInput>& inputs,
    const std::function<bool()>& cancelled);
std::optional<ProjectionOperation> projectionEnvelope(const std::vector<PointSample>& points,
    const Frame& frame, double start, double end, double tolerance, const ProjectionExtrusionOptions& options = {});
std::vector<modeling::SketchEntity> recoverProfile(const Cap& cap, const Frame& frame, double tolerance,
    const std::vector<PrimitiveEvidence>& evidence, const Cap& first, const Cap& last,
    const std::vector<PointSample>& points, const ProjectionExtrusionOptions& options = {});

using EndProfiles = std::array<std::vector<modeling::SketchEntity>, 2>;
using OperationSequence = std::vector<ProjectionOperation>;

struct AxisProposal {
    ProjectionOperation operation;
    bool allowJoin = false; // Reuse a validated standalone base as an additive step.
};
std::vector<ProjectionOperation> blindHoles(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance, const ProjectionExtrusionOptions& options = {});
std::vector<ProjectionOperation> throughHoles(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, double tolerance,
    const std::function<bool()>& cancelled, const ProjectionExtrusionOptions& options = {});
bool proposeAxisOperations(const std::vector<PointSample>& points,
    const std::vector<PrimitiveEvidence>& evidence, const Frame& frame,
    const Cap& first, const Cap& last, double tolerance, bool includeSecondary,
    std::vector<AxisProposal>& proposals, const std::function<bool()>& cancelled,
    const EndProfiles* endProfiles = nullptr, std::vector<OperationSequence>* segmented = nullptr,
    const ProjectionExtrusionOptions& options = {});

struct Fit {
    ProjectionFitQuality quality;
    std::vector<PointId> residuals;
    double volume = 0;
};

double volume(const TopoDS_Shape& shape);
// All candidates are measured against the same explicit scoring observations.
std::optional<Fit> scoreShape(const TopoDS_Shape& body, const std::vector<PointSample>& observations,
    double tolerance, std::size_t operations, std::size_t curves, const std::function<bool()>& cancelled = {},
    const ProjectionExtrusionOptions& options = {});

std::optional<Fit> trialProgram(const std::vector<ProjectionOperation>& operations,
    const std::vector<PointSample>& observations, double tolerance,
    const std::function<bool()>& cancelled = {}, const ProjectionExtrusionOptions& options = {});

struct Program {
    std::vector<ProjectionOperation> operations;
    Fit fit;
};
using ProgramEvaluator = std::function<std::optional<Fit>(const std::vector<ProjectionOperation>&)>;
enum class SearchStatus { Accepted, NoValidBase, InsufficientSupport, Cancelled };
struct SearchResult {
    SearchStatus status;
    Program best;
};
SearchResult searchPrograms(std::vector<Program> programs, const std::vector<ProjectionOperation>& operations,
    std::size_t maximumOperations, double tolerance, const ProgramEvaluator& evaluate,
    const std::function<bool()>& cancelled = {}, bool prune = true, std::size_t minimumOperations = 1,
    const ProjectionExtrusionOptions& options = {});
SearchResult searchProjectionPaths(const ProjectionWork& work, const std::array<Vec3, 3>& principalAxes,
    const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    const ProgramEvaluator& evaluate, const std::function<bool()>& cancelled);

} // namespace reconstruction::detail::projection
