#include "recognition/projection/ProjectionInternal.h"

#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <modeling/ModelingCore.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace reconstruction;
using namespace reconstruction::detail::projection;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void projectionStages() {
    constexpr double tolerance = .05;
    std::vector<PointSample> points;
    for (const double z : {0., 6.}) {
        for (int x = -6; x <= 6; ++x) {
            for (int y = -5; y <= 5; ++y) {
                if (std::abs(x) < 2 && y > -2) continue;
                points.push_back({points.size()+1, {double(x), double(y), z},
                    Vec3{0, 0, z == 0 ? -1. : 1.}});
            }
        }
    }
    PlaneEvidence capEvidence;
    capEvidence.plane = {{0, 0, 1}, 0};
    std::vector<ProjectionAxisReport> axes;
    require(generateAxes(points, points, {capEvidence}, {0, 0, 3}, {}, axes, {}),
        "axis generation completes from fixed observations");
    require(!axes.empty() && std::abs(dot(axes.front().axis, {0, 0, 1})) > .999,
        "plane evidence proposes the known axis, up to sign");

    std::vector<PrimitiveEvidence> crowdedEvidence{capEvidence};
    for (int i = 0; i < 15; ++i) {
        const double angle = .21 * i;
        PlaneEvidence side;
        side.plane = {{.96 * std::cos(angle), .96 * std::sin(angle), .28}, 0};
        crowdedEvidence.emplace_back(side);
    }
    const ProjectionExtrusionOptions options;
    require(generateAxes(points, points, crowdedEvidence, {0, 0, 3}, options, axes, {}),
        "axis generation completes with crowded evidence");
    require(axes.size() == options.maximumAxes, "crowded candidates respect the axis budget");
    for (const auto direction : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}) {
        require(std::count_if(axes.begin(), axes.end(), [&](const auto& axis) {
            return std::abs(dot(axis.axis, direction)) >= options.axisAlignmentCosine;
        }) == 1, "each PCA direction receives exactly one candidate");
    }
    require(axes.front().source == "plane_normal", "overlapping evidence keeps its source and priority");

    const auto frame = makeFrame({0, 0, 3}, {0, 0, 1});
    const auto interval = inspectInterval(points, frame, tolerance);
    require(interval.first && interval.last && interval.first->depth == -3 && interval.last->depth == 3,
        "known axis recovers both supported end sections");
    require(interval.first->points.size() + interval.last->points.size() == points.size(),
        "end observations retain their support points");
    auto missingEnd = points;
    for (auto& p : missingEnd)
        if (p.position.z == 6) p.normal = Vec3{1, 0, 0};
    const auto incomplete = inspectInterval(missingEnd, frame, tolerance);
    require(incomplete.first && !incomplete.last, "side normals do not establish a missing end section");

    // Supply the section directly, independent of both axis and endpoint discovery.
    Cap first{-3, {}}, last{3, {}};
    for (const auto& p : points) (p.position.z == 0 ? first : last).points.push_back(p);
    ProjectionOperation operation;
    operation.sketch.plane = frame.plane(first.depth);
    operation.sketch.entities = recoverProfile(first, frame, tolerance, {}, first, last, points);
    operation.depth = 6;
    modeling::ModelingCore model;
    std::string error;
    const bool built = buildProjectionProgram({operation}, model, error);
    if (!built) throw std::runtime_error("Single-axis extrusion: " + error);
    BRepClass3d_SolidClassifier classifier(model.bodyShape());
    classifier.Perform(gp_Pnt(0, 3, 3), tolerance);
    require(classifier.State() == TopAbs_OUT, "U profile preserves the open notch");
    classifier.Perform(gp_Pnt(4, 3, 3), tolerance);
    require(classifier.State() == TopAbs_IN, "U profile preserves the four-unit wall");
    const auto mass = volume(model.bodyShape());
    require(std::abs(mass / operation.depth - 92) <= 4, "U profile preserves the nominal section area");
}

void intersectionKeepsFittedContours() {
    std::vector<PointSample> points;
    const auto addPoint = [&](Vec3 position, Vec3 normal) {
        points.push_back({points.size()+1,position,normal});
    };
    for (int x = 0; x <= 40; ++x) for (int y = 0; y <= 34; ++y) {
        if ((x > 12 && x < 28 && y > 12) || (x > 16 && x < 24 && y > 3 && y < 9)) continue;
        addPoint({double(x),double(y),0},{0,0,-1});
        addPoint({double(x),double(y),20},{0,0,1});
    }
    for (int z = 0; z <= 20; ++z) for (int y = 0; y <= 34; ++y) {
        addPoint({0,double(y),double(z)},{-1,0,0});
        addPoint({40,double(y),double(z)},{1,0,0});
    }
    for (const double z : {5.,10.,15.}) {
        for (int x = 17; x <= 23; ++x) {
            addPoint({double(x),3,z},{0,1,0});
            addPoint({double(x),9,z},{0,-1,0});
        }
        for (int y = 4; y <= 8; ++y) {
            addPoint({16,double(y),z},{1,0,0});
            addPoint({24,double(y),z},{-1,0,0});
        }
    }
    constexpr double tolerance = .05;
    const auto base = projectionEnvelope(points,makeFrame({0,0,0},{0,0,1}),0,20,tolerance);
    auto transverse = projectionEnvelope(points,makeFrame({0,0,0},{1,0,0}),0,40,tolerance);
    require(base && transverse, "both directions recover an extrudable fitted outline");
    transverse->operation = modeling::ExtrudeOperation::Intersect;
    modeling::ModelingCore model;
    std::string error;
    const bool built = buildProjectionProgram({*base,*transverse},model,error);
    if (!built) throw std::runtime_error("Two-axis intersection: " + error);
    BRepClass3d_SolidClassifier classifier(model.bodyShape());
    classifier.Perform(gp_Pnt(20,24,10),tolerance);
    require(classifier.State() == TopAbs_OUT, "projected intersection preserves the open U notch");
    classifier.Perform(gp_Pnt(20,6,10),tolerance);
    require(classifier.State() == TopAbs_OUT, "projected intersection preserves the supported through hole");
    classifier.Perform(gp_Pnt(6,24,10),tolerance);
    require(classifier.State() == TopAbs_IN, "projected intersection preserves material in the side wall");
}

void proposalAndSearchStages() {
    using modeling::ExtrudeOperation;
    constexpr double tolerance = .05;
    const auto frame = makeFrame({0, 0, 0}, {0, 0, 1});
    std::vector<PointSample> points;
    Cap first{0, {}}, last{6, {}};
    for (int x = -6; x <= 6; ++x) for (int y = -5; y <= 5; ++y) {
        first.points.push_back({points.size()+1, {double(x), double(y), 0}, Vec3{0, 0, -1}});
        points.push_back(first.points.back());
        last.points.push_back({points.size()+1, {double(x), double(y), 6}, Vec3{0, 0, 1}});
        points.push_back(last.points.back());
    }
    for (int x = -2; x <= 2; ++x) for (int y = -3; y <= 3; ++y)
        points.push_back({points.size()+1, {double(x), double(y), 2}, Vec3{0, 0, 1}});
    for (const double x : {-2., 2.}) for (int y = -3; y <= 3; ++y) for (int z = 2; z <= 6; ++z)
        points.push_back({points.size()+1, {x, double(y), double(z)}, Vec3{x / 2, 0, 0}});
    PlaneEvidence floor;
    floor.plane = {{0, 0, 1}, -2};
    std::vector<AxisProposal> proposals;
    require(proposeAxisOperations(points, {floor}, frame, first, last, tolerance, true, proposals, {}),
        "proposals can be generated from fixed end sections");
    require(proposals.size() == 5 && proposals[2].allowJoin && proposals[3].allowJoin &&
        proposals[2].operation.depth == 2 && proposals[3].operation.depth == 4,
        "interior plane yields two step hypotheses in search order");
    const auto& cut = proposals.back().operation;
    require(cut.operation == ExtrudeOperation::Cut && cut.reverse &&
        std::abs(cut.depth - 4.05) < 1e-9 && !cut.supportPointIds.empty(),
        "straight slot uses the observed floor and side walls");

    CylinderEvidence wall;
    wall.cylinder = {{0, 0, 0}, {0, 0, 1}, 2};
    wall.axialMinimum = 2;
    wall.axialMaximum = 6;
    wall.angularCoverage = 1;
    std::vector<PointSample> blindObservations;
    for (int k = 0; k < 12; ++k) {
        const double angle = k * pi / 6;
        blindObservations.push_back({blindObservations.size()+1,
            {std::cos(angle), std::sin(angle), 2}, Vec3{0, 0, 1}});
        blindObservations.push_back({blindObservations.size()+1,
            {3 * std::cos(angle), 3 * std::sin(angle), 6}, Vec3{0, 0, 1}});
    }
    const auto holes = blindHoles(blindObservations, {wall}, tolerance);
    require(holes.size() == 1 && holes.front().operation == ExtrudeOperation::Cut &&
        holes.front().reverse && std::abs(holes.front().depth - 4.05) < 1e-9,
        "known wall, floor and opening produce one blind-hole cut");

    CylinderEvidence thinWall;
    thinWall.cylinder = {{0,0,0},{0,0,1},80};
    thinWall.axialMaximum = 12;
    thinWall.angularCoverage = 1;
    std::vector<PointSample> throughObservations;
    for (int k = 0; k < 24; ++k) {
        const double angle = k*pi/12;
        const double x = std::cos(angle), y = std::sin(angle);
        throughObservations.push_back({throughObservations.size()+1,{80*x,80*y,6},Vec3{-x,-y,0}});
        throughObservations.push_back({throughObservations.size()+1,{90*x,90*y,0},Vec3{0,0,-1}});
        throughObservations.push_back({throughObservations.size()+1,{90*x,90*y,12},Vec3{0,0,1}});
    }
    const auto through = throughHoles(throughObservations,{thinWall},5.57,{});
    require(through.size() == 1 && through.front().operation == ExtrudeOperation::Cut &&
        through.front().depth >= 12 && through.front().depth < 14,
        "a thin through wall uses its local scale even when global tolerance exceeds a third of its depth");
    throughObservations.push_back({throughObservations.size()+1,{0,0,12},Vec3{0,0,1}});
    require(throughHoles(throughObservations,{thinWall},5.57,{}).empty(),
        "observed material at an opening still prevents a through cut");

    Fit base;
    base.volume = 100;
    base.quality.score = .3;
    base.quality.surfaceInlierFraction = .91;
    base.quality.unsupportedFaceAreaFraction = .08;
    Fit improved = base;
    improved.volume = 88;
    improved.quality.score = .2;
    improved.quality.unsupportedFaceAreaFraction = 0;
    const ProgramEvaluator evaluate = [&](const auto& operations) -> std::optional<Fit> {
        return operations.size() == 2 ? improved : base;
    };
    const Program program{{proposals.front().operation}, base};
    const auto accepted = searchPrograms({program}, {cut}, 2, tolerance, evaluate);
    require(accepted.status == SearchStatus::Accepted && accepted.best.operations.size() == 2,
        "fixed trial scores select a cut that removes volume and improves support");
    const auto limited = searchPrograms({program}, {cut}, 1, tolerance, evaluate);
    require(limited.status == SearchStatus::InsufficientSupport && limited.best.operations.size() == 1,
        "search budget preserves the best rejected diagnostic program");
    const ProgramEvaluator scoreWorse = [&](const auto&) -> std::optional<Fit> {
        auto fit = base;
        fit.volume = improved.volume;
        fit.quality.score += .1;
        return fit;
    };
    const auto segmented = searchPrograms({program}, {cut}, 2, tolerance, scoreWorse, {}, true, 2);
    require(segmented.status == SearchStatus::InsufficientSupport && segmented.best.operations.size() == 2,
        "explicit segmented fitting retains a multi-step candidate for diagnostics even if it scores lower");
}

void scoringStages() {
    constexpr double tolerance = .05;
    std::vector<PointSample> observations;
    for (int axis = 0; axis < 3; ++axis) {
        for (const double edge : {0., 4.}) {
            for (int i = 1; i <= 3; ++i) {
                for (int j = 1; j <= 3; ++j) {
                    double position[3]{}, normal[3]{};
                    position[axis] = edge;
                    position[(axis+1)%3] = i;
                    position[(axis+2)%3] = j;
                    normal[axis] = edge == 0 ? -1. : 1.;
                    observations.push_back({observations.size()+1,
                        {position[0], position[1], position[2]},
                        Vec3{normal[0], normal[1], normal[2]}});
                }
            }
        }
    }
    const auto correct = BRepPrimAPI_MakeBox(4, 4, 4).Shape();
    const auto wrong = BRepPrimAPI_MakeBox(3, 4, 4).Shape();
    const auto good = scoreShape(correct, observations, tolerance, 1, 4);
    const auto bad = scoreShape(wrong, observations, tolerance, 1, 4);
    require(good && bad && good->quality.scoredPoints == observations.size() &&
        bad->quality.scoredPoints == observations.size(), "both solids use the fixed scoring observations");
    require(good->residuals.empty() && !bad->residuals.empty() &&
        good->quality.score < bad->quality.score, "fixed scoring prefers the correct geometry");
    require(!scoreShape(correct, observations, tolerance, 1, 4, [] { return true; }),
        "scoring discards cancelled results");
}
}

int main() {
    try {
        projectionStages();
        intersectionKeepsFittedContours();
        scoringStages();
        proposalAndSearchStages();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
