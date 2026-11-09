#include "ProjectionInternal.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace reconstruction::detail::projection {
namespace {
bool acceptable(const Fit& fit, double tolerance, const ProjectionExtrusionOptions& options) {
    return fit.quality.surfaceInlierFraction >= options.minimumInlierFraction &&
        fit.quality.p95Distance <= options.maximumP95Multiplier*tolerance &&
        fit.quality.unsupportedFaceAreaFraction <= options.maximumUnsupportedFaceFraction;
}

struct AxisExtent {
    Frame frame;
    double start;
    double end;
};

std::vector<std::size_t> intersectionDirections(const std::array<Vec3, 3>& principalAxes,
    const std::vector<ProjectionAxisReport>& axes, double alignment) {
    std::vector<std::size_t> directions;
    const auto addDirection = [&](std::size_t index) {
        if (std::find(directions.begin(),directions.end(),index) == directions.end())
            directions.push_back(index);
    };
    for (const auto principal : principalAxes) {
        const auto match = std::find_if(axes.begin(),axes.end(),[&](const auto& axis) {
            return std::abs(dot(principal,axis.axis)) >= alignment;
        });
        if (match != axes.end()) addDirection(std::size_t(match-axes.begin()));
    }
    const auto bounds = std::minmax_element(axes.begin(),axes.end(),[](const auto& a,const auto& b) {
        return a.convexProjectionArea < b.convexProjectionArea;
    });
    addDirection(std::size_t(bounds.first-axes.begin()));
    addDirection(std::size_t(bounds.second-axes.begin()));
    return directions;
}

std::vector<Program> intersectionPrograms(const ProjectionWork& work,
    const std::array<Vec3, 3>& principalAxes, const std::vector<ProjectionAxisReport>& axes,
    const std::vector<AxisExtent>& extents, const ProjectionExtrusionOptions& options, const ProgramEvaluator& evaluate,
    const std::function<bool()>& cancelled) {
    const auto directions = intersectionDirections(principalAxes,axes,options.axisAlignmentCosine);
    std::vector<std::optional<ProjectionOperation>> envelopes;
    for (const auto index : directions) {
        if (cancelled && cancelled()) return {};
        const auto& extent = extents[index];
        envelopes.push_back(projectionEnvelope(work.points,extent.frame,extent.start,extent.end,work.tolerance,options));
    }
    std::vector<Program> programs;
    for (std::size_t i = 0; i < directions.size(); ++i) {
        if (!envelopes[i]) continue;
        for (std::size_t j = i+1; j < directions.size(); ++j) {
            if (cancelled && cancelled()) return {};
            if (!envelopes[j] || std::abs(dot(axes[directions[i]].axis,axes[directions[j]].axis)) > options.intersectionMaximumAxisCosine)
                continue;
            OperationSequence intersection{*envelopes[i],*envelopes[j]};
            intersection.back().operation = modeling::ExtrudeOperation::Intersect;
            if (auto fit = evaluate(intersection))
                programs.push_back({std::move(intersection),std::move(*fit)});
        }
    }
    return programs;
}
} // namespace

void collectSketchInputs(const ProjectionWork& work, const std::array<Vec3, 3>& principalAxes,
    const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    std::vector<ProjectionSketchInput>& inputs,
    const std::function<bool()>& cancelled) {
    const auto path = options.path;
    if (axes.empty() || (path != ProjectionPath::SingleAxis && path != ProjectionPath::TwoAxisIntersection))
        return;
    const bool wholePart = path == ProjectionPath::TwoAxisIntersection;
    std::vector<AxisExtent> extents;
    for (auto& axis : axes) {
        if (cancelled && cancelled()) return;
        const auto frame = makeFrame(work.center,axis.axis);
        const auto interval = inspectInterval(work.points,frame,work.tolerance,options);
        if (wholePart) {
            axis.convexProjectionArea = interval.convexProjectionArea;
            axis.convexEnvelopeVolume = interval.convexProjectionArea*(interval.end-interval.start);
            extents.push_back({frame,interval.start,interval.end});
            continue;
        }
        using Section = ProjectionSketchInput::Section;
        const auto addCap = [&](const std::optional<Cap>& cap, Section section) {
            if (!cap) return;
            auto input = projectCap(*cap,frame,capSpacing(*cap,options),work.tolerance,options);
            input.section = section;
            inputs.push_back(std::move(input));
        };
        addCap(interval.first,Section::FirstCap);
        addCap(interval.last,Section::LastCap);
    }
    if (!wholePart) return;
    for (const auto index : intersectionDirections(principalAxes,axes,options.axisAlignmentCosine)) {
        if (cancelled && cancelled()) return;
        const auto& extent = extents[index];
        auto input = projectEnvelope(work.points,extent.frame,extent.start,extent.end,work.tolerance,
                                     options);
        if (!input.points.empty()) inputs.push_back(std::move(input));
    }
}

SearchResult searchPrograms(std::vector<Program> programs, const std::vector<ProjectionOperation>& operations,
    std::size_t maximumOperations, double tolerance, const ProgramEvaluator& evaluate,
    const std::function<bool()>& cancelled, bool prune, std::size_t minimumOperations, const ProjectionExtrusionOptions& options) {
    if (programs.empty()) return {SearchStatus::NoValidBase, {}};
    const auto rank = [tolerance,minimumOperations,&options](auto& states) {
        std::stable_sort(states.begin(), states.end(), [tolerance,minimumOperations,&options](const auto& a, const auto& b) {
            const bool firstComplete = a.operations.size() >= minimumOperations;
            const bool secondComplete = b.operations.size() >= minimumOperations;
            if (firstComplete != secondComplete) return firstComplete;
            const bool firstAccepted = acceptable(a.fit,tolerance,options);
            const bool secondAccepted = acceptable(b.fit,tolerance,options);
            if (firstAccepted != secondAccepted) return firstAccepted;
            return a.fit.quality.score < b.fit.quality.score;
        });
        if (states.size() > options.searchBeamWidth) states.resize(options.searchBeamWidth);
    };
    rank(programs);
    for (std::size_t step = 1; step < maximumOperations && !operations.empty(); ++step) {
        auto next = programs;
        bool improved = false;
        for (const auto& program : programs) {
            if (program.operations.size() != step) continue;
            for (const auto& operation : operations) {
                if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
                auto extended = program.operations;
                extended.push_back(operation);
                auto candidate = evaluate(extended);
                if (!candidate) continue;
                const double volumeChange = candidate->volume-program.fit.volume;
                const bool removesMaterial = operation.operation == modeling::ExtrudeOperation::Cut ||
                    operation.operation == modeling::ExtrudeOperation::Intersect;
                const double signedChange = removesMaterial ? -volumeChange : volumeChange;
                const bool meetsAcceptance = acceptable(*candidate,tolerance,options) &&
                    !acceptable(program.fit,tolerance,options);
                const bool improvesScore = candidate->quality.score+options.minimumScoreImprovement < program.fit.quality.score &&
                    candidate->quality.surfaceInlierFraction >= program.fit.quality.surfaceInlierFraction;
                const bool completesPath = step < minimumOperations && extended.size() >= minimumOperations;
                if (signedChange > options.minimumBooleanVolumeMultiplier*tolerance*tolerance*tolerance &&
                    (completesPath || meetsAcceptance || improvesScore)) {
                    next.push_back({std::move(extended), std::move(*candidate)});
                    improved = true;
                }
            }
        }
        if (!improved) {
            if (std::none_of(programs.begin(),programs.end(),[&](const auto& program) {
                return program.operations.size() > step;
            })) break;
            continue;
        }
        rank(next);
        programs = std::move(next);
    }
    if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
    auto& best = programs.front();
    if (best.operations.size() < minimumOperations) return {SearchStatus::NoValidBase, {}};
    // Check shorter programs after the full geometry is available.
    for (std::size_t i = best.operations.size(); prune && i-- > 1;) {
        if (best.operations.size() <= minimumOperations) break;
        auto shorter = best.operations;
        shorter.erase(shorter.begin()+i);
        if (auto pruned = evaluate(shorter);
            pruned && acceptable(*pruned,tolerance,options) && pruned->quality.score <= best.fit.quality.score &&
            pruned->quality.surfaceInlierFraction >= best.fit.quality.surfaceInlierFraction)
            best = {std::move(shorter), std::move(*pruned)};
    }
    if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
    const auto status = acceptable(best.fit,tolerance,options) ? SearchStatus::Accepted : SearchStatus::InsufficientSupport;
    return {status, std::move(best)};
}

SearchResult searchProjectionPaths(const ProjectionWork& work, const std::array<Vec3, 3>& principalAxes,
    const ProjectionExtrusionOptions& options, std::vector<ProjectionAxisReport>& axes,
    const ProgramEvaluator& evaluate, const std::function<bool()>& cancelled) {
    if (axes.empty()) return {SearchStatus::NoValidBase,{}};
    const auto& points = work.points;
    const auto& evidence = work.evidence;
    const double tolerance = work.tolerance;
    const bool intersectionOnly = options.path == ProjectionPath::TwoAxisIntersection;
    const bool segmentedOnly = options.path == ProjectionPath::Segmented;
    const bool multiOperation = options.maximumOperations > 1 && options.path != ProjectionPath::SingleAxis;
    if (!multiOperation && (intersectionOnly || segmentedOnly)) return {SearchStatus::NoValidBase,{}};
    std::vector<AxisExtent> extents;
    std::vector<Program> single, partial, layered;
    OperationSequence additions, cuts;
    for (auto& report : axes) {
        if (cancelled && cancelled()) return {SearchStatus::Cancelled,{}};
        const auto frame = makeFrame(work.center,report.axis);
        const auto interval = inspectInterval(points,frame,tolerance,options);
        extents.push_back({frame,interval.start,interval.end});
        report.convexProjectionArea = interval.convexProjectionArea;
        report.convexEnvelopeVolume = report.convexProjectionArea*(interval.end-interval.start);
        if (intersectionOnly) {
            report.status = "projection_extent";
            continue;
        }
        report.status = "missing_end_face_support";
        const auto& first = interval.first;
        const auto& last = interval.last;
        if (!first || !last || last->depth-first->depth < options.minimumDepthMultiplier*tolerance) continue;
        report.status = "invalid_profile";
        std::vector<AxisProposal> proposals;
        std::vector<OperationSequence> segments;
        if (!proposeAxisOperations(points,evidence,frame,*first,*last,tolerance,
                multiOperation,proposals,cancelled,nullptr,&segments,options))
            return {SearchStatus::Cancelled,{}};
        for (auto& proposal : proposals) {
            auto& operation = proposal.operation;
            if (operation.operation == modeling::ExtrudeOperation::Cut) {
                cuts.push_back(std::move(operation));
                continue;
            }
            if (!report.bestBaseQuality) report.status = "invalid_solid";
            if (auto fit = evaluate({operation})) {
                report.status = "evaluated";
                if (!report.bestBaseQuality || fit->quality.score < report.bestBaseQuality->score)
                    report.bestBaseQuality = fit->quality;
                if (proposal.allowJoin) {
                    partial.push_back({{operation},std::move(*fit)});
                    operation.operation = modeling::ExtrudeOperation::Join;
                    additions.push_back(std::move(operation));
                } else {
                    single.push_back({{std::move(operation)},std::move(*fit)});
                }
            }
        }
        for (auto& segment : segments) {
            if (cancelled && cancelled()) return {SearchStatus::Cancelled,{}};
            if (auto fit = evaluate(segment)) layered.push_back({std::move(segment),std::move(*fit)});
        }
    }
    if (cancelled && cancelled()) return {SearchStatus::Cancelled,{}};
    if (!multiOperation) return searchPrograms(std::move(single),{},1,tolerance,evaluate,cancelled,true,1,options);
    if (intersectionOnly) {
        auto intersections = intersectionPrograms(work,principalAxes,axes,extents,options,
                                                  evaluate,cancelled);
        if (cancelled && cancelled()) return {SearchStatus::Cancelled,{}};
        return searchPrograms(std::move(intersections),{},2,tolerance,evaluate,cancelled,false,1,options);
    }

    auto blind = blindHoles(points,evidence,tolerance,options);
    auto through = throughHoles(points,evidence,tolerance,cancelled,options);
    cuts.insert(cuts.end(),std::make_move_iterator(blind.begin()),std::make_move_iterator(blind.end()));
    cuts.insert(cuts.end(),std::make_move_iterator(through.begin()),std::make_move_iterator(through.end()));
    additions.insert(additions.end(),cuts.begin(),cuts.end());
    if (!additions.empty()) partial.insert(partial.end(),single.begin(),single.end());
    const auto minimumOperations = segmentedOnly ? 2u : 1u;
    auto best = segmentedOnly ? SearchResult{SearchStatus::NoValidBase,{}}
                              : searchPrograms(std::move(single),{},1,tolerance,evaluate,cancelled,true,1,options);
    const auto compare = [&](SearchResult candidate) {
        if (candidate.status == SearchStatus::Cancelled || best.status == SearchStatus::NoValidBase ||
            (candidate.status == SearchStatus::Accepted && best.status != SearchStatus::Accepted) ||
            (candidate.status == best.status && candidate.best.fit.quality.score < best.best.fit.quality.score))
            best = std::move(candidate);
    };
    if (best.status == SearchStatus::Cancelled) return best;
    if (!segmentedOnly) {
        auto intersections = intersectionPrograms(work,principalAxes,axes,extents,options,
                                                  evaluate,cancelled);
        if (cancelled && cancelled()) return {SearchStatus::Cancelled,{}};
        // Intersection is a complete baseline. Keep its two operations together.
        compare(searchPrograms(std::move(intersections),{},2,tolerance,evaluate,cancelled,false,1,options));
    }
    if (best.status == SearchStatus::Cancelled) return best;
    compare(searchPrograms(std::move(layered),cuts,options.maximumOperations,tolerance,evaluate,cancelled,true,minimumOperations,options));
    if (best.status == SearchStatus::Cancelled) return best;
    compare(searchPrograms(std::move(partial),additions,options.maximumOperations,tolerance,evaluate,cancelled,true,minimumOperations,options));
    return best;
}
} // namespace reconstruction::detail::projection
