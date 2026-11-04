#include "ProjectionInternal.h"

#include <algorithm>
#include <cmath>

namespace reconstruction::detail::projection {
SearchResult searchPrograms(std::vector<Program> programs, const std::vector<ProjectionOperation>& operations,
    std::size_t maximumOperations, double tolerance, const ProgramEvaluator& evaluate,
    const std::function<bool()>& cancelled) {
    if (programs.empty()) return {SearchStatus::NoValidBase, {}};
    auto rank = [](auto& states) {
        std::stable_sort(states.begin(), states.end(), [](const auto& a, const auto& b) {
            return a.fit.quality.score < b.fit.quality.score;
        });
        if (states.size() > 4) states.resize(4);
    };
    rank(programs);
    for (std::size_t step = 1; step < maximumOperations && !operations.empty(); ++step) {
        auto next = programs;
        bool improved = false;
        for (const auto& program : programs) for (const auto& operation : operations) {
            if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
            auto extended = program.operations;
            if (extended.size() != step) continue;
            extended.push_back(operation);
            auto candidate = evaluate(extended);
            if (!candidate) continue;
            const double volumeChange = candidate->volume-program.fit.volume;
            const double signedChange = operation.operation == modeling::ExtrudeOperation::Cut ? -volumeChange : volumeChange;
            if (signedChange > std::pow(tolerance,3) &&
                candidate->quality.score+.01 < program.fit.quality.score &&
                candidate->quality.surfaceInlierFraction >= program.fit.quality.surfaceInlierFraction) {
                next.push_back({std::move(extended), std::move(*candidate)});
                improved = true;
            }
        }
        if (!improved) break;
        rank(next);
        programs = std::move(next);
    }
    if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
    auto acceptable = [&](const Fit& fit) {
        return fit.quality.surfaceInlierFraction >= .9 &&
            fit.quality.p95Distance <= 3*tolerance &&
            fit.quality.unsupportedFaceAreaFraction <= .04;
    };
    const auto firstAccepted = std::find_if(programs.begin(), programs.end(), [&](const Program& program) {
        return acceptable(program.fit);
    });
    auto& best = firstAccepted == programs.end() ? programs.front() : *firstAccepted;
    // Check shorter programs after the full geometry is available.
    for (std::size_t i = best.operations.size(); i-- > 1;) {
        auto shorter = best.operations;
        shorter.erase(shorter.begin()+i);
        if (auto pruned = evaluate(shorter);
            pruned && acceptable(*pruned) && pruned->quality.score <= best.fit.quality.score &&
            pruned->quality.surfaceInlierFraction >= best.fit.quality.surfaceInlierFraction)
            best = {std::move(shorter), std::move(*pruned)};
    }
    if (cancelled && cancelled()) return {SearchStatus::Cancelled, {}};
    const auto status = acceptable(best.fit) ? SearchStatus::Accepted : SearchStatus::InsufficientSupport;
    return {status, std::move(best)};
}
} // namespace reconstruction::detail::projection
