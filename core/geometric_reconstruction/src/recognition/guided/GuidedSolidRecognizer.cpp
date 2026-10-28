#include "detail/GuidedInternal.h"
#include <algorithm>
#include <map>
#include <set>

namespace reconstruction {
using namespace detail::guided;

bool reconstructGuidedSolid(const PointStore& points, const PlaneDetectionOptions& planes,
                            const CylinderDetectionOptions& cylinders,
                            const std::vector<TemplateHint>& hints, GuidedSolidResult& output,
                            std::string& error, const std::function<bool()>& cancelled,
                            const SurfaceAnalysisResult* sharedAnalysis,
                            const std::vector<SurfaceRelationHint>& relationHints, const PointStore* source)
{
    output = {};
    if (hints.empty()) {
        const auto options = recognitionAnalysisOptions(points, planes, cylinders);
        if (sharedAnalysis && analysisInputMatches(points, *sharedAnalysis) &&
            analysisSettingsMatch(options,sharedAnalysis->options)) {
            output.analysis = *sharedAnalysis;
            output.analysisReused = true;
        }
        else if (!analyzeSurfaces(points, options,
                                  output.analysis, error, {}, cancelled)) return false;
        RepairTransactionReport transaction;
        const bool recognized=recognizeAndRepairParametricSolid(points, output.analysis, output.candidate, error, source, &transaction, relationHints);
        if(!recognized)output.analysis.localFailures=std::move(transaction.failures);
        return recognized;
    }
    auto samples = points.points();
    std::map<PointId, std::size_t> indices;
    for (std::size_t i = 0; i < samples.size(); ++i)
        indices[samples[i].id] = i;
    std::set<PointId> assigned;
    std::vector<PrimitiveEvidence> seeds;
    std::vector<CurvedSolidCandidate> closed;
    for (const auto& hint : hints) {
        if (cancelled && cancelled()) {
            error = "Cancelled";
            return false;
        }
        output.failedRegionId = hint.regionId;
        std::vector<PointSample> selected;
        for (auto id : hint.points) {
            if (!assigned.insert(id).second) {
                error = "Overlapping template groups";
                return false;
            }
            const auto found = indices.find(id);
            if (found != indices.end())
                selected.push_back(samples[found->second]);
        }
        if (selected.size() < 20) {
            error = "At least 20 valid retained points are required for a template";
            return false;
        }
        const PointStore subset(selected, points.unit());
        TemplateFit fit;
        fit.regionId = hint.regionId;
        if (hint.kind == TemplateKind::Box) {
            auto settings = planes;
            settings.minimumSupportPoints = 20;
            settings.maximumPlanes = 6;
            BoxCandidate box;
            if (!reconstructBox(subset, settings, {}, box, error)) {
                // A local patch cannot determine all dimensions by itself.
                // Complete it only from actual observations of the whole cloud.
                CompoundCandidate context;
                if (!reconstructParametricSolid(points, planes, cylinders, context, error))
                    return false;
                const auto found =
                    std::find_if(context.boxes.begin(), context.boxes.end(), [&](const auto& c) {
                        return std::count_if(selected.begin(), selected.end(), [&](const auto& p) {
                                   return onBox(c.primitive, p, planes.distanceThreshold * 3);
                               }) >= selected.size() * 0.9;
                    });
                if (found == context.boxes.end()) {
                    error = "No observed box boundary supports this template patch";
                    return false;
                }
                box = *found;
            }
            fit.box = box;
            for (const auto& plane : box.sourceSurfaces)
                seeds.push_back(plane);
        } else {
            SurfaceAnalysisOptions localOptions;
            localOptions.detection = detectionFor(hint.kind, planes.distanceThreshold);
            localOptions.relations.distanceTolerance = planes.distanceThreshold * 4;
            SurfaceAnalysisResult proposals;
            if (!analyzeSurfaces(subset, localOptions, proposals, error, {}, cancelled))
                return false;
            if (!proposals.surfaces.empty()) {
                fit.surface = *std::max_element(
                    proposals.surfaces.begin(), proposals.surfaces.end(),
                    [](const auto& a, const auto& b) { return supportSize(a) < supportSize(b); });
            }
            if (hint.kind != TemplateKind::Plane) {
                CurvedSolidRecognitionOptions settings;
                settings.distanceTolerance = planes.distanceThreshold;
                settings.requiredKind = static_cast<PrimitiveKind>(hint.kind);
                if (fit.surface)
                    settings.seeds.push_back(*fit.surface);
                CurvedSolidCandidate solid;
                std::string incomplete;
                bool complete = recognizeCurvedSolidFromAnalysis(subset, proposals, settings, solid, incomplete);
                if (!complete && fit.surface) {
                    const auto expanded =
                        expandCurvedSupport(*fit.surface, samples, planes.distanceThreshold);
                    if (expanded.size() >= 20) {
                        PointStore local(expanded, points.unit());
                        SurfaceAnalysisResult expandedAnalysis;
                        if (analyzeSurfaces(local, localOptions, expandedAnalysis, incomplete,
                                            {*fit.surface}, cancelled))
                            complete = recognizeCurvedSolidFromAnalysis(local, expandedAnalysis, settings, solid, incomplete);
                    }
                }
                if (complete) {
                    const auto same =
                        std::find_if(closed.begin(), closed.end(), [&](const auto& known) {
                            return std::count_if(
                                       selected.begin(), selected.end(), [&](const auto& p) {
                                           return matchesCurvedBoundary(
                                               known.primitive, p, planes.distanceThreshold * 3);
                                       }) >= selected.size() * 0.9;
                        });
                    if (same != closed.end())
                        solid = *same;
                    else
                        closed.push_back(solid);
                    fit.solid = solid;
                }
            }
            if (!fit.solid && (!fit.surface || supportSize(*fit.surface) < selected.size() * 0.7)) {
                error = "Selected points do not support the requested template; correct membership "
                        "or add observations (region=" + std::to_string(hint.regionId) +
                        ", kind=" + std::to_string(static_cast<int>(hint.kind)) +
                        ", selected=" + std::to_string(selected.size()) + ")";
                return false;
            }
            if (fit.surface)
                seeds.push_back(*fit.surface);
        }
        // The user's planar equation guides normal/frame recovery. Preserve
        // outward orientation; the original document's observations stay intact.
        std::vector<PlaneEvidence> fixed;
        if (fit.box)
            fixed = fit.box->sourceSurfaces;
        if (fit.surface)
            if (const auto* p = std::get_if<PlaneEvidence>(&*fit.surface))
                fixed.push_back(*p);
        for (const auto& p : fixed)
            for (auto id : p.supportPointIds) {
                auto& sample = samples[indices.at(id)];
                auto normal = p.plane.normal;
                if (sample.normal && math3::dot(normal, *sample.normal) < 0)
                    normal = math3::scale(normal, -1);
                sample.normal = normal;
                const double offset = math3::dot(p.plane.normal, sample.position) + p.plane.offset;
                sample.position =
                    math3::subtract(sample.position, math3::scale(p.plane.normal, offset));
            }
        output.fits.push_back(std::move(fit));
    }
    output.failedRegionId = 0;
    if (cancelled && cancelled()) {
        error = "Cancelled";
        return false;
    }
    // A complete manually identified curved component is fixed first. Its
    // remaining observed boundary is consumed, and automatic reconstruction
    // operates on the remaining material rather than refitting that component.
    std::vector<PointSample> remaining;
    for (const auto& p : samples) {
        const bool consumed = std::any_of(closed.begin(), closed.end(), [&](const auto& solid) {
            return matchesCurvedBoundary(solid.primitive, p, planes.distanceThreshold * 3);
        });
        if (!consumed)
            remaining.push_back(p);
    }
    if (!remaining.empty()) {
        std::set<PointId> remainingIds;
        for (const auto& p : remaining)
            remainingIds.insert(p.id);
        seeds.erase(
            std::remove_if(seeds.begin(), seeds.end(),
                           [&](auto& seed) {
                               return std::visit(
                                   [&](auto& evidence) {
                                       auto& ids = evidence.supportPointIds;
                                       ids.erase(std::remove_if(ids.begin(), ids.end(),
                                                                [&](auto id) {
                                                                    return !remainingIds.count(id);
                                                                }),
                                                 ids.end());
                                       return ids.size() < 20;
                                   },
                                   seed);
                           }),
            seeds.end());
        PointStore local(remaining, points.unit());
        if (!analyzeSurfaces(local, recognitionAnalysisOptions(local, planes, cylinders),
                             output.analysis, error, seeds, cancelled) ||
            !recognizeParametricSolidFromAnalysis(local, output.analysis, output.candidate, error)) return false;
    }
    output.candidate.curvedSolids.insert(output.candidate.curvedSolids.end(), closed.begin(),
                                         closed.end());
    if (output.candidate.boxes.empty() && output.candidate.prisms.empty() &&
        output.candidate.curvedSolids.empty()) {
        error = "No supported material";
        return false;
    }
    if (!verifyHints(points, hints, output.candidate, output.fits, planes.distanceThreshold,
                     output.failedRegionId, error, cancelled))
        return false;
    output.candidate.surfaceCoverage =
        (remaining.size() * output.candidate.surfaceCoverage + (points.size() - remaining.size())) /
        points.size();
    output.candidate.confidence = output.candidate.surfaceCoverage;
    if(!relationHints.empty()) {
        // Guided completion may remove a fully observed independent component.
        // Relationship endpoints still refer to the original full snapshot.
        if(!analyzeSurfaces(points,recognitionAnalysisOptions(points,planes,cylinders),output.analysis,error,{},cancelled) ||
           !applySurfaceRelationHints(points,relationHints,output.analysis,error)) return false;
    }
    error.clear();
    return true;
}
} // namespace reconstruction
