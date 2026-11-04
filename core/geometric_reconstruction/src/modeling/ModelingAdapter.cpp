#include "reconstruction/modeling/ModelingAdapter.h"

namespace reconstruction {

std::optional<modeling::ModelPatch> planCompoundCandidate(
    const CompoundCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error, modeling::ModelRevision expectedRevision,
    bool replacePrevious, bool overwriteManual) {
    modeling::ModelPatch patch;
    patch.expectedRevision = snapshot.revision();
    error.clear();
    auto reject = [&](const std::string& reason) -> std::optional<modeling::ModelPatch> {
        error = reason;
        return std::nullopt;
    };
    if (expectedRevision != modeling::kAnyModelRevision && expectedRevision != snapshot.revision()) {
        return reject("Model revision mismatch: expected " + std::to_string(expectedRevision) +
            ", current " + std::to_string(snapshot.revision()));
    }
    // Allocate sketch IDs on a private copy; the final patch applies atomically.
    auto staging = snapshot;
    if (replacePrevious) {
        for (const auto& feature : snapshot.features()) {
            if (feature.reconstructed && feature.manuallyEdited && !overwriteManual)
                return reject("Manually edited CAD features are protected; explicit replacement is required");
        }
        // Remove only this workflow's features, in dependency order. Authored
        // dependents make replacement fail atomically rather than being deleted.
        const auto features = snapshot.features();
        for (auto i = features.rbegin(); i != features.rend(); ++i) if (i->reconstructed) {
            patch.commands.push_back(modeling::RemoveFeatureCommand{i->id, false});
            const auto removed = staging.execute(patch.commands.back());
            if (!removed.success) return reject(removed.error);
        }
    }
    auto append = [&](const modeling::FeatureParams &params) {
        patch.commands.push_back(modeling::AddFeatureCommand{params, true});
        return staging.execute(patch.commands.back());
    };
    for (const auto &box : candidate.boxes) {
        auto params = box.primitive;
        params.join = true;
        const auto result = append(params);
        if (!result.success)
            return reject(result.error);
    }
    for (const auto &prism : candidate.prisms) {
        modeling::SketchFeatureParams sketch;
        sketch.plane = prism.plane;
        for (std::size_t i = 0; i < prism.polygon.size(); ++i) {
            const auto a = prism.polygon[i], b = prism.polygon[(i + 1) % prism.polygon.size()];
            sketch.entities.push_back({static_cast<modeling::SketchEntityId>(i + 1),
                                       modeling::Line2D{a.x, a.y, b.x, b.y}});
        }
        const auto result = append(sketch);
        if (!result.success)
            return reject(result.error);
        modeling::ExtrudeFeatureParams extrusion;
        extrusion.sketchId = result.featureId;
        extrusion.depth = prism.depth;
        extrusion.operation = prism.subtract ? modeling::ExtrudeOperation::Cut :
            (staging.features().size() > 1 ? modeling::ExtrudeOperation::Join : modeling::ExtrudeOperation::NewBody);
        const auto added = append(extrusion);
        if (!added.success)
            return reject(added.error);
    }
    for (const auto &hole : candidate.throughHoles) {
        const auto added = append(hole.primitive);
        if (!added.success) return reject(added.error);
    }
    for (const auto& solid : candidate.curvedSolids) {
        const auto added = std::visit([&](const auto& params) { return append(params); }, solid.primitive);
        if (!added.success) return reject(added.error);
    }
    for(const auto& body:candidate.profileBodies) {
        const auto added=append(body);
        if(!added.success)return reject(added.error);
    }
    for (const auto& program : candidate.projectionPrograms) {
        if (program.empty() || program.front().operation != modeling::ExtrudeOperation::NewBody)
            return reject("Projection program has no base extrusion");
        modeling::FeatureId target = modeling::kInvalidFeatureId;
        for (const auto& operation : program) {
            const auto sketch = append(operation.sketch);
            if (!sketch.success) return reject(sketch.error);
            const auto extrude = append(modeling::ExtrudeFeatureParams{
                sketch.featureId,operation.depth,operation.reverse,operation.operation,target});
            if (!extrude.success) return reject(extrude.error);
            target = extrude.featureId;
        }
    }
    return patch;
}

modeling::ModelPatchResult commitCompoundCandidate(
    const CompoundCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision, bool replacePrevious, bool overwriteManual) {
    std::string error;
    const auto patch = planCompoundCandidate(candidate, core, error, expectedRevision,
        replacePrevious, overwriteManual);
    if (!patch) return {false, core.revision(), {}, std::move(error)};
    return core.apply(*patch);
}

modeling::ModelPatchResult commitBoxCandidate(const BoxCandidate &candidate,
                                              modeling::ModelingCore &core,
                                              modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.primitive, true});
    patch.rebuild = true;
    return core.apply(patch);
}

modeling::ModelPatchResult
commitBoxWithThroughHolesCandidate(const BoxWithThroughHolesCandidate &candidate,
                                   modeling::ModelingCore &core,
                                   modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.box.primitive, true});
    for (const ThroughHoleCandidate &hole : candidate.throughHoles) {
        patch.commands.push_back(modeling::AddFeatureCommand{hole.primitive, true});
    }
    patch.rebuild = true;
    return core.apply(patch);
}

} // namespace reconstruction
