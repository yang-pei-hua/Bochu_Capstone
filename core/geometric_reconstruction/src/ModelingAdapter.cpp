#include "reconstruction/ModelingAdapter.h"

namespace reconstruction {

modeling::ModelPatchResult commitCompoundCandidate(const CompoundCandidate &candidate,
                                                   modeling::ModelingCore &core,
                                                   modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    // Allocate sketch IDs on a private copy; the final patch applies atomically.
    auto staging = core;
    auto append = [&](const modeling::FeatureParams &params) {
        patch.commands.push_back(modeling::AddFeatureCommand{params});
        return staging.execute(patch.commands.back());
    };
    for (const auto &box : candidate.boxes) {
        auto params = box.primitive;
        params.join = true;
        const auto result = append(params);
        if (!result.success)
            return {false, core.revision(), {}, result.error};
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
            return {false, core.revision(), {}, result.error};
        modeling::ExtrudeFeatureParams extrusion;
        extrusion.sketchId = result.featureId;
        extrusion.depth = prism.depth;
        extrusion.operation = staging.features().size() > 1 ? modeling::ExtrudeOperation::Join
                                                            : modeling::ExtrudeOperation::NewBody;
        const auto added = append(extrusion);
        if (!added.success)
            return {false, core.revision(), {}, added.error};
    }
    for (const auto &hole : candidate.throughHoles)
        append(hole.primitive);
    for (const auto& solid : candidate.curvedSolids) {
        const auto added = std::visit([&](const auto& params) { return append(params); }, solid.primitive);
        if (!added.success) return {false, core.revision(), {}, added.error};
    }
    return core.apply(patch);
}

modeling::ModelPatchResult commitBoxCandidate(const BoxCandidate &candidate,
                                              modeling::ModelingCore &core,
                                              modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.primitive});
    patch.rebuild = true;
    return core.apply(patch);
}

modeling::ModelPatchResult
commitBoxWithThroughHolesCandidate(const BoxWithThroughHolesCandidate &candidate,
                                   modeling::ModelingCore &core,
                                   modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.box.primitive});
    for (const ThroughHoleCandidate &hole : candidate.throughHoles) {
        patch.commands.push_back(modeling::AddFeatureCommand{hole.primitive});
    }
    patch.rebuild = true;
    return core.apply(patch);
}

} // namespace reconstruction
