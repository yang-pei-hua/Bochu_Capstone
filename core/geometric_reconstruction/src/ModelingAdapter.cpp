#include "reconstruction/ModelingAdapter.h"

namespace reconstruction {

modeling::ModelPatchResult commitBoxCandidate(
    const BoxCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.primitive});
    patch.rebuild = true;
    return core.apply(patch);
}

}  // namespace reconstruction
