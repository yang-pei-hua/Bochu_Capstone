#pragma once

#include "modeling/ModelingCore.h"
#include "reconstruction/ReconstructionCandidate.h"

namespace reconstruction {

modeling::ModelPatchResult commitBoxCandidate(
    const BoxCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

}  // namespace reconstruction
