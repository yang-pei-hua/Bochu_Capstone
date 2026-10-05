#pragma once

#include "modeling/ModelingCore.h"
#include "reconstruction/ReconstructionCandidate.h"
#include "reconstruction/CompoundRecognizer.h"

namespace reconstruction {

modeling::ModelPatchResult commitCompoundCandidate(
    const CompoundCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision,
    bool replacePrevious = false);

modeling::ModelPatchResult commitBoxCandidate(
    const BoxCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

modeling::ModelPatchResult commitBoxWithThroughHolesCandidate(
    const BoxWithThroughHolesCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

}  // namespace reconstruction
