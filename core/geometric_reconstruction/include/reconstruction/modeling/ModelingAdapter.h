#pragma once

#include "modeling/ModelingCore.h"
#include "reconstruction/data/ReconstructionCandidate.h"
#include "reconstruction/recognition/CompoundRecognizer.h"

namespace reconstruction {

modeling::ModelPatchResult commitCompoundCandidate(
    const CompoundCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision,
    bool replacePrevious = false, bool overwriteManual = false);

modeling::ModelPatchResult commitBoxCandidate(
    const BoxCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

modeling::ModelPatchResult commitBoxWithThroughHolesCandidate(
    const BoxWithThroughHolesCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

}  // namespace reconstruction
