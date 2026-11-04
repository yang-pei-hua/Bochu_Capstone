#pragma once

#include "modeling/ModelingCore.h"
#include "reconstruction/data/ReconstructionCandidate.h"
#include "reconstruction/recognition/CompoundRecognizer.h"

#include <optional>

namespace reconstruction {

// Resolves candidate dependencies on a staging copy. The resulting patch
// is bound to the snapshot revision and can be inspected before applying it.
std::optional<modeling::ModelPatch> planCompoundCandidate(
    const CompoundCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision,
    bool replacePrevious = false, bool overwriteManual = false);

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
