#pragma once

#include "modeling/ModelCommand.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace modeling {

using ModelRevision = std::uint64_t;
inline constexpr ModelRevision kAnyModelRevision =
    std::numeric_limits<ModelRevision>::max();

// A patch is the mutation boundary used by reconstruction and other automated
// producers. All commands and the optional rebuild commit atomically.
struct ModelPatch {
    ModelRevision expectedRevision = kAnyModelRevision;
    std::vector<ModelCommand> commands;
    bool rebuild = true;
};

struct ModelPatchResult {
    bool success = false;
    ModelRevision revision = 0;
    std::vector<FeatureId> affectedFeatures;
    std::string error;
};

}  // namespace modeling
