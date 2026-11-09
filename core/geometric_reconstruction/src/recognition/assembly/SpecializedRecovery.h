#pragma once

#include "reconstruction/recognition/AssemblyReconstruction.h"

#include <string_view>
#include <unordered_set>

namespace reconstruction::detail::assembly {

struct SpecializedRecoveryInput {
    // Borrowed for one invocation. Observations and patches share point IDs and a metric frame.
    const PointStore& observations;
    const std::vector<FiniteSurfacePatch>& patches;
    double distanceTolerance;
};

struct SpecializedRecoveryCandidate {
    modeling::ProfileBodyParams body;
    std::unordered_set<PointId> supportPointIds;
    std::string reason;
};

// A registry entry describes one geometry family. The caller owns scheduling,
// validation and publication. Callbacks must not modify shared reconstruction state.
struct SpecializedRecoveryRegistration {
    std::string_view id;
    bool (*applies)(const SpecializedRecoveryInput&);
    std::optional<SpecializedRecoveryCandidate> (*fit)(
        const SpecializedRecoveryInput&, const std::function<bool()>& cancelled);
};

} // namespace reconstruction::detail::assembly
