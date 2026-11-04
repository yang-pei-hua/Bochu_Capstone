#pragma once

#include "reconstruction/recognition/CompoundRecognizer.h"

namespace reconstruction::detail::repair {
struct RepairEvidence {
    PointStore observations;
    std::vector<PrimitiveEvidence> evidence;
    std::vector<SurfaceIdMapping> mappings;
};

RepairEvidence collectRepairEvidence(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const std::vector<LocalAnalysisFailure>& failures);

// Returns a rejection reason, or an empty string after checking stability.
std::string validateRepairCandidate(const PointStore& points, const SurfaceAnalysisOptions& options,
    const CompoundCandidate& baseline, const CompoundCandidate& checked,
    const std::vector<LocalAnalysisFailure>& failures, RepairTransactionReport& status);
} // namespace reconstruction::detail::repair
