#pragma once
#include "reconstruction/recognition/CompoundRecognizer.h"
namespace reconstruction {
// Bounded mixed-feature recognizer: an observed thin rectangular host, attached
// planar structures in independent local frames, vertical cylindrical/conical
// bosses, and round/rectangular through cuts. No template labels are consumed.
bool recognizeMixedFeatures(const PointStore& points, const SurfaceAnalysisResult& analysis,
                            CompoundCandidate& output, std::string& error,
                            std::vector<LocalAnalysisFailure>* failures = nullptr,
                            const PointStore* source = nullptr);
} // namespace reconstruction
