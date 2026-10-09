#pragma once

#include "modeling/Feature.h"
#include <optional>
#include <string>
#include <vector>

namespace modeling {

const char* faceRoleName(FaceRole role);

// Resolve a primitive's semantic support plane in model coordinates. These
// planes describe the source feature, including after a downstream boolean.
bool primitiveFacePlane(const FeatureParams& params, FaceRole role,
                        Plane3d& plane, std::string& error);

// Shared by rebuilding, display and producers of structured commands.
bool resolveSketchPlane(const std::vector<Feature>& features,
                        const SketchPlaneReference& reference,
                        Plane3d& plane, std::string& error);

// Only attach a picked planar patch when exactly one supported primitive face
// contains the model-space pick. Ambiguous coplanar owners are never guessed.
std::optional<FaceReference> identifyPrimitiveFace(
    const std::vector<Feature>& features, const Plane3d& pickedPlane,
    Vec3 pickedPoint, std::string& error, double tolerance = 1e-6);

} // namespace modeling
