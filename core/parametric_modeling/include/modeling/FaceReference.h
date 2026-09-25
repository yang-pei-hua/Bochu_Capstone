#pragma once

#include "modeling/Geometry.h"
#include "modeling/Id.h"

#include <variant>

namespace modeling {

enum class DatumPlane {
    XY,
    YZ,
    XZ,
};

enum class FaceRole {
    StartFace,
    EndFace,
    Unknown,
};

struct FaceReference {
    FeatureId ownerFeature = kInvalidFeatureId;
    FaceRole role = FaceRole::Unknown;

    // Geometric hints are persisted for future fallback matching. V0 resolves
    // StartFace/EndFace semantically and reports ReferenceLost otherwise.
    Vec3 normal{};
    Vec3 centroid{};
    double area = 0.0;
    int transientFaceIndexHint = -1;
};

struct DatumPlaneReference {
    DatumPlane plane = DatumPlane::XY;
};

using SketchPlaneReference = std::variant<DatumPlaneReference, FaceReference>;

inline SketchPlaneReference datumPlane(DatumPlane plane) {
    return DatumPlaneReference{plane};
}

inline SketchPlaneReference featureFace(FeatureId owner, FaceRole role) {
    FaceReference reference;
    reference.ownerFeature = owner;
    reference.role = role;
    return reference;
}

}  // namespace modeling
