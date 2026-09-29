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

struct OffsetDatumPlane {
    DatumPlane base = DatumPlane::XY;
    double offset = 0.0;
};

// A sketch plane captured directly in world coordinates. It lets a caller sketch
// on an arbitrary planar face of the current body, which no reference above can
// express. Unlike FaceReference it is not re-resolved against the feature
// history, so it is a snapshot: it does not follow the face when an upstream
// feature changes.
struct Plane3d {
    Vec3 origin{};
    Vec3 normal{0.0, 0.0, 1.0};
    Vec3 xDirection{1.0, 0.0, 0.0};
};

using SketchPlaneReference = std::variant<
    DatumPlaneReference,
    OffsetDatumPlane,
    FaceReference,
    Plane3d>;

inline SketchPlaneReference datumPlane(DatumPlane plane) {
    return DatumPlaneReference{plane};
}

inline SketchPlaneReference offsetDatumPlane(DatumPlane plane, double offset) {
    return OffsetDatumPlane{plane, offset};
}

inline SketchPlaneReference featureFace(FeatureId owner, FaceRole role) {
    FaceReference reference;
    reference.ownerFeature = owner;
    reference.role = role;
    return reference;
}

inline SketchPlaneReference explicitPlane(Vec3 origin, Vec3 normal, Vec3 xDirection) {
    return Plane3d{origin, normal, xDirection};
}

}  // namespace modeling
