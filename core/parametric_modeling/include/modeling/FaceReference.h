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
    StartFace = 0,
    EndFace = 1,
    Unknown = 2,
    BoxXMin = 3,
    BoxXMax = 4,
    BoxYMin = 5,
    BoxYMax = 6,
    BoxZMin = 7,
    BoxZMax = 8,
    CylinderBase = 9,
    CylinderTop = 10,
};

struct FaceReference {
    FeatureId ownerFeature = kInvalidFeatureId;
    FaceRole role = FaceRole::Unknown;

    // Semantic roles are resolved from their owner's parameters, independently
    // of transient face indices or boolean-result topology. Old enum values
    // remain stable for project compatibility.
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
