#pragma once

#include "modeling/Geometry.h"
#include <string>
#include <variant>

namespace modeling {

// A world-space right-handed frame. The modeling core normalizes the axes and
// rejects degenerate or non-orthogonal input before creating derived geometry.
struct Pose3d {
    Vec3 origin{};
    Vec3 xDirection{1.0, 0.0, 0.0};
    Vec3 zDirection{0.0, 0.0, 1.0};
};

// A semantic solid primitive. Unlike an Extrude this describes what the solid
// is, not a guessed authoring operation that may have produced it.
struct BoxPrimitiveParams {
    Pose3d pose{};
    double sizeX = 0.0;
    double sizeY = 0.0;
    double sizeZ = 0.0;
    // Fuse with the existing body when reconstructing connected primitives.
    bool join = false;
};

// A cylindrical material removal that crosses the complete current body. The
// axis is expressed in world space; unlike a blind Cut it deliberately has no
// depth because the modeling backend derives a safe span from the body bounds.
struct ThroughHolePrimitiveParams {
    Vec3 axisPoint{};
    Vec3 axisDirection{0.0, 0.0, 1.0};
    double radius = 0.0;
};

struct SpherePrimitiveParams { Vec3 center{}; double radius = 0.0; };
struct CylinderPrimitiveParams {
    Vec3 baseCenter{};
    Vec3 axisDirection{0.0, 0.0, 1.0};
    double radius = 0.0;
    double height = 0.0;
};
// Radii belong to the start and end planes along axisDirection. Either may be
// zero for a pointed cone; both positive describe a conical frustum.
struct ConePrimitiveParams {
    Vec3 baseCenter{};
    Vec3 axisDirection{0.0, 0.0, 1.0};
    double baseRadius = 0.0;
    double topRadius = 0.0;
    double height = 0.0;
};
// First version supports complete non-self-intersecting ring tori (R > r > 0).
struct TorusPrimitiveParams {
    Vec3 center{};
    Vec3 axisDirection{0.0, 0.0, 1.0};
    double majorRadius = 0.0;
    double minorRadius = 0.0;
};
using CurvedPrimitiveParams = std::variant<SpherePrimitiveParams, CylinderPrimitiveParams,
                                          ConePrimitiveParams, TorusPrimitiveParams>;
bool validateCurvedPrimitiveParams(const CurvedPrimitiveParams& params, std::string& error);

}  // namespace modeling
