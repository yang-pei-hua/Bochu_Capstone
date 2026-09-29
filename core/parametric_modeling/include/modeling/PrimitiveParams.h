#pragma once

#include "modeling/Geometry.h"

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
};

// A cylindrical material removal that crosses the complete current body. The
// axis is expressed in world space; unlike a blind Cut it deliberately has no
// depth because the modeling backend derives a safe span from the body bounds.
struct ThroughHolePrimitiveParams {
    Vec3 axisPoint{};
    Vec3 axisDirection{0.0, 0.0, 1.0};
    double radius = 0.0;
};

}  // namespace modeling
