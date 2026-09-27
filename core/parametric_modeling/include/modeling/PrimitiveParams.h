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

}  // namespace modeling
