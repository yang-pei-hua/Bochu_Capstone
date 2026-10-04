#pragma once

#include "app/CaptureController.h"

// Interactive and validation captures use the same 62 camera poses.
inline std::vector<OrbitRing> sphericalCaptureRings()
{
    return {{12, -60.0}, {12, -30.0}, {12, 0.0}, {12, 30.0},
            {12, 60.0}, {1, 90.0}, {1, -90.0}};
}
