#pragma once

#include "core/CameraController.h"

// Spherical description of where a camera sits relative to what it looks at.
// Angles are how a photographer reasons about camera placement; the raw
// CameraParameters remain the only thing the viewer ever consumes.
struct OrbitParameters
{
    double distance = 5.0;
    double azimuthDeg = 0.0;    // rotation about +Z, measured from +X
    double elevationDeg = 0.0;  // pitch above the XY plane, positive upwards
};

class OrbitCamera
{
public:
    // Exact pole views are supported. toCamera() assigns them an explicit
    // horizontal up vector instead of using the spherical derivative, which
    // degenerates at +/-90 degrees.
    static constexpr double kMaxElevation = 90.0;

    static OrbitParameters fromCamera(const CameraParameters& parameters);
    static CameraParameters toCamera(const OrbitParameters& orbit,
                                     const CameraParameters& base,
                                     bool lookAtOrigin);
};
