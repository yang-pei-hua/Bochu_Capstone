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
    // Elevation stops just short of the poles. At exactly +/-90 degrees the up
    // vector degenerates and CameraController::apply() cannot derive a stable
    // orientation, so the limit is enforced both here and by the input widgets.
    // The Top/Bottom standard views still provide true pole views.
    static constexpr double kMaxElevation = 89.9;

    static OrbitParameters fromCamera(const CameraParameters& parameters);
    static CameraParameters toCamera(const OrbitParameters& orbit,
                                     const CameraParameters& base,
                                     bool lookAtOrigin);
};