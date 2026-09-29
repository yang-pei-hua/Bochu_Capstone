#include "core/OrbitCamera.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kMinDistance = 1.0e-6;

double clampElevation(double degrees)
{
    return std::clamp(degrees, -OrbitCamera::kMaxElevation, OrbitCamera::kMaxElevation);
}

double normalizedAzimuth(double degrees)
{
    double result = std::fmod(degrees, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}
}

OrbitParameters OrbitCamera::fromCamera(const CameraParameters& parameters)
{
    OrbitParameters orbit;

    const double dx = parameters.position[0] - parameters.target[0];
    const double dy = parameters.position[1] - parameters.target[1];
    const double dz = parameters.position[2] - parameters.target[2];

    const double horizontal = std::sqrt(dx * dx + dy * dy);
    const double distance = std::sqrt(horizontal * horizontal + dz * dz);
    if (distance < kMinDistance) {
        // The camera sits on its own target, so there is no direction to report.
        return orbit;
    }

    orbit.distance = distance;
    orbit.azimuthDeg = normalizedAzimuth(std::atan2(dy, dx) * kRadToDeg);
    orbit.elevationDeg = clampElevation(std::asin(dz / distance) * kRadToDeg);
    return orbit;
}

CameraParameters OrbitCamera::toCamera(const OrbitParameters& orbit,
                                       const CameraParameters& base,
                                       bool lookAtOrigin)
{
    CameraParameters result = base;

    const double distance = orbit.distance > kMinDistance ? orbit.distance : kMinDistance;
    const double azimuth = orbit.azimuthDeg * kDegToRad;
    const double elevation = clampElevation(orbit.elevationDeg) * kDegToRad;

    const double cosElevation = std::cos(elevation);
    const double sinElevation = std::sin(elevation);
    const double cosAzimuth = std::cos(azimuth);
    const double sinAzimuth = std::sin(azimuth);

    // Looking at the origin is the default: the target is pinned to (0, 0, 0) and
    // the position is derived purely from the orbit. Otherwise the caller keeps
    // whatever target it had and the camera simply orbits around it.
    if (lookAtOrigin) {
        result.target[0] = 0.0;
        result.target[1] = 0.0;
        result.target[2] = 0.0;
    }

    const bool atPole = std::abs(cosElevation) < 1.0e-12;
    if (atPole) {
        // All azimuths describe the same point at a pole. Pin X/Y exactly to the
        // target so the aggregate camera file contains one canonical pose.
        result.position[0] = result.target[0];
        result.position[1] = result.target[1];
        result.position[2] = result.target[2] + std::copysign(distance, sinElevation);
    } else {
        result.position[0] = result.target[0] + distance * cosElevation * cosAzimuth;
        result.position[1] = result.target[1] + distance * cosElevation * sinAzimuth;
        result.position[2] = result.target[2] + distance * sinElevation;
    }

    if (lookAtOrigin) {
        if (atPole) {
            // Keep +X pointing right in both pole images. This mirrors the
            // standard Top/Bottom camera convention and stays perpendicular to
            // the viewing direction.
            result.up[0] = 0.0;
            result.up[1] = sinElevation > 0.0 ? 1.0 : -1.0;
            result.up[2] = 0.0;
        } else {
            // The spherical derivative keeps the horizon level away from the
            // poles.
            result.up[0] = -sinElevation * cosAzimuth;
            result.up[1] = -sinElevation * sinAzimuth;
            result.up[2] = cosElevation;
        }
    }

    return result;
}
