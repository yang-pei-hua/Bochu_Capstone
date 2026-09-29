#include "core/OrbitCamera.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double left, double right, double tolerance = 1.0e-12)
{
    return std::abs(left - right) <= tolerance;
}

void checkPole(double elevation, double expectedZ, double expectedUpY)
{
    CameraParameters base;
    const CameraParameters camera =
        OrbitCamera::toCamera({10.0, 270.0, elevation}, base, true);
    require(near(camera.position[0], 0.0) && near(camera.position[1], 0.0),
            "pole X/Y must be pinned to the target");
    require(near(camera.position[2], expectedZ),
            "pole Z must equal the signed orbit radius");
    require(near(camera.target[0], 0.0) && near(camera.target[1], 0.0)
                && near(camera.target[2], 0.0),
            "pole camera must look at the origin");
    require(near(camera.up[0], 0.0) && near(camera.up[1], expectedUpY)
                && near(camera.up[2], 0.0),
            "pole camera must have a stable horizontal up vector");

    const double view[3]{camera.target[0] - camera.position[0],
                         camera.target[1] - camera.position[1],
                         camera.target[2] - camera.position[2]};
    const double dot = view[0] * camera.up[0] + view[1] * camera.up[1]
                       + view[2] * camera.up[2];
    require(near(dot, 0.0), "pole up vector must be perpendicular to view direction");
}

} // namespace

int main()
{
    try {
        checkPole(90.0, 10.0, 1.0);
        checkPole(-90.0, -10.0, -1.0);
        std::cout << "Orbit camera pole test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Orbit camera pole test failed: " << exception.what() << '\n';
        return 1;
    }
}
