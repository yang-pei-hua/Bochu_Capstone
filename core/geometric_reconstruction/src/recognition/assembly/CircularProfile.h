#pragma once

#include <modeling/ProfileBody.h>

namespace reconstruction::detail::assembly {

struct ProfileHalfPlane {
    modeling::Vec3 normal;
    double offset;
};

std::vector<modeling::ProfileCurve2D> circularOutline(double radius,
    const std::vector<ProfileHalfPlane>& cuts, double tolerance);
double profileWallCoordinate(const modeling::ProfileCurve2D& curve, modeling::Vec3 point,
                             modeling::Vec3 outwardNormal, double tolerance);

}
