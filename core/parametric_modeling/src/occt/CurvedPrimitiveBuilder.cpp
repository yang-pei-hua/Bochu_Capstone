#include "occt/Builders.h"
#include "occt/OcctError.h"
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <cmath>
#include <algorithm>
#include <type_traits>

namespace modeling {
bool validateCurvedPrimitiveParams(const CurvedPrimitiveParams &params, std::string &error) {
    const auto finite = [](Vec3 p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
    const bool valid = std::visit(
        [&](const auto &p) {
            using P = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<P, SpherePrimitiveParams>) {
                return finite(p.center) && positive(p.radius);
            } else {
                const auto a = p.axisDirection;
                const double squared = a.x * a.x + a.y * a.y + a.z * a.z;
                if (!finite(a) || !std::isfinite(squared) || !(squared > 1e-24))
                    return false;
                if constexpr (std::is_same_v<P, TorusPrimitiveParams>) {
                    return finite(p.center) && positive(p.minorRadius) && positive(p.majorRadius) &&
                           p.majorRadius > p.minorRadius;
                } else if constexpr (std::is_same_v<P, CylinderPrimitiveParams>) {
                    return finite(p.baseCenter) && positive(p.radius) && positive(p.height);
                } else {
                    return finite(p.baseCenter) && positive(p.height) &&
                           std::isfinite(p.baseRadius) && std::isfinite(p.topRadius) &&
                           p.baseRadius >= 0 && p.topRadius >= 0 &&
                           std::max(p.baseRadius, p.topRadius) > 0 && p.baseRadius != p.topRadius;
                }
            }
        },
        params);
    if (!valid)
        error = "Curved solid requires finite dimensions, a valid axis and positive size; ring "
                "torus requires R > r";
    return valid;
}
namespace occt {
bool buildCurvedPrimitive(const CurvedPrimitiveParams &params, TopoDS_Shape &output,
                          std::string &error) {
    if (!validateCurvedPrimitiveParams(params, error))
        return false;
    try {
        output = std::visit(
            [](const auto &p) -> TopoDS_Shape {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, SpherePrimitiveParams>) {
                    return BRepPrimAPI_MakeSphere(gp_Pnt(p.center.x, p.center.y, p.center.z),
                                                  p.radius)
                        .Shape();
                } else {
                    Vec3 center;
                    if constexpr (std::is_same_v<P, TorusPrimitiveParams>)
                        center = p.center;
                    else
                        center = p.baseCenter;
                    const gp_Ax2 axis(
                        gp_Pnt(center.x, center.y, center.z),
                        gp_Dir(p.axisDirection.x, p.axisDirection.y, p.axisDirection.z));
                    if constexpr (std::is_same_v<P, CylinderPrimitiveParams>)
                        return BRepPrimAPI_MakeCylinder(axis, p.radius, p.height).Shape();
                    else if constexpr (std::is_same_v<P, ConePrimitiveParams>)
                        return BRepPrimAPI_MakeCone(axis, p.baseRadius, p.topRadius, p.height)
                            .Shape();
                    else
                        return BRepPrimAPI_MakeTorus(axis, p.majorRadius, p.minorRadius).Shape();
                }
            },
            params);
        return isValidShape(output, error);
    } catch (const Standard_Failure &failure) {
        error = "OpenCASCADE curved primitive failure: " + failureMessage(failure);
        return false;
    }
}
} // namespace occt
} // namespace modeling
