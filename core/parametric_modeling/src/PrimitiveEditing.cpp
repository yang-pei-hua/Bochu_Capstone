#include "modeling/PrimitiveEditing.h"
#include <cmath>
#include <stdexcept>

namespace modeling {
namespace {
Vec3 normalized(Vec3 v)
{
    const double length = std::hypot(v.x, v.y, v.z);
    if (!std::isfinite(length) || length <= 1e-12) throw std::runtime_error("Invalid primitive axis");
    return {v.x / length, v.y / length, v.z / length};
}
void shift(Vec3& point, Vec3 axis, double distance)
{
    point.x += axis.x * distance; point.y += axis.y * distance; point.z += axis.z * distance;
}
}

std::array<Vec3, 3> boxAxes(const BoxPrimitiveParams& box)
{
    const auto z = normalized(box.pose.zDirection);
    auto x = box.pose.xDirection;
    const double dot = x.x*z.x + x.y*z.y + x.z*z.z;
    x = normalized({x.x-dot*z.x, x.y-dot*z.y, x.z-dot*z.z});
    return {x, Vec3{z.y*x.z-z.z*x.y, z.z*x.x-z.x*x.z, z.x*x.y-z.y*x.x}, z};
}

bool validResizeAnchor(const FeatureParams& params, ResizeAnchor anchor)
{
    if (std::holds_alternative<BoxPrimitiveParams>(params))
        return anchor >= ResizeAnchor::Center && anchor <= ResizeAnchor::ZMax;
    if (std::holds_alternative<CylinderPrimitiveParams>(params))
        return anchor == ResizeAnchor::Center || anchor == ResizeAnchor::Base || anchor == ResizeAnchor::Top;
    return anchor == ResizeAnchor::Center;
}

bool adjustedPrimitive(const FeatureParams& original, const std::array<double, 3>& values,
                       ResizeAnchor anchor, FeatureParams& adjusted, std::string& error)
{
    if (!validResizeAnchor(original, anchor)) { error = "Invalid resize anchor"; return false; }
    const int count = std::holds_alternative<BoxPrimitiveParams>(original) ? 3
        : std::holds_alternative<CylinderPrimitiveParams>(original) ? 2 : 1;
    for (int i = 0; i < count; ++i) if (!std::isfinite(values[i]) || values[i] <= 1e-7) {
        error = "Dimensions must be finite and greater than 0.0000001 mm"; return false;
    }
    try {
        adjusted = original;
        if (auto* box = std::get_if<BoxPrimitiveParams>(&adjusted)) {
            const auto axes = boxAxes(*box);
            const std::array<double,3> old{box->sizeX,box->sizeY,box->sizeZ};
            const int side = static_cast<int>(anchor) - 1;
            for (int i = 0; i < 3; ++i) {
                const double delta = values[i] - old[i];
                if (anchor == ResizeAnchor::Center) shift(box->pose.origin, axes[i], -delta/2);
                else if (i == side/2) {
                    if (side%2 == 1) shift(box->pose.origin, axes[i], -delta);
                } else if (std::abs(delta) > 1e-9) {
                    error = "A fixed face allows changing only its normal dimension"; return false;
                }
            }
            box->sizeX=values[0]; box->sizeY=values[1]; box->sizeZ=values[2];
        } else if (auto* hole = std::get_if<ThroughHolePrimitiveParams>(&adjusted)) {
            hole->radius = values[0]/2;
        } else if (auto* cylinder = std::get_if<CylinderPrimitiveParams>(&adjusted)) {
            const auto axis = normalized(cylinder->axisDirection);
            const double delta = values[1] - cylinder->height;
            shift(cylinder->baseCenter, axis, anchor == ResizeAnchor::Top ? -delta
                : anchor == ResizeAnchor::Center ? -delta/2 : 0);
            cylinder->radius=values[0]; cylinder->height=values[1];
        } else { error = "This feature has no primitive parameter editor"; return false; }
        error.clear();
        return true;
    } catch (const std::exception& failure) { error = failure.what(); return false; }
}
}
