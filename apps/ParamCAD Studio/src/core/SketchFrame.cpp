#include "core/SketchFrame.h"
#include <modeling/SemanticFaces.h>

#include <cmath>
#include <variant>

namespace sketchapp {
namespace {

void cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}


}  // namespace

PlaneFrame datumPlaneFrame(modeling::DatumPlane plane)
{
    PlaneFrame frame;
    switch (plane) {
    case modeling::DatumPlane::YZ:
        frame.xDirection[0] = 0.0;
        frame.xDirection[1] = 1.0;
        frame.xDirection[2] = 0.0;
        frame.yDirection[0] = 0.0;
        frame.yDirection[1] = 0.0;
        frame.yDirection[2] = 1.0;
        frame.normal[0] = 1.0;
        frame.normal[1] = 0.0;
        frame.normal[2] = 0.0;
        break;
    case modeling::DatumPlane::XZ:
        frame.xDirection[0] = 1.0;
        frame.xDirection[1] = 0.0;
        frame.xDirection[2] = 0.0;
        frame.yDirection[0] = 0.0;
        frame.yDirection[1] = 0.0;
        frame.yDirection[2] = 1.0;
        frame.normal[0] = 0.0;
        frame.normal[1] = -1.0;
        frame.normal[2] = 0.0;
        break;
    case modeling::DatumPlane::XY:
    default:
        // XY is the identity frame the core's datumPlaneFrame() returns.
        break;
    }
    return frame;
}

PlaneFrame translatedFrame(const PlaneFrame& source, const double normal[3],
                           double distance)
{
    PlaneFrame frame;
    for (int axis = 0; axis < 3; ++axis) {
        frame.origin[axis] = source.origin[axis] + normal[axis] * distance;
        frame.xDirection[axis] = source.xDirection[axis];
        frame.normal[axis] = normal[axis];
    }
    cross(normal, source.xDirection, frame.yDirection);
    return frame;
}

modeling::Plane3d plane3dOf(const PlaneFrame& frame)
{
    modeling::Plane3d plane;
    plane.origin = modeling::Vec3{frame.origin[0], frame.origin[1], frame.origin[2]};
    plane.normal = modeling::Vec3{frame.normal[0], frame.normal[1], frame.normal[2]};
    plane.xDirection =
        modeling::Vec3{frame.xDirection[0], frame.xDirection[1], frame.xDirection[2]};
    return plane;
}

// The core normalizes and orthogonalizes an explicit plane on its own, but the
// preview has to agree with the result, so the same normalization is applied
// here instead of trusting whatever the picker produced.
PlaneFrame planeFrameOf(const modeling::Plane3d& plane)
{
    const double normal[3] = {plane.normal.x, plane.normal.y, plane.normal.z};
    double xDirection[3] = {plane.xDirection.x, plane.xDirection.y, plane.xDirection.z};

    double normalLength = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        normalLength += normal[axis] * normal[axis];
    }
    normalLength = std::sqrt(normalLength);

    PlaneFrame frame;
    frame.origin[0] = plane.origin.x;
    frame.origin[1] = plane.origin.y;
    frame.origin[2] = plane.origin.z;
    if (normalLength <= 1.0e-12) {
        frame.normal[2] = 1.0;
        frame.xDirection[0] = 1.0;
        frame.yDirection[1] = 1.0;
        return frame;
    }

    for (int axis = 0; axis < 3; ++axis) {
        frame.normal[axis] = normal[axis] / normalLength;
    }
    const double projection = (xDirection[0] * normal[0] + xDirection[1] * normal[1] +
                               xDirection[2] * normal[2]) /
                              (normalLength * normalLength);
    double xLength = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        xDirection[axis] -= projection * normal[axis];
        xLength += xDirection[axis] * xDirection[axis];
    }
    xLength = std::sqrt(xLength);
    if (xLength <= 1.0e-12) {
        xDirection[0] = 1.0;
        xDirection[1] = 0.0;
        xDirection[2] = 0.0;
    } else {
        for (double& value : xDirection) {
            value /= xLength;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        frame.xDirection[axis] = xDirection[axis];
    }
    cross(frame.normal, frame.xDirection, frame.yDirection);
    return frame;
}

QString planeLabel(const modeling::SketchPlaneReference& reference)
{
    if (const auto* datum = std::get_if<modeling::DatumPlaneReference>(&reference)) {
        switch (datum->plane) {
        case modeling::DatumPlane::YZ:
            return QStringLiteral("YZ Plane");
        case modeling::DatumPlane::XZ:
            return QStringLiteral("XZ Plane");
        case modeling::DatumPlane::XY:
        default:
            return QStringLiteral("XY Plane");
        }
    }
    if (std::holds_alternative<modeling::OffsetDatumPlane>(reference)) {
        return QStringLiteral("Offset Plane");
    }
    if (std::holds_alternative<modeling::Plane3d>(reference)) {
        return QStringLiteral("Face Plane");
    }
    const auto& face = std::get<modeling::FaceReference>(reference);
    return QStringLiteral("%1 of #%2").arg(QString::fromLatin1(modeling::faceRoleName(face.role))).arg(face.ownerFeature);
}

bool resolveSketchFrame(const std::vector<modeling::Feature>& features,
                        const modeling::SketchPlaneReference& reference,
                        PlaneFrame& frame)
{
    modeling::Plane3d plane;
    std::string error;
    if (!modeling::resolveSketchPlane(features, reference, plane, error)) return false;
    frame = planeFrameOf(plane);
    return true;
}

}  // namespace sketchapp
