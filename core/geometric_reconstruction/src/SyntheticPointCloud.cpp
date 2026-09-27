#include "reconstruction/SyntheticPointCloud.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace reconstruction {
namespace {

using modeling::Vec3;

double dot(const Vec3& left, const Vec3& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec3 cross(const Vec3& left, const Vec3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

Vec3 scaled(const Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 normalized(const Vec3& value) {
    const double length = std::sqrt(dot(value, value));
    if (!std::isfinite(length) || length <= 1.0e-12) {
        throw std::invalid_argument("Synthetic box axes must be non-degenerate");
    }
    return scaled(value, 1.0 / length);
}

Vec3 pointInFrame(
    const modeling::Pose3d& pose,
    const Vec3& xAxis,
    const Vec3& yAxis,
    const Vec3& zAxis,
    double x,
    double y,
    double z) {
    return {
        pose.origin.x + xAxis.x * x + yAxis.x * y + zAxis.x * z,
        pose.origin.y + xAxis.y * x + yAxis.y * y + zAxis.y * z,
        pose.origin.z + xAxis.z * x + yAxis.z * y + zAxis.z * z,
    };
}

}  // namespace

PointStore makeSyntheticBoxSurface(const SyntheticBoxRequest& request) {
    if (request.samplesPerEdge < 2U) {
        throw std::invalid_argument(
            "Synthetic box requires at least two samples per edge");
    }
    if (!std::isfinite(request.box.sizeX) ||
        !std::isfinite(request.box.sizeY) ||
        !std::isfinite(request.box.sizeZ) || request.box.sizeX <= 0.0 ||
        request.box.sizeY <= 0.0 || request.box.sizeZ <= 0.0) {
        throw std::invalid_argument(
            "Synthetic box dimensions must be finite and positive");
    }

    const Vec3 xAxis = normalized(request.box.pose.xDirection);
    const Vec3 zAxis = normalized(request.box.pose.zDirection);
    if (std::abs(dot(xAxis, zAxis)) > 1.0e-9) {
        throw std::invalid_argument("Synthetic box X and Z axes must be orthogonal");
    }
    const Vec3 yAxis = normalized(cross(zAxis, xAxis));

    std::vector<PointSample> points;
    const std::size_t faceSamples =
        request.samplesPerEdge * request.samplesPerEdge;
    points.reserve(6U * faceSamples);
    PointId nextId = 1;

    const auto addFace = [&](const Vec3& normal, auto makeLocalPoint) {
        for (std::size_t row = 0; row < request.samplesPerEdge; ++row) {
            const double v = static_cast<double>(row) /
                static_cast<double>(request.samplesPerEdge - 1U);
            for (std::size_t column = 0;
                 column < request.samplesPerEdge;
                 ++column) {
                const double u = static_cast<double>(column) /
                    static_cast<double>(request.samplesPerEdge - 1U);
                const Vec3 local = makeLocalPoint(u, v);
                points.push_back(PointSample{
                    nextId++,
                    pointInFrame(request.box.pose, xAxis, yAxis, zAxis,
                                 local.x, local.y, local.z),
                    normal,
                    1.0,
                });
            }
        }
    };

    addFace(scaled(xAxis, -1.0), [&](double u, double v) {
        return Vec3{0.0, u * request.box.sizeY, v * request.box.sizeZ};
    });
    addFace(xAxis, [&](double u, double v) {
        return Vec3{request.box.sizeX,
                    u * request.box.sizeY,
                    v * request.box.sizeZ};
    });
    addFace(scaled(yAxis, -1.0), [&](double u, double v) {
        return Vec3{u * request.box.sizeX, 0.0, v * request.box.sizeZ};
    });
    addFace(yAxis, [&](double u, double v) {
        return Vec3{u * request.box.sizeX,
                    request.box.sizeY,
                    v * request.box.sizeZ};
    });
    addFace(scaled(zAxis, -1.0), [&](double u, double v) {
        return Vec3{u * request.box.sizeX, v * request.box.sizeY, 0.0};
    });
    addFace(zAxis, [&](double u, double v) {
        return Vec3{u * request.box.sizeX,
                    v * request.box.sizeY,
                    request.box.sizeZ};
    });

    return PointStore(std::move(points), LengthUnit::Millimeter);
}

}  // namespace reconstruction
