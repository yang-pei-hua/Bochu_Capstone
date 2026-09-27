#include "reconstruction/SyntheticPointCloud.h"

#include <cmath>
#include <random>
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
    if (!std::isfinite(request.gaussianNoiseSigma) ||
        request.gaussianNoiseSigma < 0.0 ||
        !std::isfinite(request.outlierPaddingFraction) ||
        request.outlierPaddingFraction < 0.0) {
        throw std::invalid_argument(
            "Synthetic noise and outlier padding must be finite and non-negative");
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
    points.reserve(6U * faceSamples + request.outlierCount);
    PointId nextId = 1;
    std::mt19937 generator(request.randomSeed);
    std::normal_distribution<double> noise(0.0, request.gaussianNoiseSigma);

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
                Vec3 position = pointInFrame(
                    request.box.pose, xAxis, yAxis, zAxis,
                    local.x, local.y, local.z);
                position.x += noise(generator);
                position.y += noise(generator);
                position.z += noise(generator);
                points.push_back(PointSample{
                    nextId++,
                    position,
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

    const double paddingX = request.box.sizeX * request.outlierPaddingFraction;
    const double paddingY = request.box.sizeY * request.outlierPaddingFraction;
    const double paddingZ = request.box.sizeZ * request.outlierPaddingFraction;
    std::uniform_real_distribution<double> outlierX(
        -paddingX, request.box.sizeX + paddingX);
    std::uniform_real_distribution<double> outlierY(
        -paddingY, request.box.sizeY + paddingY);
    std::uniform_real_distribution<double> outlierZ(
        -paddingZ, request.box.sizeZ + paddingZ);
    for (std::size_t index = 0; index < request.outlierCount; ++index) {
        const Vec3 local{outlierX(generator), outlierY(generator), outlierZ(generator)};
        points.push_back(PointSample{
            nextId++,
            pointInFrame(request.box.pose, xAxis, yAxis, zAxis,
                         local.x, local.y, local.z),
            std::nullopt,
            0.25,
        });
    }

    return PointStore(std::move(points), LengthUnit::Millimeter);
}

}  // namespace reconstruction
