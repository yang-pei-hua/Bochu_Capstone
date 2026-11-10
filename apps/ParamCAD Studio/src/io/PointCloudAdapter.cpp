#include "io/PointCloudAdapter.h"

#include "io/PointCloudMetadata.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

bool finitePoint(const std::array<float, 3>& position)
{
    return std::isfinite(position[0]) && std::isfinite(position[1])
        && std::isfinite(position[2]);
}

double diagonalOf(const std::vector<std::array<float, 3>>& positions)
{
    std::array<float, 3> lower{};
    std::array<float, 3> upper{};
    bool initialized = false;
    for (const std::array<float, 3>& position : positions) {
        if (!finitePoint(position)) {
            continue;
        }
        if (!initialized) {
            lower = upper = position;
            initialized = true;
            continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
            lower[axis] = std::min(lower[axis], position[axis]);
            upper[axis] = std::max(upper[axis], position[axis]);
        }
    }
    if (!initialized) {
        return 0.0;
    }
    double squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double extent = static_cast<double>(upper[axis]) - lower[axis];
        squared += extent * extent;
    }
    return std::sqrt(squared);
}

double diagonalOf(const reconstruction::PointStore& cloud)
{
    if (cloud.empty()) {
        return 0.0;
    }
    modeling::Vec3 lower = cloud.points().front().position;
    modeling::Vec3 upper = lower;
    for (const reconstruction::PointSample& point : cloud.points()) {
        lower.x = std::min(lower.x, point.position.x);
        lower.y = std::min(lower.y, point.position.y);
        lower.z = std::min(lower.z, point.position.z);
        upper.x = std::max(upper.x, point.position.x);
        upper.y = std::max(upper.y, point.position.y);
        upper.z = std::max(upper.z, point.position.z);
    }
    const double x = upper.x - lower.x;
    const double y = upper.y - lower.y;
    const double z = upper.z - lower.z;
    return std::sqrt(x * x + y * y + z * z);
}

double robustDiagonalOf(const std::vector<std::array<float, 3>>& positions)
{
    std::array<std::vector<float>, 3> coordinates;
    for (auto& axis : coordinates) axis.reserve(positions.size());
    for (const auto& point : positions) {
        if (!finitePoint(point)) continue;
        for (int axis = 0; axis < 3; ++axis) coordinates[axis].push_back(point[axis]);
    }
    if (coordinates[0].size() < 100) return diagonalOf(positions);
    double squared = 0;
    for (auto& axis : coordinates) {
        const auto trim = axis.size() / 100;
        std::nth_element(axis.begin(), axis.begin() + trim, axis.end());
        const double lower = axis[trim];
        std::nth_element(axis.begin(), axis.end() - trim - 1, axis.end());
        const double extent = axis[axis.size() - trim - 1] - lower;
        squared += extent * extent;
    }
    const double diagonal = std::sqrt(squared);
    return diagonal > 0 ? diagonal : diagonalOf(positions);
}

}  // namespace

reconstruction::PointStore PointCloudAdapter::toPointStore(
    const PlyCloud& cloud, reconstruction::LengthUnit unit)
{
    if (!cloud.sourceRows.empty() && cloud.sourceRows.size() != cloud.positions.size())
        throw std::invalid_argument("Point-cloud source rows must align with positions");

    std::vector<reconstruction::PointSample> samples;
    samples.reserve(cloud.positions.size());

    const bool hasNormals = cloud.normals.size() == cloud.positions.size();
    for (std::size_t index = 0; index < cloud.positions.size(); ++index) {
        const std::array<float, 3>& position = cloud.positions[index];
        if (!finitePoint(position)) {
            continue;
        }
        reconstruction::PointSample sample;
        // IDs refer to original rows even when a non-finite row is skipped.
        const auto row = cloud.sourceRows.empty() ? index : cloud.sourceRows[index];
        sample.id = static_cast<reconstruction::PointId>(row) + 1;
        sample.position = modeling::Vec3{position[0], position[1], position[2]};
        if (hasNormals && finitePoint(cloud.normals[index])) {
            const std::array<float, 3>& normal = cloud.normals[index];
            const double squaredLength = static_cast<double>(normal[0]) * normal[0]
                + static_cast<double>(normal[1]) * normal[1]
                + static_cast<double>(normal[2]) * normal[2];
            if (squaredLength > 1.0e-20) {
                sample.normal = modeling::Vec3{normal[0], normal[1], normal[2]};
            }
        }
        sample.confidence = 1.0;
        samples.push_back(sample);
    }

    return reconstruction::PointStore(std::move(samples), unit);
}

reconstruction::LengthUnit PointCloudAdapter::lengthUnitFor(
    const QString& pointCloudPath)
{
    const auto unit = pointCloudUnitName(pointCloudPath);
    if (unit == QLatin1String("mm")) return reconstruction::LengthUnit::Millimeter;
    if (unit == QLatin1String("m")) return reconstruction::LengthUnit::Meter;
    return reconstruction::LengthUnit::Arbitrary;
}

reconstruction::PointCloudPreprocessingOptions
PointCloudAdapter::preprocessingOptionsFor(const PlyCloud& cloud)
{
    return preprocessingOptionsFor(cloud, reconstruction::loadPreprocessingSettings());
}

reconstruction::PointCloudPreprocessingOptions
PointCloudAdapter::preprocessingOptionsFor(const PlyCloud& cloud,
                                           const reconstruction::PreprocessingSettings& settings)
{
    auto options = settings.options;
    // A few distant observations must not collapse the part into a handful of
    // oversized voxels before statistical filtering can remove them.
    const double diagonal = robustDiagonalOf(cloud.positions);
    if (settings.voxelSizeMm > 0) {
        options.voxelSize = settings.voxelSizeMm;
    } else if (diagonal > 0.0) {
        // Preserve thin walls in mixed-scale scenes. The former diagonal/200
        // voxel erased most axial samples of a small hole in a large plate.
        options.voxelSize = diagonal * settings.voxelDiagonalFraction;
    }
    return options;
}

void PointCloudAdapter::applyDetectionScale(reconstruction::PrimitiveDetectionOptions& options,
                                            double diagonal, std::size_t pointCount,
                                            const reconstruction::DetectionSettings& settings)
{
    const double threshold = diagonal > 0.0
        ? diagonal * settings.distanceToleranceDiagonalFraction : 0.0;
    const auto support = [pointCount](std::size_t divisor, std::size_t floor, std::size_t ceiling) {
        if (divisor == 0) return floor;
        return std::max(floor, std::min(ceiling, pointCount / divisor));
    };
    auto apply = [&](auto& family, std::size_t divisor, std::size_t floor, std::size_t ceiling) {
        if (family.distanceThreshold <= 0.0) family.distanceThreshold = threshold;
        if (family.minimumSupportPoints == 0)
            family.minimumSupportPoints = support(divisor, floor, ceiling);
    };
    apply(options.plane, settings.planeSupportDivisor, settings.planeSupportFloor, settings.planeSupportCeiling);
    apply(options.cylinder, settings.cylinderSupportDivisor, settings.cylinderSupportFloor, settings.cylinderSupportCeiling);
    apply(options.sphere, settings.curvedSupportDivisor, settings.curvedSupportFloor, settings.curvedSupportCeiling);
    apply(options.cone, settings.curvedSupportDivisor, settings.curvedSupportFloor, settings.curvedSupportCeiling);
    apply(options.torus, settings.curvedSupportDivisor, settings.curvedSupportFloor, settings.curvedSupportCeiling);
    if (!std::isfinite(options.cylinder.maximumRadius))
        options.cylinder.maximumRadius = diagonal * settings.cylinderMaximumRadiusDiagonalFraction;
    if (!std::isfinite(options.sphere.maximumRadius))
        options.sphere.maximumRadius = diagonal * settings.cylinderMaximumRadiusDiagonalFraction;
    if (!std::isfinite(options.torus.maximumMajorRadius))
        options.torus.maximumMajorRadius = diagonal * settings.cylinderMaximumRadiusDiagonalFraction;
    if (!std::isfinite(options.torus.maximumMinorRadius))
        options.torus.maximumMinorRadius = diagonal * settings.cylinderMaximumRadiusDiagonalFraction;
    if (options.cylinder.minimumRadius <= 0.0)
        options.cylinder.minimumRadius = options.cylinder.distanceThreshold
            * settings.cylinderMinimumRadiusToleranceMultiplier;
    if (options.sphere.minimumRadius <= 0.0)
        options.sphere.minimumRadius = options.sphere.distanceThreshold
            * settings.cylinderMinimumRadiusToleranceMultiplier;
    if (options.torus.minimumMajorRadius <= 0.0)
        options.torus.minimumMajorRadius = options.torus.distanceThreshold
            * settings.cylinderMinimumRadiusToleranceMultiplier;
    if (options.torus.minimumMinorRadius <= 0.0)
        options.torus.minimumMinorRadius = options.torus.distanceThreshold
            * settings.cylinderMinimumRadiusToleranceMultiplier;
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const PlyCloud& cloud)
{
    return planeOptionsFor(cloud, reconstruction::loadDetectionSettings());
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const PlyCloud& cloud, const reconstruction::DetectionSettings& settings)
{
    reconstruction::PrimitiveDetectionOptions options = settings.options;
    applyDetectionScale(options, diagonalOf(cloud.positions), cloud.positions.size(), settings);
    return options.plane;
}

reconstruction::CylinderDetectionOptions PointCloudAdapter::cylinderOptionsFor(
    const reconstruction::PointStore& cloud)
{
    return cylinderOptionsFor(cloud, reconstruction::loadDetectionSettings());
}

reconstruction::CylinderDetectionOptions PointCloudAdapter::cylinderOptionsFor(
    const reconstruction::PointStore& cloud, const reconstruction::DetectionSettings& settings)
{
    reconstruction::PrimitiveDetectionOptions options = settings.options;
    applyDetectionScale(options, diagonalOf(cloud), cloud.size(), settings);
    return options.cylinder;
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const reconstruction::PointStore& cloud)
{
    return planeOptionsFor(cloud, reconstruction::loadDetectionSettings());
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const reconstruction::PointStore& cloud, const reconstruction::DetectionSettings& settings)
{
    reconstruction::PrimitiveDetectionOptions options = settings.options;
    applyDetectionScale(options, diagonalOf(cloud), cloud.size(), settings);
    return options.plane;
}
