#include "io/PointCloudAdapter.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <array>
#include <cmath>
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

reconstruction::LengthUnit unitFromManifest(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return reconstruction::LengthUnit::Arbitrary;
    }
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject()) {
        return reconstruction::LengthUnit::Arbitrary;
    }
    const QString unit = document.object().value(QStringLiteral("unit"))
                             .toString().trimmed().toLower();
    if (unit == QLatin1String("mm") || unit == QLatin1String("millimeter")
        || unit == QLatin1String("millimetre")) {
        return reconstruction::LengthUnit::Millimeter;
    }
    if (unit == QLatin1String("m") || unit == QLatin1String("meter")
        || unit == QLatin1String("metre")) {
        return reconstruction::LengthUnit::Meter;
    }
    return reconstruction::LengthUnit::Arbitrary;
}

}  // namespace

reconstruction::PointStore PointCloudAdapter::toPointStore(
    const PlyCloud& cloud, reconstruction::LengthUnit unit)
{
    std::vector<reconstruction::PointSample> samples;
    samples.reserve(cloud.positions.size());

    reconstruction::PointId nextId = 1;
    const bool hasNormals = cloud.normals.size() == cloud.positions.size();
    for (std::size_t index = 0; index < cloud.positions.size(); ++index) {
        const std::array<float, 3>& position = cloud.positions[index];
        if (!finitePoint(position)) {
            continue;
        }
        reconstruction::PointSample sample;
        sample.id = nextId++;
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
    const QFileInfo cloudInfo(pointCloudPath);
    const QDir directory = cloudInfo.dir();
    const QStringList candidates{
        directory.filePath(QStringLiteral("workspace/reconstruction.json")),
        directory.filePath(QStringLiteral("reconstruction.json")),
        directory.filePath(cloudInfo.completeBaseName() + QStringLiteral(".json")),
    };
    for (const QString& candidate : candidates) {
        if (QFileInfo(candidate).isFile()) {
            return unitFromManifest(candidate);
        }
    }
    return reconstruction::LengthUnit::Arbitrary;
}

reconstruction::PointCloudPreprocessingOptions
PointCloudAdapter::preprocessingOptionsFor(const PlyCloud& cloud)
{
    reconstruction::PointCloudPreprocessingOptions options;
    const double diagonal = diagonalOf(cloud.positions);
    if (diagonal > 0.0) {
        // At most roughly 200 samples span the bounding-box diagonal before
        // surface occupancy is considered. This keeps dense COLMAP clouds
        // tractable without erasing the edges of ordinary mechanical parts.
        options.voxelSize = diagonal * 5.0e-3;
    }
    return options;
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const PlyCloud& cloud)
{
    reconstruction::PlaneDetectionOptions options;
    if (cloud.positions.empty()) {
        return options;
    }

    const double diagonal = diagonalOf(cloud.positions);

    // A tenth of a percent of the diagonal: tight enough that two parallel
    // sides of a box stay separate planes, loose enough for scan noise, and
    // free of any assumption about the cloud's unit.
    if (diagonal > 0.0) {
        options.distanceThreshold = diagonal * 1.0e-3;
    }
    return options;
}

reconstruction::CylinderDetectionOptions PointCloudAdapter::cylinderOptionsFor(
    const reconstruction::PointStore& cloud)
{
    reconstruction::CylinderDetectionOptions options;
    const double diagonal = diagonalOf(cloud);
    if (diagonal > 0.0) {
        options.distanceThreshold = diagonal * 1.0e-3;
        // Reject sub-resolution cylinders before the semantic containment test.
        options.minimumRadius = options.distanceThreshold * 2.0;
        options.maximumRadius = diagonal * 0.5;
    }
    options.minimumSupportPoints = std::max<std::size_t>(
        30U, std::min<std::size_t>(150U, cloud.size() / 500U));
    options.maximumCylinders = 8;
    return options;
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const reconstruction::PointStore& cloud)
{
    reconstruction::PlaneDetectionOptions options;
    const double diagonal = diagonalOf(cloud);
    if (diagonal > 0.0) {
        options.distanceThreshold = diagonal * 1.0e-3;
    }
    // Downsampling changes absolute support counts. A relative floor keeps all
    // six faces eligible while still rejecting tiny accidental patches.
    options.minimumSupportPoints = std::max<std::size_t>(
        20U, std::min<std::size_t>(200U, cloud.size() / 200U));
    return options;
}
