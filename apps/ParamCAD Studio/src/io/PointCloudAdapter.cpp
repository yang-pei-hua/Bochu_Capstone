#include "io/PointCloudAdapter.h"

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

}  // namespace

reconstruction::PointStore PointCloudAdapter::toPointStore(
    const PlyCloud& cloud, reconstruction::LengthUnit unit)
{
    std::vector<reconstruction::PointSample> samples;
    samples.reserve(cloud.positions.size());

    reconstruction::PointId nextId = 1;
    for (const std::array<float, 3>& position : cloud.positions) {
        if (!finitePoint(position)) {
            continue;
        }
        reconstruction::PointSample sample;
        sample.id = nextId++;
        sample.position = modeling::Vec3{position[0], position[1], position[2]};
        // A PLY carries vertices only on the way in, so no normal is recorded;
        // the plane fit derives its own from each support set.
        sample.confidence = 1.0;
        samples.push_back(sample);
    }

    return reconstruction::PointStore(std::move(samples), unit);
}

reconstruction::PlaneDetectionOptions PointCloudAdapter::planeOptionsFor(
    const PlyCloud& cloud)
{
    reconstruction::PlaneDetectionOptions options;
    if (cloud.positions.empty()) {
        return options;
    }

    std::array<float, 3> lower{cloud.positions.front()};
    std::array<float, 3> upper{cloud.positions.front()};
    for (const std::array<float, 3>& position : cloud.positions) {
        if (!finitePoint(position)) {
            continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
            lower[axis] = std::min(lower[axis], position[axis]);
            upper[axis] = std::max(upper[axis], position[axis]);
        }
    }

    double squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double extent = static_cast<double>(upper[axis]) - lower[axis];
        squared += extent * extent;
    }
    const double diagonal = std::sqrt(squared);

    // A tenth of a percent of the diagonal: tight enough that two parallel
    // sides of a box stay separate planes, loose enough for scan noise, and
    // free of any assumption about the cloud's unit.
    if (diagonal > 0.0) {
        options.distanceThreshold = diagonal * 1.0e-3;
    }
    return options;
}