#pragma once
#include "reconstruction/preprocessing/PointCloudPreprocessor.h"

#include "common/Math3.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace reconstruction::detail::preprocessing {
using modeling::Vec3;

inline double coordinate(const Vec3& point, int axis) {
    return axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
}

inline double squaredDistance(const Vec3& left, const Vec3& right) {
    const double x = left.x - right.x;
    const double y = left.y - right.y;
    const double z = left.z - right.z;
    return x * x + y * y + z * z;
}



std::vector<PointSample> voxelDownsample(
    const std::vector<PointSample>& points,
    double voxelSize,
    const std::unordered_set<PointId>& protectedIds);

std::vector<PointSample> removeOutliers(
    const std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report,
    const std::unordered_set<PointId>& protectedIds);

void estimateNormalsAndCurvature(
    std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report);

bool validOptions(
    const PointCloudPreprocessingOptions& options,
    std::string& error);
} // namespace reconstruction::detail::preprocessing
