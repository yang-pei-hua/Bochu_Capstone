#include "reconstruction/PointCloudPreprocessor.h"

#include "Math3.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace reconstruction {
namespace {

using modeling::Vec3;

double coordinate(const Vec3& point, int axis) {
    return axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
}

double squaredDistance(const Vec3& left, const Vec3& right) {
    const double x = left.x - right.x;
    const double y = left.y - right.y;
    const double z = left.z - right.z;
    return x * x + y * y + z * z;
}

class KdTree {
public:
    explicit KdTree(const std::vector<PointSample>& points) : points_(points) {
        order_.resize(points.size());
        for (std::size_t index = 0; index < order_.size(); ++index) {
            order_[index] = index;
        }
        nodes_.reserve(points.size());
        root_ = build(0U, order_.size(), 0);
    }

    std::vector<std::pair<double, std::size_t>> nearest(
        std::size_t queryIndex,
        std::size_t count) const {
        Heap heap;
        search(root_, points_[queryIndex].position, queryIndex, count, heap);
        std::vector<std::pair<double, std::size_t>> result;
        result.reserve(heap.size());
        while (!heap.empty()) {
            result.push_back(heap.top());
            heap.pop();
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

private:
    struct Node {
        std::size_t pointIndex = 0;
        int axis = 0;
        int left = -1;
        int right = -1;
    };

    using Heap = std::priority_queue<std::pair<double, std::size_t>>;

    int build(std::size_t begin, std::size_t end, int depth) {
        if (begin >= end) {
            return -1;
        }
        const int axis = depth % 3;
        const std::size_t middle = begin + (end - begin) / 2U;
        std::nth_element(
            order_.begin() + static_cast<std::ptrdiff_t>(begin),
            order_.begin() + static_cast<std::ptrdiff_t>(middle),
            order_.begin() + static_cast<std::ptrdiff_t>(end),
            [this, axis](std::size_t left, std::size_t right) {
                return coordinate(points_[left].position, axis) <
                    coordinate(points_[right].position, axis);
            });
        const int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{order_[middle], axis, -1, -1});
        const int left = build(begin, middle, depth + 1);
        const int right = build(middle + 1U, end, depth + 1);
        nodes_[static_cast<std::size_t>(nodeIndex)].left = left;
        nodes_[static_cast<std::size_t>(nodeIndex)].right = right;
        return nodeIndex;
    }

    void search(
        int nodeIndex,
        const Vec3& query,
        std::size_t excludedIndex,
        std::size_t count,
        Heap& heap) const {
        if (nodeIndex < 0 || count == 0U) {
            return;
        }
        const Node& node = nodes_[static_cast<std::size_t>(nodeIndex)];
        const Vec3& point = points_[node.pointIndex].position;
        if (node.pointIndex != excludedIndex) {
            const double distance = squaredDistance(query, point);
            if (heap.size() < count) {
                heap.emplace(distance, node.pointIndex);
            } else if (distance < heap.top().first) {
                heap.pop();
                heap.emplace(distance, node.pointIndex);
            }
        }

        const double delta = coordinate(query, node.axis) -
            coordinate(point, node.axis);
        const int nearChild = delta < 0.0 ? node.left : node.right;
        const int farChild = delta < 0.0 ? node.right : node.left;
        search(nearChild, query, excludedIndex, count, heap);
        if (heap.size() < count || delta * delta < heap.top().first) {
            search(farChild, query, excludedIndex, count, heap);
        }
    }

    const std::vector<PointSample>& points_;
    std::vector<std::size_t> order_;
    std::vector<Node> nodes_;
    int root_ = -1;
};

struct VoxelKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U +
            (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U +
            (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct VoxelRepresentative {
    std::size_t pointIndex = 0;
    double squaredDistanceToCenter = std::numeric_limits<double>::infinity();
};

std::vector<PointSample> voxelDownsample(
    const std::vector<PointSample>& points,
    double voxelSize) {
    if (voxelSize <= 0.0 || points.empty()) {
        return points;
    }

    std::unordered_map<VoxelKey, VoxelRepresentative, VoxelKeyHash> voxels;
    voxels.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        const Vec3& point = points[index].position;
        const VoxelKey key{
            static_cast<std::int64_t>(std::floor(point.x / voxelSize)),
            static_cast<std::int64_t>(std::floor(point.y / voxelSize)),
            static_cast<std::int64_t>(std::floor(point.z / voxelSize)),
        };
        const Vec3 center{
            (static_cast<double>(key.x) + 0.5) * voxelSize,
            (static_cast<double>(key.y) + 0.5) * voxelSize,
            (static_cast<double>(key.z) + 0.5) * voxelSize,
        };
        const double distance = squaredDistance(point, center);
        auto [entry, inserted] = voxels.emplace(
            key, VoxelRepresentative{index, distance});
        if (!inserted && distance < entry->second.squaredDistanceToCenter) {
            entry->second = VoxelRepresentative{index, distance};
        }
    }

    std::vector<std::size_t> selected;
    selected.reserve(voxels.size());
    for (const auto& entry : voxels) {
        selected.push_back(entry.second.pointIndex);
    }
    // Preserve source order and deterministic PointId ordering regardless of
    // unordered_map bucket layout.
    std::sort(selected.begin(), selected.end());
    std::vector<PointSample> result;
    result.reserve(selected.size());
    for (const std::size_t index : selected) {
        result.push_back(points[index]);
    }
    return result;
}

std::vector<PointSample> removeOutliers(
    const std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report) {
    if (!options.removeStatisticalOutliers || points.size() < 4U ||
        options.outlierNeighborCount == 0U) {
        return points;
    }

    const std::size_t neighborCount = std::min(
        options.outlierNeighborCount, points.size() - 1U);
    const KdTree tree(points);
    std::vector<double> meanDistances(points.size(), 0.0);
    double globalMean = 0.0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto neighbors = tree.nearest(index, neighborCount);
        double total = 0.0;
        for (const auto& neighbor : neighbors) {
            total += std::sqrt(neighbor.first);
        }
        meanDistances[index] = neighbors.empty()
            ? 0.0
            : total / static_cast<double>(neighbors.size());
        globalMean += meanDistances[index];
    }
    globalMean /= static_cast<double>(meanDistances.size());

    double variance = 0.0;
    for (const double distance : meanDistances) {
        const double delta = distance - globalMean;
        variance += delta * delta;
    }
    variance /= static_cast<double>(meanDistances.size());
    const double threshold = globalMean +
        options.outlierStandardDeviationMultiplier * std::sqrt(variance);
    report.outlierMeanNeighborDistance = globalMean;
    report.outlierDistanceThreshold = threshold;

    std::vector<PointSample> result;
    result.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (meanDistances[index] <= threshold) {
            result.push_back(points[index]);
        }
    }
    report.removedAsOutliers = points.size() - result.size();
    return result;
}

void estimateNormalsAndCurvature(
    std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report) {
    if (points.size() < 4U || options.normalNeighborCount < 3U) {
        return;
    }
    const std::size_t neighborCount = std::min(
        options.normalNeighborCount, points.size() - 1U);
    const KdTree tree(points);
    for (std::size_t index = 0; index < points.size(); ++index) {
        PointSample& sample = points[index];
        if (sample.normal.has_value()) {
            try {
                sample.normal = math3::canonicalDirection(*sample.normal);
                ++report.normalsFromInput;
            } catch (const std::invalid_argument&) {
                sample.normal.reset();
            }
        }

        const auto neighbors = tree.nearest(index, neighborCount);
        if (neighbors.size() < 3U) {
            continue;
        }
        Vec3 centroid = sample.position;
        for (const auto& neighbor : neighbors) {
            centroid = math3::add(centroid, points[neighbor.second].position);
        }
        centroid = math3::scale(
            centroid, 1.0 / static_cast<double>(neighbors.size() + 1U));

        std::array<std::array<double, 3>, 3> covariance{};
        const auto accumulate = [&covariance, &centroid](const Vec3& position) {
            const Vec3 delta = math3::subtract(position, centroid);
            const std::array<double, 3> values{{delta.x, delta.y, delta.z}};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    covariance[row][column] += values[row] * values[column];
                }
            }
        };
        accumulate(sample.position);
        for (const auto& neighbor : neighbors) {
            accumulate(points[neighbor.second].position);
        }

        try {
            const math3::SymmetricEigenResult eigen =
                math3::symmetricEigenDecomposition(covariance);
            const double sum = eigen.values[0] + eigen.values[1] +
                eigen.values[2];
            if (sum > 1.0e-18) {
                sample.curvature = eigen.values[0] / sum;
                ++report.pointsWithCurvature;
            }
            if (!sample.normal.has_value() && options.estimateMissingNormals) {
                sample.normal = eigen.vectors[0];
                ++report.normalsEstimated;
            }
        } catch (const std::invalid_argument&) {
            // A coincident or collinear neighbourhood has no stable surface
            // normal. Preserve the point and leave its attributes absent.
        }
    }
}

bool validOptions(
    const PointCloudPreprocessingOptions& options,
    std::string& error) {
    if (!std::isfinite(options.voxelSize) || options.voxelSize < 0.0) {
        error = "Voxel size must be finite and non-negative";
        return false;
    }
    if (!std::isfinite(options.outlierStandardDeviationMultiplier) ||
        options.outlierStandardDeviationMultiplier < 0.0) {
        error = "Outlier standard-deviation multiplier must be finite and non-negative";
        return false;
    }
    if (options.estimateMissingNormals && options.normalNeighborCount < 3U) {
        error = "Normal estimation requires at least three neighbors";
        return false;
    }
    return true;
}

}  // namespace

bool preprocessPointCloud(
    const PointStore& input,
    const PointCloudPreprocessingOptions& options,
    PointStore& output,
    PointCloudPreprocessingReport& report,
    std::string& error) {
    report = {};
    error.clear();
    if (!validOptions(options, error)) {
        return false;
    }
    if (input.empty()) {
        error = "Point-cloud preprocessing requires at least one point";
        return false;
    }

    report.inputPointCount = input.size();
    std::vector<PointSample> points = input.points();
    LengthUnit outputUnit = input.unit();
    if (options.convertKnownUnitsToMillimeters) {
        const std::optional<double> scale = millimetersPerUnit(input.unit());
        if (scale.has_value()) {
            report.appliedScaleToMillimeters = *scale;
            if (*scale != 1.0) {
                for (PointSample& point : points) {
                    point.position = math3::scale(point.position, *scale);
                }
            }
            outputUnit = LengthUnit::Millimeter;
        }
    }

    double voxelSize = options.voxelSize;
    if (input.unit() == LengthUnit::Meter && outputUnit == LengthUnit::Millimeter) {
        voxelSize *= 1000.0;
    }
    points = voxelDownsample(points, voxelSize);
    report.voxelPointCount = points.size();
    report.removedByVoxel = report.inputPointCount - points.size();

    points = removeOutliers(points, options, report);
    estimateNormalsAndCurvature(points, options, report);
    report.outputPointCount = points.size();
    if (points.empty()) {
        error = "Point-cloud preprocessing removed every point";
        return false;
    }

    output = PointStore(std::move(points), outputUnit);
    return true;
}

}  // namespace reconstruction
