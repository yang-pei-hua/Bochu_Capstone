#include "detail/PreprocessingInternal.h"

namespace reconstruction::detail::preprocessing {
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
    double voxelSize,
    const std::unordered_set<PointId>& protectedIds) {
    if (voxelSize <= 0.0 || points.empty()) {
        return points;
    }

    std::unordered_map<VoxelKey, VoxelRepresentative, VoxelKeyHash> voxels;
    std::vector<std::size_t> selected;
    voxels.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (protectedIds.count(points[index].id)) {
            selected.push_back(index);
            continue;
        }
        const Vec3& point = points[index].position;
        const std::array<double, 3> cells{{std::floor(point.x / voxelSize),
            std::floor(point.y / voxelSize), std::floor(point.z / voxelSize)}};
        const double minimumCell = static_cast<double>(std::numeric_limits<std::int64_t>::min());
        const double maximumCell = static_cast<double>(std::numeric_limits<std::int64_t>::max());
        if (std::any_of(cells.begin(), cells.end(), [&](double cell) {
                return !std::isfinite(cell) || cell < minimumCell || cell >= maximumCell;
            })) {
            // A robust part-scale voxel can be tiny relative to a far outlier.
            // Keep unindexable observations for filtering instead of invoking
            // an overflowing float-to-integer conversion or losing their IDs.
            selected.push_back(index);
            continue;
        }
        const VoxelKey key{
            static_cast<std::int64_t>(cells[0]),
            static_cast<std::int64_t>(cells[1]),
            static_cast<std::int64_t>(cells[2]),
        };
        const Vec3 center{
            (static_cast<double>(key.x) + 0.5) * voxelSize,
            (static_cast<double>(key.y) + 0.5) * voxelSize,
            (static_cast<double>(key.z) + 0.5) * voxelSize,
        };
        const double distance = squaredDistance(point, center);
        auto [entry, inserted] = voxels.emplace(
            key, VoxelRepresentative{index, distance});
        if (!inserted && (distance < entry->second.squaredDistanceToCenter ||
            (distance == entry->second.squaredDistanceToCenter &&
             points[index].id < points[entry->second.pointIndex].id))) {
            entry->second = VoxelRepresentative{index, distance};
        }
    }

    selected.reserve(selected.size() + voxels.size());
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
} // namespace reconstruction::detail::preprocessing
