#include "detail/KdTree.h"

namespace reconstruction::detail::preprocessing {
void estimateNormalsAndCurvature(
    std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report) {
    for (PointSample& sample : points) {
        if (sample.normal.has_value()) {
            try {
                // Input normals carry outward/inward orientation needed for
                // material occupancy. Canonicalizing destroys that information.
                sample.normal = math3::normalized(*sample.normal);
                ++report.normalsFromInput;
            } catch (const std::invalid_argument&) {
                sample.normal.reset();
            }
        }
    }
    if (points.size() < 4U || options.normalNeighborCount < 3U) {
        for (const auto& sample : points)
            if (sample.normal && !sample.normalOrientationKnown) ++report.normalsWithoutOrientation;
        return;
    }
    const std::size_t neighborCount = std::min(
        options.normalNeighborCount, points.size() - 1U);
    const KdTree tree(points);
    std::vector<double> spacing(points.size(), 0.0);
    for (std::size_t index = 0; index < points.size(); ++index) {
        PointSample& sample = points[index];

        const auto neighbors = tree.nearest(index, neighborCount);
        for (const auto& neighbor : neighbors) {
            if (neighbor.first > 1.0e-24) {
                spacing[index] = std::sqrt(neighbor.first);
                break;
            }
        }
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
            // Rank-one lines and coincident samples have no surface normal,
            // even though the eigensolver returns a valid unit eigenvector.
            if (eigen.values[1] <= std::max(1.0e-18, eigen.values[2] * 1.0e-8)) {
                ++report.degenerateNeighborhoods;
                continue;
            }
            if (sum > 1.0e-18) {
                sample.curvature = eigen.values[0] / sum;
                ++report.pointsWithCurvature;
            }
            if (!sample.normal.has_value() && options.estimateMissingNormals) {
                sample.normal = eigen.vectors[0];
                sample.normalOrientationKnown = false;
                ++report.normalsEstimated;
            }
        } catch (const std::invalid_argument&) {
            // A coincident or collinear neighbourhood has no stable surface
            // normal. Preserve the point and leave its attributes absent.
        }
    }

    if (std::none_of(points.begin(), points.end(), [](const PointSample& point) {
            return point.normal && !point.normalOrientationKnown;
        })) return;

    // A multi-source shortest-path forest uses only signed input observations
    // as roots. Edge weights prefer smooth nearby directions. Tangent tests
    // prevent jumping across thin parallel walls or across a disconnected gap.
    // Input signs are immutable; components without a root remain unoriented.
    using Entry = std::tuple<double, PointId, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    std::vector<double> costs(points.size(), std::numeric_limits<double>::infinity());
    std::vector<bool> settled(points.size(), false);
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (points[index].normal && points[index].normalOrientationKnown) {
            costs[index] = 0;
            queue.emplace(0, points[index].id, index);
        }
    }
    while (!queue.empty()) {
        const auto [cost, id, index] = queue.top();
        queue.pop();
        (void)id;
        if (settled[index] || cost != costs[index]) continue;
        settled[index] = true;
        const auto neighbors = tree.nearest(index, neighborCount);
        for (const auto& [distanceSquared, neighbor] : neighbors) {
            auto& target = points[neighbor];
            if (settled[neighbor] || !target.normal || target.normalOrientationKnown) continue;
            const auto delta = math3::subtract(target.position, points[index].position);
            const double distance = std::sqrt(distanceSquared);
            const double alignment = math3::dot(*points[index].normal, *target.normal);
            if (std::abs(alignment) < options.normalOrientationMinimumAlignment || distance <= 1.0e-12 ||
                distance > options.normalOrientationMaximumSpacingRatio * std::max(spacing[index], spacing[neighbor]) ||
                std::abs(math3::dot(delta, *points[index].normal)) > options.normalOrientationMaximumTangentDeviation * distance ||
                std::abs(math3::dot(delta, *target.normal)) > options.normalOrientationMaximumTangentDeviation * distance) continue;
            const double nextCost = cost + 1.0 - std::abs(alignment) +
                1.0e-3 * distance / std::max(spacing[index], spacing[neighbor]);
            if (nextCost >= costs[neighbor]) continue;
            costs[neighbor] = nextCost;
            if (alignment < 0) target.normal = math3::scale(*target.normal, -1);
            queue.emplace(nextCost, target.id, neighbor);
        }
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        auto& point = points[index];
        if (!point.normal || point.normalOrientationKnown) continue;
        if (settled[index]) {
            point.normalOrientationKnown = true;
            ++report.estimatedNormalsOriented;
        } else {
            ++report.normalsWithoutOrientation;
        }
    }
}
} // namespace reconstruction::detail::preprocessing
