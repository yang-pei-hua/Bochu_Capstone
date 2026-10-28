#include "detail/KdTree.h"

namespace reconstruction::detail::preprocessing {
std::vector<PointSample> removeOutliers(
    const std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report,
    const std::unordered_set<PointId>& protectedIds) {
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
    double threshold = globalMean +
        options.outlierStandardDeviationMultiplier * std::sqrt(variance);
    auto orderedDistances = meanDistances;
    const auto middle = orderedDistances.begin() + orderedDistances.size() / 2;
    std::nth_element(orderedDistances.begin(), middle, orderedDistances.end());
    const double median = *middle;
    std::vector<double> deviations;
    deviations.reserve(meanDistances.size());
    for (const double distance : meanDistances) deviations.push_back(std::abs(distance - median));
    const auto deviationMiddle = deviations.begin() + deviations.size() / 2;
    std::nth_element(deviations.begin(), deviationMiddle, deviations.end());
    // Distant points inflate both mean and variance, masking less-distant
    // outliers. A robust ceiling prevents that without repeatedly trimming
    // legitimate sparse edges or applying a physical-unit cutoff.
    if (median > 1.0e-12) {
        threshold = std::min(threshold, std::max(options.outlierRobustMedianMultiplier * median,
            median + options.outlierStandardDeviationMultiplier * 1.4826 * *deviationMiddle));
    }
    report.outlierMeanNeighborDistance = globalMean;
    report.outlierMedianNeighborDistance = median;
    report.outlierDistanceThreshold = threshold;

    std::vector<PointSample> result;
    result.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (meanDistances[index] <= threshold || protectedIds.count(points[index].id)) {
            result.push_back(points[index]);
        }
    }
    report.removedAsOutliers = points.size() - result.size();
    return result;
}
} // namespace reconstruction::detail::preprocessing
