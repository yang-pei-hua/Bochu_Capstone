#include "detail/PreprocessingInternal.h"

namespace reconstruction::detail::preprocessing {
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
    if (!std::isfinite(options.outlierRobustMedianMultiplier) || options.outlierRobustMedianMultiplier < 1.0 ||
        !std::isfinite(options.normalOrientationMinimumAlignment) ||
        options.normalOrientationMinimumAlignment <= 0 || options.normalOrientationMinimumAlignment > 1 ||
        !std::isfinite(options.normalOrientationMaximumTangentDeviation) ||
        options.normalOrientationMaximumTangentDeviation < 0 || options.normalOrientationMaximumTangentDeviation >= 1 ||
        !std::isfinite(options.normalOrientationMaximumSpacingRatio) || options.normalOrientationMaximumSpacingRatio < 1) {
        error = "Invalid dimensionless robust filtering or normal orientation gates";
        return false;
    }
    return true;
}
} // namespace reconstruction::detail::preprocessing

namespace reconstruction {
using namespace detail::preprocessing;
namespace {
bool validateInput(const PointStore& input, const PointCloudPreprocessingOptions& options,
    PointCloudPreprocessingReport& report, std::string& error) {
    report = {};
    error.clear();
    if (!validOptions(options, error)) return false;
    if (input.empty()) {
        error = "Point-cloud preprocessing requires at least one point";
        return false;
    }
    return true;
}

bool preprocessInFrame(const PointStore& input, double scale,
    const PointCloudPreprocessingOptions& options, PointStore& output,
    PointCloudPreprocessingReport& report, std::string& error) {
    report.inputPointCount = input.size();
    report.appliedScaleToMillimeters = scale;
    std::vector<PointSample> points = input.points();
    const double voxelSize = options.voxelSize * scale;
    const std::unordered_set<PointId> protectedIds(options.protectedPointIds.begin(),
                                                 options.protectedPointIds.end());
    points = voxelDownsample(points, voxelSize, protectedIds);
    report.voxelPointCount = points.size();
    report.removedByVoxel = report.inputPointCount - points.size();

    points = removeOutliers(points, options, report, protectedIds);
    estimateNormalsAndCurvature(points, options, report);
    report.outputPointCount = points.size();
    if (points.empty()) {
        error = "Point-cloud preprocessing removed every point";
        return false;
    }

    output = PointStore(std::move(points), input.unit());
    return true;
}
} // namespace

bool preprocessPointCloud(
    const PointStore& input,
    const PointCloudPreprocessingOptions& options,
    PointStore& output,
    PointCloudPreprocessingReport& report,
    std::string& error) {
    if (!validateInput(input, options, report, error)) return false;
    if (!options.convertKnownUnitsToMillimeters || input.unit() != LengthUnit::Meter)
        return preprocessInFrame(input, 1.0, options, output, report, error);
    const ObservationSnapshot snapshot(input);
    return preprocessInFrame(snapshot.observations(), snapshot.sourceToWorkingScale(),
        options, output, report, error);
}

bool preprocessPointCloud(
    const ObservationSnapshot& input,
    const PointCloudPreprocessingOptions& options,
    PointStore& output,
    PointCloudPreprocessingReport& report,
    std::string& error) {
    if (!validateInput(input.observations(), options, report, error)) return false;
    return preprocessInFrame(input.observations(), input.sourceToWorkingScale(),
        options, output, report, error);
}
} // namespace reconstruction
