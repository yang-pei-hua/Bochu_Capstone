#pragma once

#include "reconstruction/PointStore.h"

#include <cstddef>
#include <string>

namespace reconstruction {

struct PointCloudPreprocessingOptions {
    // Zero disables voxel downsampling. The representative is an original
    // observation nearest the voxel centre, so its PointId remains valid.
    double voxelSize = 0.0;

    bool removeStatisticalOutliers = true;
    std::size_t outlierNeighborCount = 12;
    double outlierStandardDeviationMultiplier = 2.5;
    // Robust distance ceiling retains neighborhoods up to this multiple of
    // the median, unless the observed MAD calls for a larger ceiling.
    double outlierRobustMedianMultiplier = 3.0;

    bool estimateMissingNormals = true;
    std::size_t normalNeighborCount = 18;
    // Dimensionless propagation gates; directions are anchored only in
    // signed observations, never in a world axis or the cloud centroid.
    double normalOrientationMinimumAlignment = 0.75;
    double normalOrientationMaximumTangentDeviation = 0.55;
    double normalOrientationMaximumSpacingRatio = 4.0;

    // The modeling core uses millimetres. Meter input is converted before any
    // thresholds are applied; arbitrary input remains arbitrary.
    bool convertKnownUnitsToMillimeters = true;

    // Explicit template observations survive voxel and outlier filtering.
    // Other observations retain the normal preprocessing/performance path.
    std::vector<PointId> protectedPointIds;
};

struct PointCloudPreprocessingReport {
    std::size_t inputPointCount = 0;
    std::size_t voxelPointCount = 0;
    std::size_t outputPointCount = 0;
    std::size_t removedByVoxel = 0;
    std::size_t removedAsOutliers = 0;
    std::size_t normalsFromInput = 0;
    std::size_t normalsEstimated = 0;
    std::size_t estimatedNormalsOriented = 0;
    std::size_t normalsWithoutOrientation = 0;
    std::size_t degenerateNeighborhoods = 0;
    std::size_t pointsWithCurvature = 0;
    double appliedScaleToMillimeters = 1.0;
    double outlierMeanNeighborDistance = 0.0;
    double outlierMedianNeighborDistance = 0.0;
    double outlierDistanceThreshold = 0.0;
};

bool preprocessPointCloud(
    const PointStore& input,
    const PointCloudPreprocessingOptions& options,
    PointStore& output,
    PointCloudPreprocessingReport& report,
    std::string& error);

}  // namespace reconstruction
