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

    bool estimateMissingNormals = true;
    std::size_t normalNeighborCount = 18;

    // The modeling core uses millimetres. Meter input is converted before any
    // thresholds are applied; arbitrary input remains arbitrary.
    bool convertKnownUnitsToMillimeters = true;
};

struct PointCloudPreprocessingReport {
    std::size_t inputPointCount = 0;
    std::size_t voxelPointCount = 0;
    std::size_t outputPointCount = 0;
    std::size_t removedByVoxel = 0;
    std::size_t removedAsOutliers = 0;
    std::size_t normalsFromInput = 0;
    std::size_t normalsEstimated = 0;
    std::size_t pointsWithCurvature = 0;
    double appliedScaleToMillimeters = 1.0;
    double outlierMeanNeighborDistance = 0.0;
    double outlierDistanceThreshold = 0.0;
};

bool preprocessPointCloud(
    const PointStore& input,
    const PointCloudPreprocessingOptions& options,
    PointStore& output,
    PointCloudPreprocessingReport& report,
    std::string& error);

}  // namespace reconstruction
