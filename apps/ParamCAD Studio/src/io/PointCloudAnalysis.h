#pragma once

#include "core/PointCloudSelection.h"
#include <reconstruction/GuidedSolidRecognizer.h>

struct PointCloudAnalysisResult
{
    std::vector<DetectedPointCloudSurface> surfaces;
    QString error;
    size_t processedPoints = 0;
    size_t assignedPoints = 0;
};

// Runs off the UI thread. Detector point IDs refer to the input observations;
// projection to all original observations recovers downsampled support without
// confusing representative indices with source indices.
PointCloudAnalysisResult analysePointCloud(const PlyCloud& cloud,
    const std::vector<vtkIdType>& originalIds, double distanceTolerance,
    const QString& geometry = QString());
PointCloudRegion describeTemplateFit(const reconstruction::TemplateFit& fit, double scaleToMillimeters);
