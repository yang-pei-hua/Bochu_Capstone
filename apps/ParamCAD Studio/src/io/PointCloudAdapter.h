#pragma once

#include "io/PlyReader.h"

#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/data/PointStore.h>
#include <reconstruction/detection/PrimitiveDetector.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QString>

// Translation layer between the raw vertices the PLY reader produces and the
// observation store the reconstruction module consumes. It lives in the client
// so that neither module has to know the other's data types.
class PointCloudAdapter
{
public:
    // Builds the immutable observation store. Vertices that are not finite are
    // dropped here: the store rejects them, and a PLY written by an external
    // tool may legitimately carry them. IDs are one-based source row numbers.
    static reconstruction::PointStore toPointStore(const PlyCloud& cloud,
                                                   reconstruction::LengthUnit unit);

    // Reads the reconstruction manifest beside a generated cloud. Directly
    // opened files without metadata remain arbitrary-unit rather than being
    // silently interpreted as millimetres.
    static reconstruction::LengthUnit lengthUnitFor(const QString& pointCloudPath);

    // Scale-relative defaults keep preprocessing useful for both metric and
    // arbitrary-unit clouds. Known metric input is converted to millimetres by
    // the core preprocessor. The overloads without a settings argument load
    // config/preprocessing.conf and config/detection.conf.
    static reconstruction::PointCloudPreprocessingOptions preprocessingOptionsFor(
        const PlyCloud& cloud);
    static reconstruction::PointCloudPreprocessingOptions preprocessingOptionsFor(
        const PlyCloud& cloud, const reconstruction::PreprocessingSettings& settings);

    static reconstruction::PlaneDetectionOptions planeOptionsFor(const PlyCloud& cloud);
    static reconstruction::PlaneDetectionOptions planeOptionsFor(
        const reconstruction::PointStore& cloud);
    static reconstruction::PlaneDetectionOptions planeOptionsFor(
        const PlyCloud& cloud, const reconstruction::DetectionSettings& settings);
    static reconstruction::PlaneDetectionOptions planeOptionsFor(
        const reconstruction::PointStore& cloud, const reconstruction::DetectionSettings& settings);
    static reconstruction::CylinderDetectionOptions cylinderOptionsFor(
        const reconstruction::PointStore& cloud);
    static reconstruction::CylinderDetectionOptions cylinderOptionsFor(
        const reconstruction::PointStore& cloud, const reconstruction::DetectionSettings& settings);

    // Applies the scale-aware policy to every primitive family in place.
    static void applyDetectionScale(reconstruction::PrimitiveDetectionOptions& options,
                                    double diagonal, std::size_t pointCount,
                                    const reconstruction::DetectionSettings& settings);
};
