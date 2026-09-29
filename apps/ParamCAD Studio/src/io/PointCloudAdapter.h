#pragma once

#include "io/PlyReader.h"

#include <reconstruction/PointCloudPreprocessor.h>
#include <reconstruction/PointStore.h>
#include <reconstruction/PrimitiveDetector.h>

#include <QString>

// Translation layer between the raw vertices the PLY reader produces and the
// observation store the reconstruction module consumes. It lives in the client
// so that neither module has to know the other's data types.
class PointCloudAdapter
{
public:
    // Builds the immutable observation store. Vertices that are not finite are
    // dropped here: the store rejects them, and a PLY written by an external
    // tool may legitimately carry them. Ids are allocated densely from 1.
    static reconstruction::PointStore toPointStore(const PlyCloud& cloud,
                                                   reconstruction::LengthUnit unit);

    // Reads the reconstruction manifest beside a generated cloud. Directly
    // opened files without metadata remain arbitrary-unit rather than being
    // silently interpreted as millimetres.
    static reconstruction::LengthUnit lengthUnitFor(const QString& pointCloudPath);

    // Scale-relative defaults keep preprocessing useful for both metric and
    // arbitrary-unit clouds. Known metric input is converted to millimetres by
    // the core preprocessor.
    static reconstruction::PointCloudPreprocessingOptions preprocessingOptionsFor(
        const PlyCloud& cloud);

    // Plane-detection tolerances for a cloud whose scale is unknown. A fixed
    // millimetre threshold is meaningless for an arbitrary-unit COLMAP cloud,
    // so the distance threshold follows the cloud's own diagonal instead.
    static reconstruction::PlaneDetectionOptions planeOptionsFor(const PlyCloud& cloud);
    static reconstruction::PlaneDetectionOptions planeOptionsFor(
        const reconstruction::PointStore& cloud);
    static reconstruction::CylinderDetectionOptions cylinderOptionsFor(
        const reconstruction::PointStore& cloud);
};
