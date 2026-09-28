#pragma once

#include "io/PlyReader.h"

#include <reconstruction/PlaneDetector.h>
#include <reconstruction/PointStore.h>

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

    // Plane-detection tolerances for a cloud whose scale is unknown. A fixed
    // millimetre threshold is meaningless for an arbitrary-unit COLMAP cloud,
    // so the distance threshold follows the cloud's own diagonal instead.
    static reconstruction::PlaneDetectionOptions planeOptionsFor(const PlyCloud& cloud);
};