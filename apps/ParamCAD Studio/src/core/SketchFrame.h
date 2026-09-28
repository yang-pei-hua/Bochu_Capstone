#pragma once

#include "modeling/FaceReference.h"
#include "modeling/Feature.h"
#include "modeling/Id.h"

#include <QString>

#include <vector>

// Placement of a sketch plane in world space.
//
// The core resolves sketch planes while it rebuilds but exposes no frame to the
// front end, so the 3D preview mirrors the core's own occt::PlaneFrame
// arithmetic to land exactly where the core will build the sketch. Everything
// here is display-only: the feature history stays the single source of truth.
namespace sketchapp {

struct PlaneFrame {
    double origin[3]{0.0, 0.0, 0.0};
    double xDirection[3]{1.0, 0.0, 0.0};
    double yDirection[3]{0.0, 1.0, 0.0};
    double normal[3]{0.0, 0.0, 1.0};
};

PlaneFrame datumPlaneFrame(modeling::DatumPlane plane);

// Same rule as the core's translatedFrame(): the X axis is preserved and Y is
// derived from the normal, so X x Y = normal.
PlaneFrame translatedFrame(const PlaneFrame& source, const double normal[3],
                           double distance);

// Conversion to and from the core's explicit plane reference. A face sketch is
// captured as an explicit plane, so this is how a picked face reaches the core.
modeling::Plane3d plane3dOf(const PlaneFrame& frame);
PlaneFrame planeFrameOf(const modeling::Plane3d& plane);

// Short human-readable name for a plane reference, shown in the sketch panel.
QString planeLabel(const modeling::SketchPlaneReference& reference);

// Resolves a sketch plane reference against the feature history. Datum and
// offset planes are direct; an explicit plane is already in world coordinates;
// a face reference is resolved through the extrusion that owns it, which is why
// the whole history is needed. Returns false when the reference cannot be
// resolved (for example a deleted owner).
bool resolveSketchFrame(const std::vector<modeling::Feature>& features,
                        const modeling::SketchPlaneReference& reference,
                        PlaneFrame& frame);

}  // namespace sketchapp
