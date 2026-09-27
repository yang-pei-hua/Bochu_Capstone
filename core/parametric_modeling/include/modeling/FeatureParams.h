#pragma once

#include "modeling/FaceReference.h"
#include "modeling/Sketch.h"

#include <variant>
#include <vector>

namespace modeling {

struct SketchFeatureParams {
    SketchPlaneReference plane = DatumPlaneReference{DatumPlane::XY};
    std::vector<SketchEntity> entities;
};

enum class ExtrudeOperation {
    NewBody,
    Join,
    Cut,
    Intersect,
};

struct ExtrudeFeatureParams {
    FeatureId sketchId = kInvalidFeatureId;
    double depth = 0.0;
    bool reverse = false;
    ExtrudeOperation operation = ExtrudeOperation::NewBody;
};

struct CutFeatureParams {
    FeatureId sketchId = kInvalidFeatureId;
    double depth = 0.0;
    bool throughAll = false;
    bool reverse = false;
};

using FeatureParams = std::variant<
    SketchFeatureParams,
    ExtrudeFeatureParams,
    CutFeatureParams>;

}  // namespace modeling
