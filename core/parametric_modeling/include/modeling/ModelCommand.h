#pragma once

#include "modeling/FeatureParams.h"

#include <variant>

namespace modeling {

struct AddFeatureCommand {
    FeatureParams params;
};

struct EditFeatureCommand {
    FeatureId id = kInvalidFeatureId;
    FeatureParams params;
};

struct RemoveFeatureCommand {
    FeatureId id = kInvalidFeatureId;
    bool cascade = false;
};

struct AddSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntity entity;
};

struct EditSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntityId entityId = kInvalidSketchEntityId;
    SketchGeometry geometry;
};

struct RemoveSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntityId entityId = kInvalidSketchEntityId;
};

struct SetFeatureSuppressedCommand {
    FeatureId id = kInvalidFeatureId;
    bool suppressed = false;
};

using ModelCommand = std::variant<
    AddFeatureCommand,
    EditFeatureCommand,
    RemoveFeatureCommand,
    AddSketchEntityCommand,
    EditSketchEntityCommand,
    RemoveSketchEntityCommand,
    SetFeatureSuppressedCommand>;

}  // namespace modeling
