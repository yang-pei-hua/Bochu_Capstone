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
};

using ModelCommand = std::variant<
    AddFeatureCommand,
    EditFeatureCommand,
    RemoveFeatureCommand>;

}  // namespace modeling
