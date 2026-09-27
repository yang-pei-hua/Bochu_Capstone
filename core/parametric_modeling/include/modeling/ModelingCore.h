#pragma once

#include "modeling/ModelCommand.h"
#include "modeling/PartDocument.h"

#include <string>

namespace modeling {

struct ModelResult {
    bool success = false;
    FeatureId featureId = kInvalidFeatureId;
    std::string error;
    SketchEntityId sketchEntityId = kInvalidSketchEntityId;
};

class ModelingCore {
public:
    ModelResult execute(const ModelCommand& command);

    PartDocument& document() noexcept;
    const PartDocument& document() const noexcept;

private:
    PartDocument document_;
};

}  // namespace modeling
