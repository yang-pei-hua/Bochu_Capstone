#pragma once

#include "modeling/ModelCommand.h"
#include "modeling/ModelPatch.h"
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
    ModelPatchResult apply(const ModelPatch& patch);

    bool rebuild();
    ModelRevision revision() const noexcept;
    const std::vector<Feature>& features() const noexcept;
    const TopoDS_Shape& bodyShape() const noexcept;
    const std::string& lastError() const noexcept;

    // Compatibility escape hatch for the existing client. New integrations
    // should use execute/apply and the read-only accessors above.
    PartDocument& document() noexcept;
    const PartDocument& document() const noexcept;

private:
    ModelResult executeOne(const ModelCommand& command);

    PartDocument document_;
    ModelRevision revision_ = 0;
};

}  // namespace modeling
