#pragma once

#include "modeling/FeatureParams.h"
#include "modeling/Id.h"

#include <string>

namespace modeling {

enum class FeatureType {
    Sketch,
    Extrude,
    Cut,
    BoxPrimitive,
    ThroughHolePrimitive,
    SpherePrimitive,
    CylinderPrimitive,
    ConePrimitive,
    TorusPrimitive,
};

inline FeatureType featureTypeOf(const FeatureParams& params) {
    if (std::holds_alternative<SketchFeatureParams>(params)) {
        return FeatureType::Sketch;
    }
    if (std::holds_alternative<ExtrudeFeatureParams>(params)) {
        return FeatureType::Extrude;
    }
    if (std::holds_alternative<CutFeatureParams>(params)) {
        return FeatureType::Cut;
    }
    if (std::holds_alternative<BoxPrimitiveParams>(params)) {
        return FeatureType::BoxPrimitive;
    }
    if (std::holds_alternative<ThroughHolePrimitiveParams>(params)) return FeatureType::ThroughHolePrimitive;
    if (std::holds_alternative<SpherePrimitiveParams>(params)) return FeatureType::SpherePrimitive;
    if (std::holds_alternative<CylinderPrimitiveParams>(params)) return FeatureType::CylinderPrimitive;
    if (std::holds_alternative<ConePrimitiveParams>(params)) return FeatureType::ConePrimitive;
    return FeatureType::TorusPrimitive;
}

struct Feature {
    FeatureId id = kInvalidFeatureId;
    std::string name;
    FeatureType type = FeatureType::Sketch;
    FeatureParams params;
    bool suppressed = false;
    bool valid = true;
    std::string errorText;
    bool reconstructed = false;
    bool manuallyEdited = false;
    ResizeAnchor resizeAnchor = ResizeAnchor::Center;
};

}  // namespace modeling
