#pragma once

#include "modeling/FeatureParams.h"
#include "modeling/Id.h"

#include <string>

namespace modeling {

enum class FeatureType {
    Sketch,
    Extrude,
    Cut,
};

inline FeatureType featureTypeOf(const FeatureParams& params) {
    if (std::holds_alternative<SketchFeatureParams>(params)) {
        return FeatureType::Sketch;
    }
    if (std::holds_alternative<ExtrudeFeatureParams>(params)) {
        return FeatureType::Extrude;
    }
    return FeatureType::Cut;
}

struct Feature {
    FeatureId id = kInvalidFeatureId;
    std::string name;
    FeatureType type = FeatureType::Sketch;
    FeatureParams params;
    bool suppressed = false;
    bool valid = true;
    std::string errorText;
};

}  // namespace modeling
