#pragma once

#include <modeling/Id.h>

namespace demo {

// Feature and sketch-entity identifiers of the built-in demo model. They are
// the only model state the front end keeps; everything else is queried from
// modeling::ModelingCore.
struct DemoModelIds {
    modeling::FeatureId baseSketch = modeling::kInvalidFeatureId;
    modeling::FeatureId extrude = modeling::kInvalidFeatureId;
    modeling::FeatureId holeSketch = modeling::kInvalidFeatureId;
    modeling::FeatureId cut = modeling::kInvalidFeatureId;

    bool isComplete() const noexcept
    {
        return baseSketch != modeling::kInvalidFeatureId
            && extrude != modeling::kInvalidFeatureId
            && holeSketch != modeling::kInvalidFeatureId
            && cut != modeling::kInvalidFeatureId;
    }
};

inline constexpr modeling::SketchEntityId kBaseRectangleId = 1;
inline constexpr modeling::SketchEntityId kHoleCircleId = 2;

}  // namespace demo
