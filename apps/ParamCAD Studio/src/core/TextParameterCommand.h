#pragma once
#include <modeling/Feature.h>
#include <modeling/ModelPatch.h>
#include <QString>
#include <array>

namespace textcad {
struct PrimitiveEdit {
    modeling::FeatureId featureId = modeling::kInvalidFeatureId;
    modeling::ModelRevision expectedRevision = 0;
    std::array<double, 3> values{1, 1, 1};
    modeling::ResizeAnchor anchor = modeling::ResizeAnchor::Center;
    QString summary;
};

// Entire input must match a bounded grammar. This only prepares a command;
// callers validate the full dependency rebuild before presenting it for apply.
bool parsePrimitiveEdit(const QString& input, const std::vector<modeling::Feature>& features,
    modeling::FeatureId selected, modeling::ModelRevision revision,
    PrimitiveEdit& output, QString& error);
}
