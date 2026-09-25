#include "modeling/PartDocument.h"

#include "RebuildEngine.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace modeling {
namespace {

std::string featurePrefix(FeatureType type) {
    switch (type) {
    case FeatureType::Sketch:
        return "Sketch";
    case FeatureType::Extrude:
        return "Extrude";
    case FeatureType::Cut:
        return "Cut";
    }
    return "Feature";
}

std::string makeFeatureName(
    const std::vector<Feature>& features,
    FeatureType type) {
    const auto count = static_cast<unsigned>(std::count_if(
        features.begin(), features.end(),
        [type](const Feature& feature) { return feature.type == type; })) + 1U;

    std::ostringstream stream;
    stream << featurePrefix(type) << std::setfill('0') << std::setw(3) << count;
    return stream.str();
}

}  // namespace

FeatureId PartDocument::addFeature(const FeatureParams& params) {
    const FeatureType type = featureTypeOf(params);
    const FeatureId id = nextFeatureId_++;
    features_.push_back(Feature{
        id,
        makeFeatureName(features_, type),
        type,
        params,
        false,
        true,
        {},
    });
    lastError_.clear();
    return id;
}

bool PartDocument::editFeature(FeatureId id, const FeatureParams& params) {
    auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    if (iterator == features_.end()) {
        lastError_ = "Feature " + std::to_string(id) + " was not found";
        return false;
    }
    if (iterator->type != featureTypeOf(params)) {
        lastError_ = "Feature type cannot be changed by editFeature";
        return false;
    }

    iterator->params = params;
    iterator->valid = true;
    iterator->errorText.clear();
    lastError_.clear();
    return true;
}

bool PartDocument::removeFeature(FeatureId id) {
    const auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    if (iterator == features_.end()) {
        lastError_ = "Feature " + std::to_string(id) + " was not found";
        return false;
    }

    features_.erase(iterator);
    lastError_.clear();
    return true;
}

bool PartDocument::setFeatureSuppressed(FeatureId id, bool suppressed) {
    auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    if (iterator == features_.end()) {
        lastError_ = "Feature " + std::to_string(id) + " was not found";
        return false;
    }
    iterator->suppressed = suppressed;
    iterator->valid = true;
    iterator->errorText.clear();
    lastError_.clear();
    return true;
}

bool PartDocument::rebuild() {
    return RebuildEngine{}.rebuild(*this);
}

const std::vector<Feature>& PartDocument::features() const noexcept {
    return features_;
}

const TopoDS_Shape& PartDocument::bodyShape() const noexcept {
    return bodyShape_;
}

const std::string& PartDocument::lastError() const noexcept {
    return lastError_;
}

const Feature* PartDocument::findFeature(FeatureId id) const noexcept {
    const auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    return iterator == features_.end() ? nullptr : &*iterator;
}

}  // namespace modeling
