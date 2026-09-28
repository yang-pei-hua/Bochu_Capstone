#include "modeling/PartDocument.h"

#include "RebuildEngine.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>

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
    case FeatureType::BoxPrimitive:
        return "Box";
    case FeatureType::ThroughHolePrimitive:
        return "ThroughHole";
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

bool validateSketchEntityIds(
    const SketchFeatureParams& params,
    std::string& error) {
    std::unordered_set<SketchEntityId> ids;
    for (const SketchEntity& entity : params.entities) {
        if (entity.id == kInvalidSketchEntityId) {
            error = "Sketch entity IDs must be non-zero";
            return false;
        }
        if (!ids.insert(entity.id).second) {
            error = "Duplicate sketch entity ID " + std::to_string(entity.id);
            return false;
        }
    }
    return true;
}

bool validateFeatureParams(const FeatureParams& params, std::string& error) {
    if (const auto* sketch = std::get_if<SketchFeatureParams>(&params)) {
        return validateSketchEntityIds(*sketch, error);
    }
    const auto finite = [](const Vec3& value) {
        return std::isfinite(value.x) && std::isfinite(value.y) &&
            std::isfinite(value.z);
    };
    if (const auto* hole = std::get_if<ThroughHolePrimitiveParams>(&params)) {
        if (!finite(hole->axisPoint) || !finite(hole->axisDirection) ||
            !std::isfinite(hole->radius)) {
            error = "Through-hole parameters must be finite";
            return false;
        }
        const double directionLengthSquared =
            hole->axisDirection.x * hole->axisDirection.x +
            hole->axisDirection.y * hole->axisDirection.y +
            hole->axisDirection.z * hole->axisDirection.z;
        if (directionLengthSquared <= 1.0e-24 || hole->radius <= 0.0) {
            error = "Through-hole axis must be non-degenerate and radius must be positive";
            return false;
        }
        return true;
    }

    const auto* box = std::get_if<BoxPrimitiveParams>(&params);
    if (box == nullptr) {
        return true;
    }
    if (!finite(box->pose.origin) || !finite(box->pose.xDirection) ||
        !finite(box->pose.zDirection) || !std::isfinite(box->sizeX) ||
        !std::isfinite(box->sizeY) || !std::isfinite(box->sizeZ)) {
        error = "Box primitive parameters must be finite";
        return false;
    }
    if (box->sizeX <= 0.0 || box->sizeY <= 0.0 || box->sizeZ <= 0.0) {
        error = "Box primitive dimensions must be positive";
        return false;
    }

    const double xLengthSquared =
        box->pose.xDirection.x * box->pose.xDirection.x +
        box->pose.xDirection.y * box->pose.xDirection.y +
        box->pose.xDirection.z * box->pose.xDirection.z;
    const double zLengthSquared =
        box->pose.zDirection.x * box->pose.zDirection.x +
        box->pose.zDirection.y * box->pose.zDirection.y +
        box->pose.zDirection.z * box->pose.zDirection.z;
    const double dot =
        box->pose.xDirection.x * box->pose.zDirection.x +
        box->pose.xDirection.y * box->pose.zDirection.y +
        box->pose.xDirection.z * box->pose.zDirection.z;
    constexpr double kDegenerate = 1.0e-24;
    constexpr double kOrthogonalTolerance = 1.0e-9;
    if (xLengthSquared <= kDegenerate || zLengthSquared <= kDegenerate ||
        std::abs(dot) > kOrthogonalTolerance *
            std::sqrt(xLengthSquared * zLengthSquared)) {
        error = "Box primitive axes must be non-degenerate and orthogonal";
        return false;
    }
    return true;
}

bool dependsOn(const Feature& feature, FeatureId upstreamId) {
    if (const auto* sketch = std::get_if<SketchFeatureParams>(&feature.params)) {
        const auto* face = std::get_if<FaceReference>(&sketch->plane);
        return face != nullptr && face->ownerFeature == upstreamId;
    }
    if (const auto* extrude = std::get_if<ExtrudeFeatureParams>(&feature.params)) {
        return extrude->sketchId == upstreamId;
    }
    const auto* cut = std::get_if<CutFeatureParams>(&feature.params);
    return cut != nullptr && cut->sketchId == upstreamId;
}

}  // namespace

FeatureId PartDocument::addFeature(const FeatureParams& params) {
    std::string validationError;
    if (!validateFeatureParams(params, validationError)) {
        lastError_ = validationError;
        return kInvalidFeatureId;
    }

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
    std::string validationError;
    if (!validateFeatureParams(params, validationError)) {
        lastError_ = validationError;
        return false;
    }

    iterator->params = params;
    iterator->valid = true;
    iterator->errorText.clear();
    lastError_.clear();
    return true;
}

bool PartDocument::removeFeature(FeatureId id, bool cascade) {
    const auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    if (iterator == features_.end()) {
        lastError_ = "Feature " + std::to_string(id) + " was not found";
        return false;
    }

    const std::vector<FeatureId> directDependents = dependentsOf(id);
    if (!cascade && !directDependents.empty()) {
        lastError_ = "Feature " + std::to_string(id) + " has " +
            std::to_string(directDependents.size()) + " dependent feature(s)";
        return false;
    }

    std::unordered_set<FeatureId> removals{id};
    if (cascade) {
        std::vector<FeatureId> pending{id};
        while (!pending.empty()) {
            const FeatureId upstream = pending.back();
            pending.pop_back();
            for (const FeatureId dependent : dependentsOf(upstream)) {
                if (removals.insert(dependent).second) {
                    pending.push_back(dependent);
                }
            }
        }
    }

    features_.erase(
        std::remove_if(
            features_.begin(), features_.end(),
            [&removals](const Feature& feature) {
                return removals.find(feature.id) != removals.end();
            }),
        features_.end());
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

SketchEntityId PartDocument::addSketchEntity(
    FeatureId sketchId,
    const SketchEntity& requestedEntity) {
    Feature* feature = findFeatureMutable(sketchId);
    if (feature == nullptr || feature->type != FeatureType::Sketch) {
        lastError_ = "Sketch feature " + std::to_string(sketchId) + " was not found";
        return kInvalidSketchEntityId;
    }
    auto* params = std::get_if<SketchFeatureParams>(&feature->params);
    if (params == nullptr) {
        lastError_ = "Feature parameter type does not match Sketch";
        return kInvalidSketchEntityId;
    }

    SketchEntity entity = requestedEntity;
    if (entity.id == kInvalidSketchEntityId) {
        SketchEntityId maximum = 0;
        for (const SketchEntity& existing : params->entities) {
            maximum = std::max(maximum, existing.id);
        }
        if (maximum == std::numeric_limits<SketchEntityId>::max()) {
            lastError_ = "Sketch entity ID space is exhausted";
            return kInvalidSketchEntityId;
        }
        entity.id = maximum + 1;
    } else {
        const auto duplicate = std::find_if(
            params->entities.begin(), params->entities.end(),
            [&entity](const SketchEntity& existing) { return existing.id == entity.id; });
        if (duplicate != params->entities.end()) {
            lastError_ = "Duplicate sketch entity ID " + std::to_string(entity.id);
            return kInvalidSketchEntityId;
        }
    }

    params->entities.push_back(entity);
    feature->valid = true;
    feature->errorText.clear();
    lastError_.clear();
    return entity.id;
}

bool PartDocument::editSketchEntity(
    FeatureId sketchId,
    SketchEntityId entityId,
    const SketchGeometry& geometry) {
    Feature* feature = findFeatureMutable(sketchId);
    if (feature == nullptr || feature->type != FeatureType::Sketch) {
        lastError_ = "Sketch feature " + std::to_string(sketchId) + " was not found";
        return false;
    }
    auto* params = std::get_if<SketchFeatureParams>(&feature->params);
    if (params == nullptr) {
        lastError_ = "Feature parameter type does not match Sketch";
        return false;
    }
    const auto iterator = std::find_if(
        params->entities.begin(), params->entities.end(),
        [entityId](const SketchEntity& entity) { return entity.id == entityId; });
    if (iterator == params->entities.end()) {
        lastError_ = "Sketch entity " + std::to_string(entityId) + " was not found";
        return false;
    }
    iterator->geometry = geometry;
    feature->valid = true;
    feature->errorText.clear();
    lastError_.clear();
    return true;
}

bool PartDocument::removeSketchEntity(FeatureId sketchId, SketchEntityId entityId) {
    Feature* feature = findFeatureMutable(sketchId);
    if (feature == nullptr || feature->type != FeatureType::Sketch) {
        lastError_ = "Sketch feature " + std::to_string(sketchId) + " was not found";
        return false;
    }
    auto* params = std::get_if<SketchFeatureParams>(&feature->params);
    if (params == nullptr) {
        lastError_ = "Feature parameter type does not match Sketch";
        return false;
    }
    const auto iterator = std::find_if(
        params->entities.begin(), params->entities.end(),
        [entityId](const SketchEntity& entity) { return entity.id == entityId; });
    if (iterator == params->entities.end()) {
        lastError_ = "Sketch entity " + std::to_string(entityId) + " was not found";
        return false;
    }
    params->entities.erase(iterator);
    feature->valid = true;
    feature->errorText.clear();
    lastError_.clear();
    return true;
}

std::vector<FeatureId> PartDocument::dependentsOf(FeatureId id) const {
    std::vector<FeatureId> result;
    for (const Feature& feature : features_) {
        if (dependsOn(feature, id)) {
            result.push_back(feature.id);
        }
    }
    return result;
}

void PartDocument::clear() {
    features_.clear();
    bodyShape_.Nullify();
    nextFeatureId_ = 1;
    lastError_.clear();
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

Feature* PartDocument::findFeatureMutable(FeatureId id) noexcept {
    const auto iterator = std::find_if(
        features_.begin(), features_.end(),
        [id](const Feature& feature) { return feature.id == id; });
    return iterator == features_.end() ? nullptr : &*iterator;
}

}  // namespace modeling
