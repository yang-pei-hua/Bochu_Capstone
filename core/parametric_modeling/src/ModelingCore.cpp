#include "modeling/ModelingCore.h"

#include <type_traits>

namespace modeling {

ModelResult ModelingCore::execute(const ModelCommand& command) {
    return std::visit(
        [this](const auto& concreteCommand) -> ModelResult {
            using Command = std::decay_t<decltype(concreteCommand)>;
            if constexpr (std::is_same_v<Command, AddFeatureCommand>) {
                const FeatureId id = document_.addFeature(concreteCommand.params);
                const bool success = id != kInvalidFeatureId;
                return {success, id, success ? std::string{} : document_.lastError()};
            } else if constexpr (std::is_same_v<Command, EditFeatureCommand>) {
                const bool success = document_.editFeature(
                    concreteCommand.id, concreteCommand.params);
                return {success, concreteCommand.id,
                        success ? std::string{} : document_.lastError()};
            } else if constexpr (std::is_same_v<Command, RemoveFeatureCommand>) {
                const bool success = document_.removeFeature(
                    concreteCommand.id, concreteCommand.cascade);
                return {success, concreteCommand.id,
                        success ? std::string{} : document_.lastError()};
            } else if constexpr (std::is_same_v<Command, AddSketchEntityCommand>) {
                const SketchEntityId entityId = document_.addSketchEntity(
                    concreteCommand.sketchId, concreteCommand.entity);
                const bool success = entityId != kInvalidSketchEntityId;
                return {success, concreteCommand.sketchId,
                        success ? std::string{} : document_.lastError(), entityId};
            } else if constexpr (std::is_same_v<Command, EditSketchEntityCommand>) {
                const bool success = document_.editSketchEntity(
                    concreteCommand.sketchId,
                    concreteCommand.entityId,
                    concreteCommand.geometry);
                return {success, concreteCommand.sketchId,
                        success ? std::string{} : document_.lastError(),
                        concreteCommand.entityId};
            } else if constexpr (std::is_same_v<Command, RemoveSketchEntityCommand>) {
                const bool success = document_.removeSketchEntity(
                    concreteCommand.sketchId, concreteCommand.entityId);
                return {success, concreteCommand.sketchId,
                        success ? std::string{} : document_.lastError(),
                        concreteCommand.entityId};
            } else {
                const bool success = document_.setFeatureSuppressed(
                    concreteCommand.id, concreteCommand.suppressed);
                return {success, concreteCommand.id,
                        success ? std::string{} : document_.lastError()};
            }
        },
        command);
}

PartDocument& ModelingCore::document() noexcept {
    return document_;
}

const PartDocument& ModelingCore::document() const noexcept {
    return document_;
}

}  // namespace modeling
