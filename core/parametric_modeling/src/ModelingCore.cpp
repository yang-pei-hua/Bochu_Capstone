#include "modeling/ModelingCore.h"

#include <type_traits>
#include <utility>

namespace modeling {

ModelResult ModelingCore::execute(const ModelCommand& command) {
    ModelResult result = executeOne(command);
    if (result.success) {
        ++revision_;
    }
    return result;
}

ModelResult ModelingCore::executeOne(const ModelCommand& command) {
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

ModelPatchResult ModelingCore::apply(const ModelPatch& patch) {
    if (patch.expectedRevision != kAnyModelRevision &&
        patch.expectedRevision != revision_) {
        return {
            false,
            revision_,
            {},
            "Model revision mismatch: expected " +
                std::to_string(patch.expectedRevision) + ", current " +
                std::to_string(revision_),
        };
    }

    const PartDocument original = document_;
    std::vector<FeatureId> affected;
    affected.reserve(patch.commands.size());
    for (const ModelCommand& command : patch.commands) {
        const ModelResult result = executeOne(command);
        if (!result.success) {
            document_ = original;
            return {false, revision_, {}, result.error};
        }
        affected.push_back(result.featureId);
    }

    if (patch.rebuild && !document_.rebuild()) {
        const std::string error = document_.lastError();
        document_ = original;
        return {false, revision_, {}, error};
    }

    if (!patch.commands.empty()) {
        ++revision_;
    }
    return {true, revision_, std::move(affected), {}};
}

bool ModelingCore::rebuild() {
    return document_.rebuild();
}

ModelRevision ModelingCore::revision() const noexcept {
    return revision_;
}

const std::vector<Feature>& ModelingCore::features() const noexcept {
    return document_.features();
}

const TopoDS_Shape& ModelingCore::bodyShape() const noexcept {
    return document_.bodyShape();
}

const std::string& ModelingCore::lastError() const noexcept {
    return document_.lastError();
}

PartDocument& ModelingCore::document() noexcept {
    return document_;
}

const PartDocument& ModelingCore::document() const noexcept {
    return document_;
}

}  // namespace modeling
