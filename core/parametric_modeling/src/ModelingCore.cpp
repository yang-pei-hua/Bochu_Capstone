#include "modeling/ModelingCore.h"

#include <type_traits>

namespace modeling {

ModelResult ModelingCore::execute(const ModelCommand& command) {
    return std::visit(
        [this](const auto& concreteCommand) -> ModelResult {
            using Command = std::decay_t<decltype(concreteCommand)>;
            if constexpr (std::is_same_v<Command, AddFeatureCommand>) {
                const FeatureId id = document_.addFeature(concreteCommand.params);
                return {true, id, {}};
            } else if constexpr (std::is_same_v<Command, EditFeatureCommand>) {
                const bool success = document_.editFeature(
                    concreteCommand.id, concreteCommand.params);
                return {success, concreteCommand.id,
                        success ? std::string{} : document_.lastError()};
            } else {
                const bool success = document_.removeFeature(concreteCommand.id);
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
