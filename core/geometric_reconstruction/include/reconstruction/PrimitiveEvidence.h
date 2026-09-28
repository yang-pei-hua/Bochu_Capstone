#pragma once

#include "reconstruction/SurfaceEvidence.h"

#include <type_traits>
#include <variant>

namespace reconstruction {

using PrimitiveEvidence = std::variant<
    PlaneEvidence,
    CylinderEvidence,
    SphereEvidence,
    ConeEvidence,
    TorusEvidence>;

enum class PrimitiveKind {
    Plane,
    Cylinder,
    Sphere,
    Cone,
    Torus,
};

inline PrimitiveKind primitiveKind(const PrimitiveEvidence& evidence) noexcept {
    return std::visit(
        [](const auto& value) {
            using Evidence = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Evidence, PlaneEvidence>) {
                return PrimitiveKind::Plane;
            } else if constexpr (std::is_same_v<Evidence, CylinderEvidence>) {
                return PrimitiveKind::Cylinder;
            } else if constexpr (std::is_same_v<Evidence, SphereEvidence>) {
                return PrimitiveKind::Sphere;
            } else if constexpr (std::is_same_v<Evidence, ConeEvidence>) {
                return PrimitiveKind::Cone;
            } else {
                return PrimitiveKind::Torus;
            }
        },
        evidence);
}

}  // namespace reconstruction
