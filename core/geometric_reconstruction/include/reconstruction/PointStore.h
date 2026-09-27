#pragma once

#include "modeling/Geometry.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace reconstruction {

using PointId = std::uint64_t;
inline constexpr PointId kInvalidPointId = 0;

enum class LengthUnit {
    Millimeter,
    Arbitrary,
};

struct PointSample {
    PointId id = kInvalidPointId;
    modeling::Vec3 position{};
    std::optional<modeling::Vec3> normal;
    double confidence = 1.0;
};

// Immutable source observations. Detection results reference PointId values;
// they never remove or rewrite points in this store.
class PointStore {
public:
    explicit PointStore(
        std::vector<PointSample> points,
        LengthUnit unit = LengthUnit::Millimeter);

    const std::vector<PointSample>& points() const noexcept;
    LengthUnit unit() const noexcept;
    bool empty() const noexcept;
    std::size_t size() const noexcept;

private:
    std::vector<PointSample> points_;
    LengthUnit unit_;
};

}  // namespace reconstruction
