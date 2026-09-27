#include "reconstruction/PointStore.h"

#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace reconstruction {
namespace {

bool finite(const modeling::Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
        std::isfinite(value.z);
}

}  // namespace

PointStore::PointStore(std::vector<PointSample> points, LengthUnit unit)
    : points_(std::move(points)), unit_(unit) {
    std::unordered_set<PointId> ids;
    ids.reserve(points_.size());
    for (const PointSample& point : points_) {
        if (point.id == kInvalidPointId) {
            throw std::invalid_argument("PointStore point IDs must be non-zero");
        }
        if (!ids.insert(point.id).second) {
            throw std::invalid_argument("PointStore point IDs must be unique");
        }
        if (!finite(point.position) ||
            (point.normal.has_value() && !finite(*point.normal)) ||
            !std::isfinite(point.confidence) || point.confidence < 0.0 ||
            point.confidence > 1.0) {
            throw std::invalid_argument(
                "PointStore observations must be finite with confidence in [0, 1]");
        }
    }
}

const std::vector<PointSample>& PointStore::points() const noexcept {
    return points_;
}

LengthUnit PointStore::unit() const noexcept {
    return unit_;
}

bool PointStore::empty() const noexcept {
    return points_.empty();
}

std::size_t PointStore::size() const noexcept {
    return points_.size();
}

}  // namespace reconstruction
