#include "reconstruction/data/PointStore.h"

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

PointStore normalize(PointStore source, double scale) {
    if (scale == 1.0) return source;
    auto points = source.points();
    for (auto& point : points) {
        point.position.x *= scale;
        point.position.y *= scale;
        point.position.z *= scale;
    }
    return PointStore(std::move(points), LengthUnit::Millimeter);
}

}  // namespace

std::optional<double> millimetersPerUnit(LengthUnit unit) noexcept {
    switch (unit) {
    case LengthUnit::Millimeter:
        return 1.0;
    case LengthUnit::Meter:
        return 1000.0;
    case LengthUnit::Arbitrary:
        return std::nullopt;
    }
    return std::nullopt;
}

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
            point.confidence > 1.0 ||
            (point.curvature.has_value() &&
             (!std::isfinite(*point.curvature) || *point.curvature < 0.0))) {
            throw std::invalid_argument(
                "PointStore observations must be finite with confidence in [0, 1] "
                "and non-negative curvature");
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

std::uint64_t pointStoreFingerprint(const PointStore& points) noexcept {
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](const auto& value) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(value); ++i) h = (h ^ bytes[i])*1099511628211ULL;
    };
    const auto unit = static_cast<int>(points.unit()); mix(unit);
    for (const auto& p : points.points()) {
        mix(p.id); mix(p.position.x); mix(p.position.y); mix(p.position.z);
        const bool has = p.normal.has_value(); mix(has); mix(p.normalOrientationKnown);
        if (p.normal) { mix(p.normal->x); mix(p.normal->y); mix(p.normal->z); }
        mix(p.confidence);
    }
    return h;
}

ObservationSnapshot::ObservationSnapshot(PointStore source, bool convertKnownUnits)
    : sourceUnit_(source.unit()),
      scale_(convertKnownUnits ? millimetersPerUnit(sourceUnit_).value_or(1.0) : 1.0),
      observations_(normalize(std::move(source), scale_)) {}

}  // namespace reconstruction
