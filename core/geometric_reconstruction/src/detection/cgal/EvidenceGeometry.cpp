#include "detail/RansacEvidence.h"

namespace reconstruction::detail::cgal {
bool duplicateCylinder(const CylinderEvidence& left, const CylinderEvidence& right,
                       const CylinderDetectionOptions& options) {
    if (std::abs(math3::dot(left.cylinder.axisDirection, right.cylinder.axisDirection)) <
        std::cos(kPi / 90.0)) {
        return false;
    }
    const auto offset = math3::subtract(left.cylinder.axisPoint, right.cylinder.axisPoint);
    const auto radial = math3::subtract(
        offset, math3::scale(left.cylinder.axisDirection,
                            math3::dot(offset, left.cylinder.axisDirection)));
    if (math3::length(radial) > 3.0 * options.distanceThreshold ||
        std::abs(left.cylinder.radius - right.cylinder.radius) >
            3.0 * options.distanceThreshold) {
        return false;
    }
    // Geometry alone could merge two nearby real holes. Require the proposals
    // to explain the same source observations after full-cloud reassessment.
    const std::unordered_set<PointId> ids(left.supportPointIds.begin(), left.supportPointIds.end());
    std::size_t shared = 0;
    for (const auto id : right.supportPointIds) {
        shared += ids.count(id);
    }
    return double(shared) >= 0.7 * double(std::min(left.supportPointIds.size(),
                                                 right.supportPointIds.size()));
}

modeling::Vec3 toVec3(const CgalPoint &point) {
    return {
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z()),
    };
}

modeling::Vec3 toVec3(const CgalVector& vector) {
    return {
        CGAL::to_double(vector.x()),
        CGAL::to_double(vector.y()),
        CGAL::to_double(vector.z()),
    };
}

double boundingBoxDiagonal(const PointStore& points) {
    if (points.empty()) {
        return 0.0;
    }
    modeling::Vec3 lower = points.points().front().position;
    modeling::Vec3 upper = lower;
    for (const PointSample& point : points.points()) {
        lower.x = std::min(lower.x, point.position.x);
        lower.y = std::min(lower.y, point.position.y);
        lower.z = std::min(lower.z, point.position.z);
        upper.x = std::max(upper.x, point.position.x);
        upper.y = std::max(upper.y, point.position.y);
        upper.z = std::max(upper.z, point.position.z);
    }
    return math3::length(math3::subtract(upper, lower));
}

double planeResidual(
    const PlaneEquation& plane,
    const modeling::Vec3& point) {
    return std::abs(math3::dot(plane.normal, point) + plane.offset);
}

double angularCoverage(std::vector<double> angles) {
    if (angles.size() < 2U) {
        return 0.0;
    }
    std::sort(angles.begin(), angles.end());
    double largestGap = kTwoPi - angles.back() + angles.front();
    for (std::size_t index = 1; index < angles.size(); ++index) {
        largestGap = std::max(largestGap, angles[index] - angles[index - 1U]);
    }
    return std::clamp((kTwoPi - largestGap) / kTwoPi, 0.0, 1.0);
}

void assignId(PrimitiveEvidence& evidence, SurfaceId id) {
    std::visit([id](auto& value) { value.id = id; }, evidence);
}

void addSupportIds(
    const PrimitiveEvidence& evidence,
    std::unordered_set<PointId>& assigned) {
    std::visit(
        [&assigned](const auto& value) {
            assigned.insert(
                value.supportPointIds.begin(), value.supportPointIds.end());
        },
        evidence);
}
} // namespace reconstruction::detail::cgal
