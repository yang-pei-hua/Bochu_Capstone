#include "reconstruction/CgalEfficientRansacDetector.h"

#include "Math3.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cone.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cylinder.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Efficient_RANSAC_traits.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Plane.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Sphere.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Torus.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace reconstruction {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint = Kernel::Point_3;
using CgalVector = Kernel::Vector_3;

using InputPoint = std::tuple<CgalPoint, CgalVector, PointId>;
using InputRange = std::vector<InputPoint>;
using InputPointMap = CGAL::Nth_of_tuple_property_map<0, InputPoint>;
using InputNormalMap = CGAL::Nth_of_tuple_property_map<1, InputPoint>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<
    Kernel, InputRange, InputPointMap, InputNormalMap>;
using Ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using CgalPlane = CGAL::Shape_detection::Plane<Traits>;
using CgalCylinder = CGAL::Shape_detection::Cylinder<Traits>;
using CgalSphere = CGAL::Shape_detection::Sphere<Traits>;
using CgalCone = CGAL::Shape_detection::Cone<Traits>;
using CgalTorus = CGAL::Shape_detection::Torus<Traits>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

modeling::Vec3 toVec3(const CgalPoint& point) {
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

bool validPlaneOptions(const PlaneDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        options.minimumSupportPoints >= 3U && options.maximumPlanes > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.duplicatePlaneAngularToleranceRadians) &&
        options.duplicatePlaneAngularToleranceRadians > 0.0 &&
        options.duplicatePlaneAngularToleranceRadians < kPi / 2.0 &&
        std::isfinite(options.duplicatePlaneDistanceMultiplier) &&
        options.duplicatePlaneDistanceMultiplier >= 1.0;
}

bool validCylinderOptions(const CylinderDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 2U &&
        options.maximumCylinders > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumRadius) &&
        options.minimumRadius >= 0.0 &&
        options.maximumRadius > options.minimumRadius;
}

bool validSphereOptions(const SphereDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 3U && options.maximumSpheres > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumRadius) &&
        options.minimumRadius >= 0.0 &&
        options.maximumRadius > options.minimumRadius;
}

bool validConeOptions(const ConeDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 3U && options.maximumCones > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumOpeningAngleRadians) &&
        std::isfinite(options.maximumOpeningAngleRadians) &&
        options.minimumOpeningAngleRadians > 0.0 &&
        options.maximumOpeningAngleRadians < kPi / 2.0 &&
        options.maximumOpeningAngleRadians >
            options.minimumOpeningAngleRadians;
}

bool validTorusOptions(const TorusDetectionOptions& options) {
    return std::isfinite(options.distanceThreshold) &&
        options.distanceThreshold > 0.0 &&
        std::isfinite(options.minimumNormalAlignment) &&
        options.minimumNormalAlignment >= 0.0 &&
        options.minimumNormalAlignment <= 1.0 &&
        options.minimumSupportPoints >= 4U && options.maximumTori > 0U &&
        std::isfinite(options.probability) && options.probability >= 0.0 &&
        options.probability <= 1.0 &&
        std::isfinite(options.clusterEpsilon) &&
        options.clusterEpsilon >= 0.0 &&
        std::isfinite(options.minimumMajorRadius) &&
        options.minimumMajorRadius >= 0.0 &&
        options.maximumMajorRadius > options.minimumMajorRadius &&
        std::isfinite(options.minimumMinorRadius) &&
        options.minimumMinorRadius >= 0.0 &&
        options.maximumMinorRadius > options.minimumMinorRadius;
}

PlaneEquation fitPlane(
    const InputRange& input,
    const std::vector<std::size_t>& indices) {
    modeling::Vec3 centroid{};
    for (const std::size_t index : indices) {
        centroid = math3::add(centroid, toVec3(std::get<0>(input[index])));
    }
    centroid = math3::scale(
        centroid, 1.0 / static_cast<double>(indices.size()));

    std::array<std::array<double, 3>, 3> covariance{};
    for (const std::size_t index : indices) {
        const modeling::Vec3 delta = math3::subtract(
            toVec3(std::get<0>(input[index])), centroid);
        const std::array<double, 3> value{delta.x, delta.y, delta.z};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                covariance[row][column] += value[row] * value[column];
            }
        }
    }
    const modeling::Vec3 normal = math3::smallestEigenvector(covariance);
    return {normal, -math3::dot(normal, centroid)};
}

double planeResidual(
    const PlaneEquation& plane,
    const modeling::Vec3& point) {
    return std::abs(math3::dot(plane.normal, point) + plane.offset);
}

modeling::Vec3 centroidOf(
    const InputRange& input,
    const std::vector<std::size_t>& indices) {
    modeling::Vec3 centroid{};
    for (const std::size_t index : indices) {
        centroid = math3::add(centroid, toVec3(std::get<0>(input[index])));
    }
    return math3::scale(
        centroid, 1.0 / static_cast<double>(indices.size()));
}

bool duplicatePlane(
    const PlaneEquation& candidate,
    const modeling::Vec3& candidateCentroid,
    const PlaneEquation& accepted,
    const modeling::Vec3& acceptedCentroid,
    const PlaneDetectionOptions& options) {
    const double minimumAlignment =
        std::cos(options.duplicatePlaneAngularToleranceRadians);
    if (std::abs(math3::dot(candidate.normal, accepted.normal)) <
        minimumAlignment) {
        return false;
    }
    const double tolerance = options.distanceThreshold *
        options.duplicatePlaneDistanceMultiplier;
    return planeResidual(accepted, candidateCentroid) <= tolerance &&
        planeResidual(candidate, acceptedCentroid) <= tolerance;
}

std::vector<std::size_t> filteredPlaneSupport(
    const InputRange& input,
    const std::vector<std::size_t>& candidateIndices,
    const PlaneEquation& plane,
    const PlaneDetectionOptions& options) {
    std::vector<std::size_t> result;
    result.reserve(candidateIndices.size());
    for (const std::size_t index : candidateIndices) {
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        if (planeResidual(plane, toVec3(std::get<0>(input[index]))) <=
                options.distanceThreshold &&
            std::abs(math3::dot(normal, plane.normal)) >=
                options.minimumNormalAlignment) {
            result.push_back(index);
        }
    }
    return result;
}

bool makePlaneEvidence(
    const InputRange& input,
    const CgalPlane& shape,
    const PointStore& points,
    const PlaneDetectionOptions& options,
    PlaneEvidence& evidence,
    modeling::Vec3& centroid) {
    std::vector<std::size_t> support = shape.indices_of_assigned_points();
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    PlaneEquation plane = fitPlane(input, support);
    support = filteredPlaneSupport(input, support, plane, options);
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }
    plane = fitPlane(input, support);
    support = filteredPlaneSupport(input, support, plane, options);
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    evidence = {};
    evidence.plane = plane;
    evidence.supportPointIds.reserve(support.size());
    double residualSum = 0.0;
    for (const std::size_t index : support) {
        const double residual = planeResidual(
            plane, toVec3(std::get<0>(input[index])));
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        residualSum += residual;
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(support.size());
    evidence.coverage = static_cast<double>(support.size()) /
        static_cast<double>(points.size());
    evidence.confidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0);
    centroid = centroidOf(input, support);
    return true;
}

double cylinderResidual(
    const CylinderEquation& cylinder,
    const modeling::Vec3& point,
    double* axial = nullptr,
    modeling::Vec3* radialDirection = nullptr) {
    const modeling::Vec3 offset = math3::subtract(
        point, cylinder.axisPoint);
    const double coordinate = math3::dot(offset, cylinder.axisDirection);
    const modeling::Vec3 radial = math3::subtract(
        offset, math3::scale(cylinder.axisDirection, coordinate));
    const double radialLength = math3::length(radial);
    if (axial != nullptr) {
        *axial = coordinate;
    }
    if (radialDirection != nullptr && radialLength > 1.0e-12) {
        *radialDirection = math3::scale(radial, 1.0 / radialLength);
    }
    return std::abs(radialLength - cylinder.radius);
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

bool makeCylinderEvidence(
    const InputRange& input,
    const CgalCylinder& shape,
    const PointStore& points,
    const CylinderDetectionOptions& options,
    CylinderEvidence& evidence) {
    const auto axis = shape.axis();
    modeling::Vec3 direction;
    modeling::Vec3 pointOnAxis;
    try {
        direction = math3::canonicalDirection(toVec3(axis.to_vector()));
        pointOnAxis = toVec3(axis.point(0));
    } catch (const std::invalid_argument&) {
        return false;
    }

    CylinderEquation cylinder;
    cylinder.axisDirection = direction;
    cylinder.axisPoint = math3::subtract(
        pointOnAxis,
        math3::scale(direction, math3::dot(pointOnAxis, direction)));
    cylinder.radius = CGAL::to_double(shape.radius());
    if (!std::isfinite(cylinder.radius) ||
        cylinder.radius < options.minimumRadius ||
        cylinder.radius > options.maximumRadius) {
        return false;
    }

    std::vector<std::size_t> support;
    support.reserve(shape.indices_of_assigned_points().size());
    for (const std::size_t index : shape.indices_of_assigned_points()) {
        modeling::Vec3 radialDirection{};
        const double residual = cylinderResidual(
            cylinder,
            toVec3(std::get<0>(input[index])),
            nullptr,
            &radialDirection);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        if (residual <= options.distanceThreshold &&
            std::abs(math3::dot(normal, radialDirection)) >=
                options.minimumNormalAlignment) {
            support.push_back(index);
        }
    }
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    // With the CGAL axis fixed, the least-squares radius is the mean radial
    // distance of all assigned observations rather than the two-point sample.
    double radiusSum = 0.0;
    for (const std::size_t index : support) {
        const modeling::Vec3 offset = math3::subtract(
            toVec3(std::get<0>(input[index])), cylinder.axisPoint);
        const double axial = math3::dot(offset, cylinder.axisDirection);
        radiusSum += math3::length(math3::subtract(
            offset, math3::scale(cylinder.axisDirection, axial)));
    }
    cylinder.radius = radiusSum / static_cast<double>(support.size());
    if (cylinder.radius < options.minimumRadius ||
        cylinder.radius > options.maximumRadius) {
        return false;
    }

    modeling::Vec3 basisU;
    try {
        basisU = math3::normalized(math3::cross(
            cylinder.axisDirection,
            std::abs(cylinder.axisDirection.z) < 0.9
                ? modeling::Vec3{0.0, 0.0, 1.0}
                : modeling::Vec3{1.0, 0.0, 0.0}));
    } catch (const std::invalid_argument&) {
        return false;
    }
    const modeling::Vec3 basisV = math3::normalized(
        math3::cross(cylinder.axisDirection, basisU));

    evidence = {};
    evidence.cylinder = cylinder;
    evidence.axialMinimum = std::numeric_limits<double>::infinity();
    evidence.axialMaximum = -std::numeric_limits<double>::infinity();
    evidence.supportPointIds.reserve(support.size());
    std::vector<double> angles;
    angles.reserve(support.size());
    double residualSum = 0.0;
    double alignmentSum = 0.0;
    for (const std::size_t index : support) {
        double axial = 0.0;
        modeling::Vec3 radialDirection{};
        const double residual = cylinderResidual(
            cylinder,
            toVec3(std::get<0>(input[index])),
            &axial,
            &radialDirection);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        evidence.axialMinimum = std::min(evidence.axialMinimum, axial);
        evidence.axialMaximum = std::max(evidence.axialMaximum, axial);
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
        residualSum += residual;
        alignmentSum += std::abs(math3::dot(normal, radialDirection));
        double angle = std::atan2(
            math3::dot(radialDirection, basisV),
            math3::dot(radialDirection, basisU));
        if (angle < 0.0) {
            angle += kTwoPi;
        }
        angles.push_back(angle);
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(support.size());
    evidence.angularCoverage = angularCoverage(std::move(angles));
    evidence.coverage = static_cast<double>(support.size()) /
        static_cast<double>(points.size());
    const double residualConfidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0);
    const double normalConfidence = alignmentSum /
        static_cast<double>(support.size());
    evidence.confidence = residualConfidence * normalConfidence *
        std::clamp(evidence.angularCoverage / 0.5, 0.0, 1.0);
    return true;
}

bool makeSphereEvidence(
    const InputRange& input,
    const CgalSphere& shape,
    const PointStore& points,
    const SphereDetectionOptions& options,
    SphereEvidence& evidence) {
    SphereEquation sphere;
    sphere.center = toVec3(shape.center());
    sphere.radius = CGAL::to_double(shape.radius());
    if (!std::isfinite(sphere.radius) ||
        sphere.radius < options.minimumRadius ||
        sphere.radius > options.maximumRadius) {
        return false;
    }

    std::vector<std::size_t> support;
    support.reserve(shape.indices_of_assigned_points().size());
    for (const std::size_t index : shape.indices_of_assigned_points()) {
        const modeling::Vec3 radial = math3::subtract(
            toVec3(std::get<0>(input[index])), sphere.center);
        const double distance = math3::length(radial);
        if (distance <= 1.0e-12) {
            continue;
        }
        const modeling::Vec3 radialDirection =
            math3::scale(radial, 1.0 / distance);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        if (std::abs(distance - sphere.radius) <=
                options.distanceThreshold &&
            std::abs(math3::dot(normal, radialDirection)) >=
                options.minimumNormalAlignment) {
            support.push_back(index);
        }
    }
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    double radiusSum = 0.0;
    for (const std::size_t index : support) {
        radiusSum += math3::length(math3::subtract(
            toVec3(std::get<0>(input[index])), sphere.center));
    }
    sphere.radius = radiusSum / static_cast<double>(support.size());
    if (sphere.radius < options.minimumRadius ||
        sphere.radius > options.maximumRadius) {
        return false;
    }

    evidence = {};
    evidence.sphere = sphere;
    evidence.supportPointIds.reserve(support.size());
    double residualSum = 0.0;
    double alignmentSum = 0.0;
    for (const std::size_t index : support) {
        const modeling::Vec3 radial = math3::subtract(
            toVec3(std::get<0>(input[index])), sphere.center);
        const double distance = math3::length(radial);
        const modeling::Vec3 radialDirection =
            math3::scale(radial, 1.0 / distance);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        const double residual = std::abs(distance - sphere.radius);
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        residualSum += residual;
        alignmentSum += std::abs(math3::dot(normal, radialDirection));
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(support.size());
    evidence.coverage = static_cast<double>(support.size()) /
        static_cast<double>(points.size());
    evidence.confidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0) * alignmentSum / static_cast<double>(support.size());
    return true;
}

bool makeConeEvidence(
    const InputRange& input,
    const CgalCone& shape,
    const PointStore& points,
    const ConeDetectionOptions& options,
    ConeEvidence& evidence) {
    ConeEquation cone;
    cone.apex = toVec3(shape.apex());
    cone.openingAngleRadians = CGAL::to_double(shape.angle());
    try {
        cone.axisDirection = math3::normalized(toVec3(shape.axis()));
    } catch (const std::invalid_argument&) {
        return false;
    }
    if (!std::isfinite(cone.openingAngleRadians) ||
        cone.openingAngleRadians < options.minimumOpeningAngleRadians ||
        cone.openingAngleRadians > options.maximumOpeningAngleRadians) {
        return false;
    }

    modeling::Vec3 basisU;
    try {
        basisU = math3::normalized(math3::cross(
            cone.axisDirection,
            std::abs(cone.axisDirection.z) < 0.9
                ? modeling::Vec3{0.0, 0.0, 1.0}
                : modeling::Vec3{1.0, 0.0, 0.0}));
    } catch (const std::invalid_argument&) {
        return false;
    }
    const modeling::Vec3 basisV = math3::normalized(
        math3::cross(cone.axisDirection, basisU));
    const double sine = std::sin(cone.openingAngleRadians);
    const double cosine = std::cos(cone.openingAngleRadians);

    evidence = {};
    evidence.cone = cone;
    evidence.axialMinimum = std::numeric_limits<double>::infinity();
    evidence.axialMaximum = -std::numeric_limits<double>::infinity();
    std::vector<double> angles;
    double residualSum = 0.0;
    double alignmentSum = 0.0;
    for (const std::size_t index : shape.indices_of_assigned_points()) {
        const modeling::Vec3 offset = math3::subtract(
            toVec3(std::get<0>(input[index])), cone.apex);
        const double axial = math3::dot(offset, cone.axisDirection);
        if (axial < 0.0) {
            continue;
        }
        const modeling::Vec3 radial = math3::subtract(
            offset, math3::scale(cone.axisDirection, axial));
        const double radialLength = math3::length(radial);
        if (radialLength <= 1.0e-12) {
            continue;
        }
        const modeling::Vec3 radialDirection =
            math3::scale(radial, 1.0 / radialLength);
        const modeling::Vec3 surfaceNormal = math3::subtract(
            math3::scale(radialDirection, cosine),
            math3::scale(cone.axisDirection, sine));
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        const double residual = std::abs(
            cosine * radialLength - sine * axial);
        const double alignment = std::abs(
            math3::dot(normal, surfaceNormal));
        if (residual > options.distanceThreshold ||
            alignment < options.minimumNormalAlignment) {
            continue;
        }
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        evidence.axialMinimum = std::min(evidence.axialMinimum, axial);
        evidence.axialMaximum = std::max(evidence.axialMaximum, axial);
        residualSum += residual;
        alignmentSum += alignment;
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
        double angle = std::atan2(
            math3::dot(radialDirection, basisV),
            math3::dot(radialDirection, basisU));
        if (angle < 0.0) {
            angle += kTwoPi;
        }
        angles.push_back(angle);
    }
    if (evidence.supportPointIds.size() < options.minimumSupportPoints) {
        return false;
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(evidence.supportPointIds.size());
    evidence.angularCoverage = angularCoverage(std::move(angles));
    evidence.coverage = static_cast<double>(evidence.supportPointIds.size()) /
        static_cast<double>(points.size());
    evidence.confidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0) * alignmentSum /
        static_cast<double>(evidence.supportPointIds.size()) *
        std::clamp(evidence.angularCoverage / 0.5, 0.0, 1.0);
    return true;
}

bool makeTorusEvidence(
    const InputRange& input,
    const CgalTorus& shape,
    const PointStore& points,
    const TorusDetectionOptions& options,
    TorusEvidence& evidence) {
    TorusEquation torus;
    torus.center = toVec3(shape.center());
    torus.majorRadius = CGAL::to_double(shape.major_radius());
    torus.minorRadius = CGAL::to_double(shape.minor_radius());
    try {
        torus.axisDirection = math3::canonicalDirection(toVec3(shape.axis()));
    } catch (const std::invalid_argument&) {
        return false;
    }
    if (!std::isfinite(torus.majorRadius) ||
        !std::isfinite(torus.minorRadius) ||
        torus.majorRadius < options.minimumMajorRadius ||
        torus.majorRadius > options.maximumMajorRadius ||
        torus.minorRadius < options.minimumMinorRadius ||
        torus.minorRadius > options.maximumMinorRadius) {
        return false;
    }

    evidence = {};
    evidence.torus = torus;
    double residualSum = 0.0;
    double alignmentSum = 0.0;
    for (const std::size_t index : shape.indices_of_assigned_points()) {
        const modeling::Vec3 point = toVec3(std::get<0>(input[index]));
        const modeling::Vec3 offset = math3::subtract(point, torus.center);
        const double height = math3::dot(offset, torus.axisDirection);
        const modeling::Vec3 inPlane = math3::subtract(
            offset, math3::scale(torus.axisDirection, height));
        const double axisDistance = math3::length(inPlane);
        if (axisDistance <= 1.0e-12) {
            continue;
        }
        const modeling::Vec3 ringDirection =
            math3::scale(inPlane, 1.0 / axisDistance);
        const modeling::Vec3 tubeCenter = math3::add(
            torus.center,
            math3::scale(ringDirection, torus.majorRadius));
        const modeling::Vec3 tubeVector = math3::subtract(point, tubeCenter);
        const double tubeDistance = math3::length(tubeVector);
        if (tubeDistance <= 1.0e-12) {
            continue;
        }
        const modeling::Vec3 surfaceNormal =
            math3::scale(tubeVector, 1.0 / tubeDistance);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        const double residual = std::abs(tubeDistance - torus.minorRadius);
        const double alignment = std::abs(
            math3::dot(normal, surfaceNormal));
        if (residual > options.distanceThreshold ||
            alignment < options.minimumNormalAlignment) {
            continue;
        }
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        residualSum += residual;
        alignmentSum += alignment;
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
    }
    if (evidence.supportPointIds.size() < options.minimumSupportPoints) {
        return false;
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(evidence.supportPointIds.size());
    evidence.coverage = static_cast<double>(evidence.supportPointIds.size()) /
        static_cast<double>(points.size());
    evidence.confidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0) * alignmentSum /
        static_cast<double>(evidence.supportPointIds.size());
    return true;
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

}  // namespace

bool detectWithCgalEfficientRansac(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error) {
    output = {};
    error.clear();
    if (!options.detectPlanes && !options.detectCylinders &&
        !options.detectSpheres && !options.detectCones &&
        !options.detectTori) {
        error = "At least one CGAL primitive type must be enabled";
        return false;
    }
    if ((options.detectPlanes && !validPlaneOptions(options.plane)) ||
        (options.detectCylinders &&
         !validCylinderOptions(options.cylinder)) ||
        (options.detectSpheres && !validSphereOptions(options.sphere)) ||
        (options.detectCones && !validConeOptions(options.cone)) ||
        (options.detectTori && !validTorusOptions(options.torus))) {
        error = "CGAL Efficient RANSAC options are invalid";
        return false;
    }

    InputRange input;
    input.reserve(points.size());
    for (const PointSample& point : points.points()) {
        if (!point.normal.has_value()) {
            continue;
        }
        try {
            const modeling::Vec3 normal = math3::normalized(*point.normal);
            input.push_back({
                CgalPoint(point.position.x, point.position.y, point.position.z),
                CgalVector(normal.x, normal.y, normal.z),
                point.id,
            });
        } catch (const std::invalid_argument&) {
            // A zero or non-finite normal is not a valid CGAL observation.
        }
    }

    std::size_t minimumSupport = std::numeric_limits<std::size_t>::max();
    double probability = 1.0;
    double epsilon = 0.0;
    double normalThreshold = 1.0;
    double explicitCluster = 0.0;
    const auto contribute = [&](
                                bool enabled,
                                std::size_t support,
                                double shapeProbability,
                                double shapeEpsilon,
                                double shapeNormalThreshold,
                                double shapeCluster) {
        if (!enabled) {
            return;
        }
        minimumSupport = std::min(minimumSupport, support);
        probability = std::min(probability, shapeProbability);
        epsilon = std::max(epsilon, shapeEpsilon);
        normalThreshold = std::min(
            normalThreshold, shapeNormalThreshold);
        explicitCluster = std::max(explicitCluster, shapeCluster);
    };
    contribute(
        options.detectPlanes,
        options.plane.minimumSupportPoints,
        options.plane.probability,
        options.plane.distanceThreshold,
        options.plane.minimumNormalAlignment,
        options.plane.clusterEpsilon);
    contribute(
        options.detectCylinders,
        options.cylinder.minimumSupportPoints,
        options.cylinder.probability,
        options.cylinder.distanceThreshold,
        options.cylinder.minimumNormalAlignment,
        options.cylinder.clusterEpsilon);
    contribute(
        options.detectSpheres,
        options.sphere.minimumSupportPoints,
        options.sphere.probability,
        options.sphere.distanceThreshold,
        options.sphere.minimumNormalAlignment,
        options.sphere.clusterEpsilon);
    contribute(
        options.detectCones,
        options.cone.minimumSupportPoints,
        options.cone.probability,
        options.cone.distanceThreshold,
        options.cone.minimumNormalAlignment,
        options.cone.clusterEpsilon);
    contribute(
        options.detectTori,
        options.torus.minimumSupportPoints,
        options.torus.probability,
        options.torus.distanceThreshold,
        options.torus.minimumNormalAlignment,
        options.torus.clusterEpsilon);
    if (input.size() < minimumSupport) {
        error = "CGAL Efficient RANSAC requires more points with valid normals";
        return false;
    }

    const double diagonal = boundingBoxDiagonal(points);
    Ransac::Parameters parameters;
    parameters.probability = probability;
    parameters.min_points = minimumSupport;
    parameters.epsilon = epsilon;
    parameters.normal_threshold = normalThreshold;
    // Ten percent also connects deliberately sparse mechanical-surface grids.
    // Shape distance is still governed by epsilon, so this does not merge
    // parallel depth layers. Callers with dense scenes can override it.
    parameters.cluster_epsilon = explicitCluster > 0.0
        ? explicitCluster
        : std::max(parameters.epsilon * 3.0, diagonal * 0.1);

    Ransac ransac;
    ransac.set_input(input, InputPointMap{}, InputNormalMap{});
    if (options.detectPlanes) {
        ransac.add_shape_factory<CgalPlane>();
    }
    if (options.detectCylinders) {
        ransac.add_shape_factory<CgalCylinder>();
    }
    if (options.detectSpheres) {
        ransac.add_shape_factory<CgalSphere>();
    }
    if (options.detectCones) {
        ransac.add_shape_factory<CgalCone>();
    }
    if (options.detectTori) {
        ransac.add_shape_factory<CgalTorus>();
    }
    if (!ransac.detect(parameters)) {
        error = "CGAL Efficient RANSAC rejected the input or shape factories";
        return false;
    }

    std::size_t planeCount = 0U;
    std::size_t cylinderCount = 0U;
    std::size_t sphereCount = 0U;
    std::size_t coneCount = 0U;
    std::size_t torusCount = 0U;
    std::vector<PlaneEquation> acceptedPlanes;
    std::vector<modeling::Vec3> planeCentroids;
    for (const std::shared_ptr<Ransac::Shape>& shape : ransac.shapes()) {
        if (const auto plane = std::dynamic_pointer_cast<CgalPlane>(shape)) {
            if (planeCount >= options.plane.maximumPlanes) {
                continue;
            }
            PlaneEvidence evidence;
            modeling::Vec3 centroid{};
            if (!makePlaneEvidence(
                    input, *plane, points, options.plane, evidence, centroid)) {
                continue;
            }
            bool duplicate = false;
            for (std::size_t index = 0; index < planeCount; ++index) {
                if (duplicatePlane(
                        evidence.plane,
                        centroid,
                        acceptedPlanes[index],
                        planeCentroids[index],
                        options.plane)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                output.evidence.emplace_back(std::move(evidence));
                acceptedPlanes.push_back(
                    std::get<PlaneEvidence>(output.evidence.back()).plane);
                planeCentroids.push_back(centroid);
                ++planeCount;
            }
            continue;
        }

        if (const auto cylinder =
                std::dynamic_pointer_cast<CgalCylinder>(shape)) {
            if (cylinderCount >= options.cylinder.maximumCylinders) {
                continue;
            }
            CylinderEvidence evidence;
            if (makeCylinderEvidence(
                    input,
                    *cylinder,
                    points,
                    options.cylinder,
                    evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++cylinderCount;
            }
            continue;
        }

        if (const auto sphere = std::dynamic_pointer_cast<CgalSphere>(shape)) {
            if (sphereCount >= options.sphere.maximumSpheres) {
                continue;
            }
            SphereEvidence evidence;
            if (makeSphereEvidence(
                    input, *sphere, points, options.sphere, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++sphereCount;
            }
            continue;
        }

        if (const auto cone = std::dynamic_pointer_cast<CgalCone>(shape)) {
            if (coneCount >= options.cone.maximumCones) {
                continue;
            }
            ConeEvidence evidence;
            if (makeConeEvidence(
                    input, *cone, points, options.cone, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++coneCount;
            }
            continue;
        }

        if (const auto torus = std::dynamic_pointer_cast<CgalTorus>(shape)) {
            if (torusCount >= options.torus.maximumTori) {
                continue;
            }
            TorusEvidence evidence;
            if (makeTorusEvidence(
                    input, *torus, points, options.torus, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++torusCount;
            }
        }
    }

    SurfaceId nextId = 1;
    std::unordered_set<PointId> assigned;
    for (PrimitiveEvidence& evidence : output.evidence) {
        assignId(evidence, nextId++);
        addSupportIds(evidence, assigned);
    }
    for (const PointSample& point : points.points()) {
        if (assigned.find(point.id) == assigned.end()) {
            output.unassignedPointIds.push_back(point.id);
        }
    }

    if (output.evidence.empty()) {
        error = "CGAL Efficient RANSAC found no supported primitive";
        return false;
    }
    return true;
}

}  // namespace reconstruction
