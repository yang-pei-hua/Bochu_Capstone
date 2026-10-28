#include "detail/RansacEvidence.h"
#include <Eigen/Dense>

namespace reconstruction::detail::cgal {
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
} // namespace reconstruction::detail::cgal
