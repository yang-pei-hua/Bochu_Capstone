#include "reconstruction/ThroughHoleRecognizer.h"

#include "Math3.h"
#include "reconstruction/PrimitiveDetector.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace reconstruction {
namespace {

struct BoxFrame {
    std::array<modeling::Vec3, 3> axes{};
    std::array<double, 3> sizes{};
};

BoxFrame makeBoxFrame(const modeling::BoxPrimitiveParams& box) {
    const modeling::Vec3 x = math3::normalized(box.pose.xDirection);
    const modeling::Vec3 z = math3::normalized(box.pose.zDirection);
    const modeling::Vec3 y = math3::normalized(math3::cross(z, x));
    return {{{x, y, z}}, {{box.sizeX, box.sizeY, box.sizeZ}}};
}

bool validOptions(const ThroughHoleRecognitionOptions& options) {
    return std::isfinite(options.angularToleranceRadians) &&
        options.angularToleranceRadians > 0.0 &&
        options.angularToleranceRadians < 0.7853981633974483 &&
        std::isfinite(options.minimumAngularCoverage) &&
        options.minimumAngularCoverage >= 0.0 &&
        options.minimumAngularCoverage <= 1.0 &&
        std::isfinite(options.minimumAxialCoverage) &&
        options.minimumAxialCoverage > 0.0 &&
        options.minimumAxialCoverage <= 1.0 &&
        std::isfinite(options.maximumEndpointGapFraction) &&
        options.maximumEndpointGapFraction >= 0.0 &&
        options.maximumEndpointGapFraction < 0.5 &&
        std::isfinite(options.minimumBoundaryClearance) &&
        options.minimumBoundaryClearance >= 0.0;
}

double coordinate(
    const modeling::Vec3& point,
    const modeling::Vec3& origin,
    const modeling::Vec3& axis) {
    return math3::dot(math3::subtract(point, origin), axis);
}

}  // namespace

bool recognizeThroughHoles(
    const BoxCandidate& box,
    const std::vector<CylinderEvidence>& cylinders,
    const ThroughHoleRecognitionOptions& options,
    std::vector<ThroughHoleCandidate>& output,
    std::string& error) {
    output.clear();
    error.clear();
    if (!validOptions(options)) {
        error = "Through-hole recognition options are outside their valid ranges";
        return false;
    }
    if (!(box.primitive.sizeX > 0.0 && box.primitive.sizeY > 0.0 &&
          box.primitive.sizeZ > 0.0)) {
        error = "Through-hole recognition requires a non-degenerate box";
        return false;
    }

    const BoxFrame frame = makeBoxFrame(box.primitive);
    const double minimumAlignment = std::cos(options.angularToleranceRadians);

    for (const CylinderEvidence& cylinder : cylinders) {
        if (!std::isfinite(cylinder.cylinder.radius) ||
            cylinder.cylinder.radius <= 0.0 ||
            !std::isfinite(cylinder.angularCoverage) ||
            cylinder.angularCoverage < options.minimumAngularCoverage ||
            !std::isfinite(cylinder.axialMinimum) ||
            !std::isfinite(cylinder.axialMaximum) ||
            cylinder.axialMaximum <= cylinder.axialMinimum) {
            continue;
        }

        modeling::Vec3 direction;
        try {
            direction = math3::normalized(cylinder.cylinder.axisDirection);
        } catch (const std::invalid_argument&) {
            continue;
        }

        std::size_t longitudinal = 0;
        double alignment = 0.0;
        for (std::size_t axis = 0; axis < frame.axes.size(); ++axis) {
            const double candidate =
                std::abs(math3::dot(direction, frame.axes[axis]));
            if (candidate > alignment) {
                alignment = candidate;
                longitudinal = axis;
            }
        }
        if (alignment < minimumAlignment) {
            continue;
        }

        const modeling::Vec3 constrainedDirection = frame.axes[longitudinal];
        const double directionProjection =
            math3::dot(direction, constrainedDirection);
        const double originCoordinate = math3::dot(
            math3::subtract(
                box.primitive.pose.origin, cylinder.cylinder.axisPoint),
            constrainedDirection);
        const double firstParameter = originCoordinate / directionProjection;
        const double secondParameter =
            (originCoordinate + frame.sizes[longitudinal]) /
            directionProjection;
        const double expectedMinimum = std::min(firstParameter, secondParameter);
        const double expectedMaximum = std::max(firstParameter, secondParameter);
        const double thickness = frame.sizes[longitudinal];
        const double observedSpan =
            cylinder.axialMaximum - cylinder.axialMinimum;
        const double overlap = std::max(
            0.0,
            std::min(cylinder.axialMaximum, expectedMaximum) -
                std::max(cylinder.axialMinimum, expectedMinimum));
        const double axialCoverage = overlap / thickness;
        const double endpointTolerance =
            thickness * options.maximumEndpointGapFraction;
        if (axialCoverage < options.minimumAxialCoverage ||
            std::abs(cylinder.axialMinimum - expectedMinimum) > endpointTolerance ||
            std::abs(cylinder.axialMaximum - expectedMaximum) > endpointTolerance) {
            continue;
        }

        const modeling::Vec3 firstFaceCenter = math3::add(
            cylinder.cylinder.axisPoint,
            math3::scale(direction, firstParameter));
        bool contained = true;
        for (std::size_t transverse = 0; transverse < frame.axes.size(); ++transverse) {
            if (transverse == longitudinal) {
                continue;
            }
            const double centerCoordinate = coordinate(
                firstFaceCenter, box.primitive.pose.origin, frame.axes[transverse]);
            const double margin =
                cylinder.cylinder.radius + options.minimumBoundaryClearance;
            if (centerCoordinate < margin ||
                centerCoordinate > frame.sizes[transverse] - margin) {
                contained = false;
                break;
            }
        }
        if (!contained) {
            continue;
        }

        ThroughHoleCandidate candidate;
        candidate.primitive.axisPoint = firstFaceCenter;
        candidate.primitive.axisDirection = constrainedDirection;
        candidate.primitive.radius = cylinder.cylinder.radius;
        candidate.sourceSurface = cylinder;
        candidate.bodyThickness = thickness;
        const double angularScore = std::clamp(
            (cylinder.angularCoverage - options.minimumAngularCoverage) /
                std::max(1.0e-12, 1.0 - options.minimumAngularCoverage),
            0.0,
            1.0);
        const double spanScore = std::clamp(
            std::min(observedSpan, thickness) / thickness, 0.0, 1.0);
        candidate.confidence = std::clamp(
            cylinder.confidence * alignment *
                (0.5 + 0.25 * angularScore + 0.25 * spanScore),
            0.0,
            1.0);
        output.push_back(std::move(candidate));
    }

    std::sort(output.begin(), output.end(), [](const auto& left, const auto& right) {
        return left.confidence > right.confidence;
    });
    return true;
}

bool reconstructBoxWithThroughHoles(
    const PointStore& points,
    const PlaneDetectionOptions& planeOptions,
    const CylinderDetectionOptions& cylinderOptions,
    const BoxRecognitionOptions& boxOptions,
    const ThroughHoleRecognitionOptions& holeOptions,
    BoxWithThroughHolesCandidate& output,
    std::string& error) {
    output = {};
    if (!reconstructBox(
            points, planeOptions, boxOptions, output.box, error)) {
        return false;
    }

    PrimitiveDetectionOptions detectionOptions;
    detectionOptions.detectPlanes = false;
    detectionOptions.detectCylinders = true;
    detectionOptions.detectSpheres = false;
    detectionOptions.detectCones = false;
    detectionOptions.detectTori = false;
    detectionOptions.cylinder = cylinderOptions;
    PrimitiveDetectionResult proposals;
    if (!detectPrimitiveEvidence(points, detectionOptions, proposals, error)) {
        return false;
    }

    std::vector<CylinderEvidence> cylinders;
    for (const PrimitiveEvidence& evidence : proposals.evidence) {
        if (const auto* cylinder = std::get_if<CylinderEvidence>(&evidence)) {
            cylinders.push_back(*cylinder);
        }
    }
    if (!recognizeThroughHoles(
            output.box, cylinders, holeOptions, output.throughHoles, error)) {
        return false;
    }

    double confidence = output.box.confidence;
    for (const ThroughHoleCandidate& hole : output.throughHoles) {
        confidence += hole.confidence;
    }
    output.confidence = confidence /
        static_cast<double>(1U + output.throughHoles.size());
    return true;
}

}  // namespace reconstruction
