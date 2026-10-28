#include "TestSupport.h"

namespace regression {
void testCylinderPrimitiveProposal() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadius = 10.0;
    constexpr double kHeight = 30.0;
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int ring = 0; ring < 16; ++ring) {
        const double z = 2.0 + kHeight * ring / 15.0;
        for (int sample = 0; sample < 48; ++sample) {
            const double angle = 2.0 * kPi * sample / 48.0;
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            const double radialNoise = 0.004 *
                std::sin(0.71 * ring + 0.37 * sample);
            samples.push_back({
                nextId++,
                {5.0 + (kRadius + radialNoise) * cosine,
                 -3.0 + (kRadius + radialNoise) * sine,
                 z},
                modeling::Vec3{cosine, sine, 0.0},
                1.0,
            });
        }
    }
    for (int index = 0; index < 64; ++index) {
        samples.push_back({
            nextId++,
            {40.0 + index, -50.0 + 0.5 * index, 80.0 + index},
            modeling::Vec3{0.0, 0.0, 1.0},
            1.0,
        });
    }
    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::CylinderDetectionOptions options;
    options.distanceThreshold = 0.02;
    options.minimumSupportPoints = 600;
    options.maximumCylinders = 1;
    options.probability = 0.001;
    options.minimumRadius = 5.0;
    options.maximumRadius = 15.0;

    std::string error;
    reconstruction::PrimitiveDetectionOptions primitiveOptions;
    primitiveOptions.detectPlanes = false;
    primitiveOptions.detectSpheres = false;
    primitiveOptions.detectCones = false;
    primitiveOptions.detectTori = false;
    primitiveOptions.cylinder = options;
    reconstruction::PrimitiveDetectionResult result;
    require(reconstruction::detectPrimitiveEvidence(
                points, primitiveOptions, result, error),
            error);
    require(result.evidence.size() == 1U &&
                reconstruction::primitiveKind(result.evidence.front()) ==
                    reconstruction::PrimitiveKind::Cylinder,
            "CGAL detector must expose one cylinder evidence record");
    const reconstruction::CylinderEvidence& cylinder =
        std::get<reconstruction::CylinderEvidence>(result.evidence.front());
    requireNear(cylinder.cylinder.radius, kRadius, 0.01,
                "cylinder proposal radius");
    require(std::abs(cylinder.cylinder.axisDirection.z) > 0.999999,
            "Cylinder proposal must recover the Z axis");
    requireNear(cylinder.axialMaximum - cylinder.axialMinimum, kHeight,
                1.0e-6, "cylinder proposal axial extent");
    require(cylinder.angularCoverage > 0.95,
            "Full cylinder samples must report nearly complete angular coverage");
    require(cylinder.supportPointIds.size() >= 760U,
            "Noisy cylinder samples must retain their PointIds");

    require(result.unassignedPointIds.size() >= 64U,
            "Cylinder proposal must leave distant outliers unassigned");
    requireNear(cylinder.cylinder.axisPoint.x, 5.0, 0.001,
                "refined cylinder centre X");
    requireNear(cylinder.cylinder.axisPoint.y, -3.0, 0.001,
                "refined cylinder centre Y");
    // Changing the sample stream must still recover the geometry. Returning to
    // the original stream must reproduce parameters and source support exactly,
    // regardless of the detections performed between these two calls.
    for (const std::uint32_t seed : {1U, 999U, 4500U}) {
        primitiveOptions.randomSeed = seed;
        reconstruction::PrimitiveDetectionResult repeated;
        require(reconstruction::detectPrimitiveEvidence(
                    points, primitiveOptions, repeated, error), error);
        require(repeated.evidence.size() == 1U,
                "Every sample stream must retain one cylinder");
        const auto& fitted = std::get<reconstruction::CylinderEvidence>(
            repeated.evidence.front());
        requireNear(fitted.cylinder.radius, cylinder.cylinder.radius, 1.0e-12,
                    "all-support radius must be stable across sample streams");
        requireNear(fitted.cylinder.axisPoint.x, cylinder.cylinder.axisPoint.x,
                    1.0e-12, "all-support centre X must be stable");
        requireNear(fitted.cylinder.axisPoint.y, cylinder.cylinder.axisPoint.y,
                    1.0e-12, "all-support centre Y must be stable");
        if (seed == 4500U) {
            require(fitted.supportPointIds == cylinder.supportPointIds,
                    "Repeated detection must preserve identical source support");
            require(fitted.cylinder.radius == cylinder.cylinder.radius &&
                        fitted.cylinder.axisPoint.x == cylinder.cylinder.axisPoint.x &&
                        fitted.cylinder.axisPoint.y == cylinder.cylinder.axisPoint.y,
                    "Same seed must reproduce cylinder parameters exactly");
        }
    }
    auto pairedSamples = points.points();
    for (const auto& point : points.points()) {
        if (point.id > 768U) {
            continue;
        }
        auto second = point;
        second.id += 10000U;
        second.position.x += 22.0;
        pairedSamples.push_back(second);
    }
    const reconstruction::PointStore paired(
        std::move(pairedSamples), reconstruction::LengthUnit::Millimeter);
    primitiveOptions.cylinder.maximumCylinders = 2;
    reconstruction::PrimitiveDetectionResult twoCylinders;
    require(reconstruction::detectPrimitiveEvidence(
                paired, primitiveOptions, twoCylinders, error), error);
    require(twoCylinders.evidence.size() == 2U,
            "Nearby real cylinders with distinct support must remain separate");
}
} // namespace regression
