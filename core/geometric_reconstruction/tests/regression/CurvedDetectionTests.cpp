#include "TestSupport.h"

namespace regression {
void testSpherePrimitiveProposal() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadius = 12.0;
    const modeling::Vec3 center{4.0, -2.0, 7.0};
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int latitude = 1; latitude <= 16; ++latitude) {
        const double polar = kPi * latitude / 17.0;
        for (int longitude = 0; longitude < 32; ++longitude) {
            const double azimuth = 2.0 * kPi * longitude / 32.0;
            const modeling::Vec3 normal{
                std::sin(polar) * std::cos(azimuth),
                std::sin(polar) * std::sin(azimuth),
                std::cos(polar),
            };
            const double noise = 0.003 *
                std::sin(0.41 * latitude + 0.73 * longitude);
            samples.push_back({
                nextId++,
                {center.x + (kRadius + noise) * normal.x,
                 center.y + (kRadius + noise) * normal.y,
                 center.z + (kRadius + noise) * normal.z},
                normal,
                1.0,
            });
        }
    }
    for (int index = 0; index < 32; ++index) {
        samples.push_back({
            nextId++,
            {40.0 + index, 30.0 - index, 50.0 + 0.5 * index},
            modeling::Vec3{0.0, 0.0, 1.0},
            0.2,
        });
    }
    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);

    reconstruction::PrimitiveDetectionOptions options;
    options.detectPlanes = false;
    options.detectCylinders = false;
    options.detectSpheres = true;
    options.detectCones = false;
    options.detectTori = false;
    options.sphere.distanceThreshold = 0.02;
    options.sphere.minimumSupportPoints = 400;
    options.sphere.maximumSpheres = 1;
    options.sphere.probability = 0.001;
    options.sphere.minimumRadius = 8.0;
    options.sphere.maximumRadius = 16.0;

    reconstruction::PrimitiveDetectionResult result;
    std::string error;
    require(reconstruction::detectPrimitiveEvidence(
                points, options, result, error),
            error);
    require(result.evidence.size() == 1U &&
                reconstruction::primitiveKind(result.evidence.front()) ==
                    reconstruction::PrimitiveKind::Sphere,
            "CGAL detector must expose one sphere evidence record");
    const reconstruction::SphereEvidence& sphere =
        std::get<reconstruction::SphereEvidence>(result.evidence.front());
    requireNear(sphere.sphere.center.x, center.x, 0.02, "sphere center X");
    requireNear(sphere.sphere.center.y, center.y, 0.02, "sphere center Y");
    requireNear(sphere.sphere.center.z, center.z, 0.02, "sphere center Z");
    requireNear(sphere.sphere.radius, kRadius, 0.01, "sphere radius");
    require(sphere.supportPointIds.size() >= 500U,
            "Sphere proposal must retain nearly all surface samples");
    require(result.unassignedPointIds.size() >= 32U,
            "Sphere proposal must leave distant outliers unassigned");
}

void testConePrimitiveProposal() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kAngle = kPi / 6.0;
    const modeling::Vec3 apex{1.0, 2.0, 3.0};
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int ring = 0; ring < 12; ++ring) {
        const double height = 5.0 + 15.0 * ring / 11.0;
        const double radius = height * std::tan(kAngle);
        for (int sample = 0; sample < 48; ++sample) {
            const double azimuth = 2.0 * kPi * sample / 48.0;
            const modeling::Vec3 radial{
                std::cos(azimuth), std::sin(azimuth), 0.0};
            const modeling::Vec3 normal{
                std::cos(kAngle) * radial.x,
                std::cos(kAngle) * radial.y,
                -std::sin(kAngle),
            };
            samples.push_back({
                nextId++,
                {apex.x + radius * radial.x,
                 apex.y + radius * radial.y,
                 apex.z + height},
                normal,
                1.0,
            });
        }
    }
    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);

    reconstruction::PrimitiveDetectionOptions options;
    options.detectPlanes = false;
    options.detectCylinders = false;
    options.detectSpheres = false;
    options.detectCones = true;
    options.detectTori = false;
    options.cone.distanceThreshold = 0.02;
    options.cone.minimumSupportPoints = 500;
    options.cone.maximumCones = 1;
    options.cone.probability = 0.001;

    reconstruction::PrimitiveDetectionResult result;
    std::string error;
    require(reconstruction::detectPrimitiveEvidence(
                points, options, result, error),
            error);
    require(result.evidence.size() == 1U &&
                reconstruction::primitiveKind(result.evidence.front()) ==
                    reconstruction::PrimitiveKind::Cone,
            "CGAL detector must expose one cone evidence record");
    const reconstruction::ConeEvidence& cone =
        std::get<reconstruction::ConeEvidence>(result.evidence.front());
    requireNear(cone.cone.apex.x, apex.x, 0.02, "cone apex X");
    requireNear(cone.cone.apex.y, apex.y, 0.02, "cone apex Y");
    requireNear(cone.cone.apex.z, apex.z, 0.02, "cone apex Z");
    require(std::abs(cone.cone.axisDirection.z) > 0.999,
            "Cone proposal must recover the Z axis");
    requireNear(cone.cone.openingAngleRadians, kAngle, 0.002,
                "cone opening angle");
    require(cone.angularCoverage > 0.95,
            "Full cone samples must report nearly complete angular coverage");
}

void testTorusPrimitiveProposal() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMajorRadius = 15.0;
    constexpr double kMinorRadius = 4.0;
    const modeling::Vec3 center{-4.0, 3.0, 2.0};
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int major = 0; major < 48; ++major) {
        const double u = 2.0 * kPi * major / 48.0;
        const modeling::Vec3 ringDirection{
            std::cos(u), std::sin(u), 0.0};
        for (int minor = 0; minor < 16; ++minor) {
            const double v = 2.0 * kPi * minor / 16.0;
            const modeling::Vec3 normal{
                std::cos(v) * ringDirection.x,
                std::cos(v) * ringDirection.y,
                std::sin(v),
            };
            const double inPlaneRadius =
                kMajorRadius + kMinorRadius * std::cos(v);
            samples.push_back({
                nextId++,
                {center.x + inPlaneRadius * ringDirection.x,
                 center.y + inPlaneRadius * ringDirection.y,
                 center.z + kMinorRadius * std::sin(v)},
                normal,
                1.0,
            });
        }
    }
    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);

    reconstruction::PrimitiveDetectionOptions options;
    options.detectPlanes = false;
    options.detectCylinders = false;
    options.detectSpheres = false;
    options.detectCones = false;
    options.detectTori = true;
    options.torus.distanceThreshold = 0.02;
    options.torus.minimumSupportPoints = 700;
    options.torus.maximumTori = 1;
    options.torus.probability = 0.001;
    options.torus.minimumMajorRadius = 10.0;
    options.torus.maximumMajorRadius = 20.0;
    options.torus.minimumMinorRadius = 2.0;
    options.torus.maximumMinorRadius = 6.0;

    reconstruction::PrimitiveDetectionResult result;
    std::string error;
    require(reconstruction::detectPrimitiveEvidence(
                points, options, result, error),
            error);
    require(result.evidence.size() == 1U &&
                reconstruction::primitiveKind(result.evidence.front()) ==
                    reconstruction::PrimitiveKind::Torus,
            "CGAL detector must expose one torus evidence record");
    const reconstruction::TorusEvidence& torus =
        std::get<reconstruction::TorusEvidence>(result.evidence.front());
    requireNear(torus.torus.center.x, center.x, 0.02, "torus center X");
    requireNear(torus.torus.center.y, center.y, 0.02, "torus center Y");
    requireNear(torus.torus.center.z, center.z, 0.02, "torus center Z");
    require(std::abs(torus.torus.axisDirection.z) > 0.999,
            "Torus proposal must recover the Z axis");
    requireNear(torus.torus.majorRadius, kMajorRadius, 0.02,
                "torus major radius");
    requireNear(torus.torus.minorRadius, kMinorRadius, 0.02,
                "torus minor radius");
    require(torus.supportPointIds.size() >= 740U,
            "Torus proposal must retain nearly all surface samples");
}
} // namespace regression
