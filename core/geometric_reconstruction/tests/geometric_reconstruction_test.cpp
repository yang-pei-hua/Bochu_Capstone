#include "modeling/StepExporter.h"
#include "reconstruction/AxisAlignedBoxRecognizer.h"
#include "reconstruction/BoxRecognizer.h"
#include "reconstruction/ModelingAdapter.h"
#include "reconstruction/PointCloudPreprocessor.h"
#include "reconstruction/PrimitiveDetector.h"
#include "reconstruction/SyntheticPointCloud.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <cmath>
#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& label) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            label + " expected " + std::to_string(expected) + ", got " +
            std::to_string(actual));
    }
}

double volumeOf(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

void testSyntheticBoxToCad(const std::filesystem::path& outputDirectory) {
    reconstruction::SyntheticBoxRequest request;
    request.box.pose.origin = modeling::Vec3{10.0, -5.0, 2.0};
    request.box.sizeX = 50.0;
    request.box.sizeY = 30.0;
    request.box.sizeZ = 20.0;
    request.samplesPerEdge = 11;

    const reconstruction::PointStore points =
        reconstruction::makeSyntheticBoxSurface(request);
    require(points.size() == 6U * 11U * 11U,
            "Synthetic box must sample all six source surfaces");
    require(points.unit() == reconstruction::LengthUnit::Millimeter,
            "Synthetic point cloud must carry millimeter units");

    reconstruction::BoxCandidate candidate;
    std::string error;
    require(reconstruction::recognizeAxisAlignedBox(
                points, 1.0e-7, candidate, error),
            error);
    require(candidate.sourceSurfaces.size() == 6U,
            "Box recognition must retain six plane evidence records");
    for (const reconstruction::PlaneEvidence& surface :
         candidate.sourceSurfaces) {
        require(!surface.supportPointIds.empty(),
                "Every recognized plane must reference supporting PointIds");
    }
    requireNear(candidate.primitive.pose.origin.x, 10.0, 1.0e-9,
                "recognized box origin X");
    requireNear(candidate.primitive.pose.origin.y, -5.0, 1.0e-9,
                "recognized box origin Y");
    requireNear(candidate.primitive.pose.origin.z, 2.0, 1.0e-9,
                "recognized box origin Z");
    requireNear(candidate.primitive.sizeX, 50.0, 1.0e-9,
                "recognized box size X");
    requireNear(candidate.primitive.sizeY, 30.0, 1.0e-9,
                "recognized box size Y");
    requireNear(candidate.primitive.sizeZ, 20.0, 1.0e-9,
                "recognized box size Z");
    requireNear(candidate.confidence, 1.0, 1.0e-12,
                "recognized box confidence");

    modeling::ModelingCore core;
    const modeling::ModelPatchResult committed =
        reconstruction::commitBoxCandidate(candidate, core, 0);
    require(committed.success, committed.error);
    require(core.features().size() == 1U,
            "Accepted reconstruction must create one core model-state node");
    require(core.features().front().type == modeling::FeatureType::BoxPrimitive,
            "Reconstructed box must remain a semantic primitive");
    requireNear(volumeOf(core.bodyShape()), 50.0 * 30.0 * 20.0, 1.0e-5,
                "reconstructed CAD volume");

    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path stepPath =
        outputDirectory / "synthetic_box.step";
    require(modeling::exportStep(core.bodyShape(), stepPath, &error), error);
    require(std::filesystem::exists(stepPath) &&
                std::filesystem::file_size(stepPath) > 0U,
            "Synthetic box STEP output was not created");
}

void testRecognitionRequiresKnownScale() {
    std::vector<reconstruction::PointSample> points{
        {1, {0.0, 0.0, 0.0}, std::nullopt, 1.0},
        {2, {1.0, 1.0, 1.0}, std::nullopt, 1.0},
    };
    const reconstruction::PointStore arbitrary(
        std::move(points), reconstruction::LengthUnit::Arbitrary);
    reconstruction::BoxCandidate candidate;
    std::string error;
    require(!reconstruction::recognizeAxisAlignedBox(
                arbitrary, 1.0e-6, candidate, error),
            "Recognition must reject point clouds without a physical scale");
}

void testRotatedNoisyBoxWithOutliers(
    const std::filesystem::path& outputDirectory) {
    constexpr double kAngle = 0.5235987755982988;  // 30 degrees
    reconstruction::SyntheticBoxRequest request;
    request.box.pose.origin = modeling::Vec3{-12.0, 8.0, 4.0};
    request.box.pose.xDirection =
        modeling::Vec3{std::cos(kAngle), std::sin(kAngle), 0.0};
    request.box.pose.zDirection = modeling::Vec3{
        -0.25,
        0.4330127018922193,
        0.8660254037844386,
    };
    request.box.sizeX = 50.0;
    request.box.sizeY = 30.0;
    request.box.sizeZ = 20.0;
    request.samplesPerEdge = 18;
    request.gaussianNoiseSigma = 0.015;
    request.outlierCount = 120;
    request.randomSeed = 42;
    const reconstruction::PointStore points =
        reconstruction::makeSyntheticBoxSurface(request);

    reconstruction::PlaneDetectionOptions planeOptions;
    planeOptions.distanceThreshold = 0.07;
    planeOptions.minimumSupportPoints = 180;
    planeOptions.maximumPlanes = 6;
    planeOptions.probability = 0.001;
    reconstruction::BoxRecognitionOptions boxOptions;
    boxOptions.angularToleranceRadians = 0.05235987755982989;  // 3 degrees

    reconstruction::BoxCandidate candidate;
    std::string error;
    require(reconstruction::reconstructBox(
                points, planeOptions, boxOptions, candidate, error),
            error);
    require(candidate.sourceSurfaces.size() == 6U,
            "General box reconstruction must recover six planes");
    require(candidate.confidence > 0.65,
            "Rotated noisy box should retain useful reconstruction confidence");

    std::array<double, 3> dimensions{
        candidate.primitive.sizeX,
        candidate.primitive.sizeY,
        candidate.primitive.sizeZ,
    };
    std::sort(dimensions.begin(), dimensions.end());
    requireNear(dimensions[0], 20.0, 0.12, "rotated box smallest dimension");
    requireNear(dimensions[1], 30.0, 0.12, "rotated box middle dimension");
    requireNear(dimensions[2], 50.0, 0.12, "rotated box largest dimension");

    modeling::ModelingCore core;
    const modeling::ModelPatchResult committed =
        reconstruction::commitBoxCandidate(candidate, core, 0);
    require(committed.success, committed.error);
    requireNear(volumeOf(core.bodyShape()), 30000.0, 250.0,
                "rotated noisy reconstructed CAD volume");

    const std::filesystem::path stepPath =
        outputDirectory / "rotated_noisy_box.step";
    require(modeling::exportStep(core.bodyShape(), stepPath, &error), error);
}

void testDuplicateDepthLayersDoNotHideBoxFaces() {
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    const auto addGrid = [&samples, &nextId](
                             int firstCount,
                             int secondCount,
                             const modeling::Vec3& normal,
                             const auto& positionAt) {
        for (int first = 0; first < firstCount; ++first) {
            for (int second = 0; second < secondCount; ++second) {
                samples.push_back({
                    nextId++,
                    positionAt(first, second),
                    normal,
                    1.0,
                });
            }
        }
    };

    // The two Y faces deliberately dominate the cloud. Two lower-density
    // depth layers sit just outside their RANSAC bands, while the real Z faces
    // have still lower support. A raw six-candidate cutoff therefore misses Z.
    for (const double y : {0.0, 30.0}) {
        const modeling::Vec3 normal{0.0, y == 0.0 ? -1.0 : 1.0, 0.0};
        addGrid(41, 41, normal, [y](int x, int z) {
            return modeling::Vec3{1.25 * x, y, 0.5 * z};
        });
    }
    for (const double x : {0.0, 50.0}) {
        const modeling::Vec3 normal{x == 0.0 ? -1.0 : 1.0, 0.0, 0.0};
        addGrid(21, 21, normal, [x](int y, int z) {
            return modeling::Vec3{x, 1.5 * y, static_cast<double>(z)};
        });
    }
    for (const double y : {0.18, 29.82}) {
        const modeling::Vec3 normal{0.0, y < 1.0 ? -1.0 : 1.0, 0.0};
        addGrid(16, 16, normal, [y](int x, int z) {
            return modeling::Vec3{
                (50.0 / 15.0) * x,
                y,
                (20.0 / 15.0) * z,
            };
        });
    }
    for (const double z : {0.0, 20.0}) {
        const modeling::Vec3 normal{0.0, 0.0, z == 0.0 ? -1.0 : 1.0};
        addGrid(11, 11, normal, [z](int x, int y) {
            return modeling::Vec3{5.0 * x, 3.0 * y, z};
        });
    }

    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::PlaneDetectionOptions planeOptions;
    planeOptions.distanceThreshold = 0.08;
    planeOptions.minimumSupportPoints = 80;
    planeOptions.maximumPlanes = 6;
    planeOptions.probability = 0.001;

    reconstruction::BoxCandidate candidate;
    reconstruction::BoxRecognitionOptions boxOptions;
    std::string error;
    require(reconstruction::reconstructBox(
                points, planeOptions, boxOptions, candidate, error),
            error);
    require(candidate.sourceSurfaces.size() == 6U,
            "Duplicate depth layers must not consume physical face slots");

    std::array<double, 3> dimensions{
        candidate.primitive.sizeX,
        candidate.primitive.sizeY,
        candidate.primitive.sizeZ,
    };
    std::sort(dimensions.begin(), dimensions.end());
    requireNear(dimensions[0], 20.0, 0.05, "layered box smallest dimension");
    requireNear(dimensions[1], 30.0, 0.05, "layered box middle dimension");
    requireNear(dimensions[2], 50.0, 0.05, "layered box largest dimension");
}

void testScaleAwarePreprocessingAndNormalEstimation() {
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int x = 0; x < 11; ++x) {
        for (int y = 0; y < 11; ++y) {
            samples.push_back({
                nextId++,
                {0.001 * x, 0.001 * y, 0.0},
                std::nullopt,
                1.0,
            });
        }
    }
    const reconstruction::PointId outlierId = nextId;
    samples.push_back({outlierId, {1.0, 1.0, 1.0}, std::nullopt, 1.0});
    const reconstruction::PointStore meters(
        std::move(samples), reconstruction::LengthUnit::Meter);

    reconstruction::PointCloudPreprocessingOptions options;
    options.voxelSize = 0.0;
    options.outlierNeighborCount = 8;
    options.outlierStandardDeviationMultiplier = 1.0;
    options.normalNeighborCount = 8;
    reconstruction::PointStore processed(
        {}, reconstruction::LengthUnit::Arbitrary);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(
                meters, options, processed, report, error),
            error);
    require(processed.unit() == reconstruction::LengthUnit::Millimeter,
            "Meter input must be converted to millimetres");
    requireNear(report.appliedScaleToMillimeters, 1000.0, 1.0e-12,
                "preprocessing metric scale");
    require(processed.size() == 121U,
            "The isolated point must be removed without removing the plane");
    require(report.removedAsOutliers == 1U,
            "Preprocessing must report the isolated outlier");
    for (const reconstruction::PointSample& point : processed.points()) {
        require(point.id != outlierId,
                "The isolated outlier ID must not survive preprocessing");
        require(point.normal.has_value(),
                "PCA preprocessing must estimate a plane normal");
        require(std::abs(point.normal->z) > 0.999,
                "Estimated normal must be perpendicular to the XY plane");
        require(point.curvature.has_value() && *point.curvature < 1.0e-10,
                "A noise-free plane must have near-zero curvature");
    }
    requireNear(processed.points().back().position.x, 10.0, 1.0e-12,
                "meter-to-millimetre position conversion");
}

void testVoxelDownsamplingPreservesOriginalPointIds() {
    std::vector<reconstruction::PointSample> samples{
        {10, {0.1, 0.1, 0.1}, std::nullopt, 1.0},
        {11, {0.2, 0.2, 0.2}, std::nullopt, 1.0},
        {12, {1.2, 0.2, 0.2}, std::nullopt, 1.0},
    };
    const reconstruction::PointStore input(
        std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::PointCloudPreprocessingOptions options;
    options.voxelSize = 1.0;
    options.removeStatisticalOutliers = false;
    options.estimateMissingNormals = false;

    reconstruction::PointStore processed(
        {}, reconstruction::LengthUnit::Arbitrary);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(
                input, options, processed, report, error),
            error);
    require(processed.size() == 2U,
            "Two occupied voxels must produce two representatives");
    require(report.removedByVoxel == 1U,
            "Voxel report must count one removed observation");
    require(processed.points()[0].id == 11U && processed.points()[1].id == 12U,
            "Voxel representatives must retain deterministic original PointIds");
}

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
}

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

}  // namespace

int main() {
    try {
        testSyntheticBoxToCad(RECONSTRUCTION_TEST_OUTPUT_DIR);
        testRecognitionRequiresKnownScale();
        testRotatedNoisyBoxWithOutliers(RECONSTRUCTION_TEST_OUTPUT_DIR);
        testDuplicateDepthLayersDoNotHideBoxFaces();
        testScaleAwarePreprocessingAndNormalEstimation();
        testVoxelDownsamplingPreservesOriginalPointIds();
        testCylinderPrimitiveProposal();
        testSpherePrimitiveProposal();
        testConePrimitiveProposal();
        testTorusPrimitiveProposal();
        std::cout << "Geometric reconstruction vertical-slice test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Geometric reconstruction test failed: "
                  << exception.what() << '\n';
        return 1;
    }
}
