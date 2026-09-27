#include "modeling/StepExporter.h"
#include "reconstruction/AxisAlignedBoxRecognizer.h"
#include "reconstruction/BoxRecognizer.h"
#include "reconstruction/ModelingAdapter.h"
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
    planeOptions.ransacIterations = 1200;
    planeOptions.randomSeed = 7;
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

}  // namespace

int main() {
    try {
        testSyntheticBoxToCad(RECONSTRUCTION_TEST_OUTPUT_DIR);
        testRecognitionRequiresKnownScale();
        testRotatedNoisyBoxWithOutliers(RECONSTRUCTION_TEST_OUTPUT_DIR);
        std::cout << "Geometric reconstruction vertical-slice test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Geometric reconstruction test failed: "
                  << exception.what() << '\n';
        return 1;
    }
}
