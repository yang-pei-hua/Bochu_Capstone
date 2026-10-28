#include "TestSupport.h"

namespace regression {
void testThroughHoleRecognitionAndCadCommit() {
    constexpr double kPi = 3.14159265358979323846;
    reconstruction::BoxCandidate box;
    box.primitive.pose.origin = modeling::Vec3{0.0, 0.0, 0.0};
    box.primitive.sizeX = 50.0;
    box.primitive.sizeY = 30.0;
    box.primitive.sizeZ = 20.0;
    box.confidence = 1.0;

    reconstruction::CylinderEvidence cylinder;
    cylinder.id = 7;
    cylinder.cylinder.axisPoint = modeling::Vec3{25.0, 15.0, 0.0};
    cylinder.cylinder.axisDirection = modeling::Vec3{0.0, 0.0, 1.0};
    cylinder.cylinder.radius = 4.0;
    cylinder.axialMinimum = 0.0;
    cylinder.axialMaximum = 20.0;
    cylinder.angularCoverage = 0.98;
    cylinder.confidence = 0.95;

    std::vector<reconstruction::ThroughHoleCandidate> holes;
    std::string error;
    require(reconstruction::recognizeThroughHoles(
                box,
                {cylinder},
                reconstruction::ThroughHoleRecognitionOptions{},
                holes,
                error),
            error);
    require(holes.size() == 1U,
            "A complete contained cylinder must be recognized as a through hole");
    requireNear(holes.front().primitive.radius, 4.0, 1.0e-12,
                "recognized through-hole radius");
    requireNear(holes.front().bodyThickness, 20.0, 1.0e-12,
                "recognized through-hole body thickness");

    reconstruction::CylinderEvidence blind = cylinder;
    blind.axialMaximum = 12.0;
    std::vector<reconstruction::ThroughHoleCandidate> rejected;
    require(reconstruction::recognizeThroughHoles(
                box,
                {blind},
                reconstruction::ThroughHoleRecognitionOptions{},
                rejected,
                error),
            error);
    require(rejected.empty(),
            "A cylinder that does not reach both exterior faces is a blind hole");

    reconstruction::CylinderEvidence outside = cylinder;
    outside.cylinder.axisPoint = modeling::Vec3{2.0, 15.0, 0.0};
    require(reconstruction::recognizeThroughHoles(
                box,
                {outside},
                reconstruction::ThroughHoleRecognitionOptions{},
                rejected,
                error),
            error);
    require(rejected.empty(),
            "A cylinder crossing the box side boundary is not a contained hole");

    reconstruction::BoxWithThroughHolesCandidate candidate;
    candidate.box = box;
    candidate.throughHoles = holes;
    candidate.confidence = 0.95;
    modeling::ModelingCore core;
    const modeling::ModelPatchResult committed =
        reconstruction::commitBoxWithThroughHolesCandidate(candidate, core, 0);
    require(committed.success, committed.error);
    require(core.features().size() == 2U,
            "Box plus one through hole must create two semantic features");
    require(core.features()[0].type == modeling::FeatureType::BoxPrimitive &&
                core.features()[1].type ==
                    modeling::FeatureType::ThroughHolePrimitive,
            "The recognized hole must remain semantic feature-graph state");
    requireNear(
        volumeOf(core.bodyShape()),
        50.0 * 30.0 * 20.0 - kPi * 4.0 * 4.0 * 20.0,
        1.0e-4,
        "box with through-hole CAD volume");
}

void testBoxThroughHoleEndToEnd() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kCenterX = 25.0;
    constexpr double kCenterY = 15.0;
    constexpr double kRadius = 4.0;
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;

    const auto addFace = [&](const modeling::Vec3& normal, auto pointAt) {
        for (int row = 0; row < 21; ++row) {
            for (int column = 0; column < 21; ++column) {
                const modeling::Vec3 point = pointAt(row, column);
                samples.push_back({nextId++, point, normal, 1.0});
            }
        }
    };
    addFace({-1.0, 0.0, 0.0}, [](int row, int column) {
        return modeling::Vec3{0.0, 1.5 * row, static_cast<double>(column)};
    });
    addFace({1.0, 0.0, 0.0}, [](int row, int column) {
        return modeling::Vec3{50.0, 1.5 * row, static_cast<double>(column)};
    });
    addFace({0.0, -1.0, 0.0}, [](int row, int column) {
        return modeling::Vec3{2.5 * row, 0.0, static_cast<double>(column)};
    });
    addFace({0.0, 1.0, 0.0}, [](int row, int column) {
        return modeling::Vec3{2.5 * row, 30.0, static_cast<double>(column)};
    });

    for (const double z : {0.0, 20.0}) {
        for (int row = 0; row < 21; ++row) {
            const double y = 1.5 * row;
            for (int column = 0; column < 21; ++column) {
                const double x = 2.5 * column;
                const double dx = x - kCenterX;
                const double dy = y - kCenterY;
                if (dx * dx + dy * dy < kRadius * kRadius) {
                    continue;
                }
                samples.push_back({
                    nextId++,
                    {x, y, z},
                    modeling::Vec3{0.0, 0.0, z == 0.0 ? -1.0 : 1.0},
                    1.0,
                });
            }
        }
    }

    for (int ring = 0; ring < 17; ++ring) {
        const double z = 20.0 * ring / 16.0;
        for (int sample = 0; sample < 64; ++sample) {
            const double angle = 2.0 * kPi * sample / 64.0;
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            samples.push_back({
                nextId++,
                {kCenterX + kRadius * cosine,
                 kCenterY + kRadius * sine,
                 z},
                modeling::Vec3{cosine, sine, 0.0},
                1.0,
            });
        }
    }

    const reconstruction::PointStore points(
        std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::PlaneDetectionOptions planeOptions;
    planeOptions.distanceThreshold = 0.03;
    planeOptions.minimumSupportPoints = 300;
    planeOptions.maximumPlanes = 6;
    planeOptions.probability = 0.001;
    reconstruction::CylinderDetectionOptions cylinderOptions;
    cylinderOptions.distanceThreshold = 0.03;
    cylinderOptions.minimumSupportPoints = 900;
    cylinderOptions.maximumCylinders = 1;
    cylinderOptions.probability = 0.001;
    cylinderOptions.minimumRadius = 3.0;
    cylinderOptions.maximumRadius = 5.0;

    reconstruction::BoxWithThroughHolesCandidate candidate;
    std::string error;
    require(reconstruction::reconstructBoxWithThroughHoles(
                points,
                planeOptions,
                cylinderOptions,
                reconstruction::BoxRecognitionOptions{},
                reconstruction::ThroughHoleRecognitionOptions{},
                candidate,
                error),
            error);
    require(candidate.throughHoles.size() == 1U,
            "End-to-end reconstruction must classify the cylinder as one through hole");
    requireNear(candidate.throughHoles.front().primitive.radius, kRadius, 0.02,
                "end-to-end through-hole radius");
    auto oriented = points.points();
    reconstruction::TemplateHint wall{3, reconstruction::TemplateKind::Cylinder, {}};
    for (auto& point : oriented) if (point.id >= nextId - 17 * 64) {
        wall.points.push_back(point.id);
        point.normal = modeling::Vec3{-point.normal->x, -point.normal->y, -point.normal->z};
    }
    reconstruction::GuidedSolidResult guided;
    require(reconstruction::reconstructGuidedSolid(
                reconstruction::PointStore(oriented, points.unit()), planeOptions, cylinderOptions,
                {wall}, guided, error), error);
    require(guided.candidate.boxes.size() == 1 && guided.candidate.throughHoles.size() == 1
            && guided.fits[0].surface && !guided.fits[0].solid,
            "Manual cylinder wall remains a through hole rather than becoming positive material");
}
} // namespace regression
