#include "app/CadReconstructionTask.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

PlyCloud boxCloud()
{
    PlyCloud cloud;
    for (int axis = 0; axis < 3; ++axis) for (float sign : {-1.0F, 1.0F})
        for (int i = 0; i < 10; ++i) for (int j = 0; j < 10; ++j) {
            std::array<float, 3> position{}, normal{};
            position[axis] = 3 * sign;
            normal[axis] = sign;
            position[(axis + 1) % 3] = -3 + (i + 0.5F) * 0.6F;
            position[(axis + 2) % 3] = -3 + (j + 0.5F) * 0.6F;
            cloud.positions.push_back(position);
            cloud.normals.push_back(normal);
        }
    return cloud;
}

PlyCloud cappedCloud(bool cone)
{
    PlyCloud cloud;
    const double pi = std::acos(-1.0), slope = cone ? -1.0 / 3 : 0;
    const auto append = [&](double x, double y, double z, double nx, double ny, double nz) {
        cloud.positions.push_back({float(x), float(y), float(z)});
        cloud.normals.push_back({float(nx), float(ny), float(nz)});
    };
    for (int z = 0; z < 12; ++z) for (int a = 0; a < 48; ++a) {
        const double height = (z + 0.5) * 6 / 12, angle = (a + 0.5) * 2 * pi / 48;
        const double radius = (cone ? 3 : 2) + slope * height;
        const double norm = std::sqrt(1 + slope * slope);
        append(radius * std::cos(angle), radius * std::sin(angle), height,
               std::cos(angle) / norm, std::sin(angle) / norm, -slope / norm);
    }
    for (int cap = 0; cap < 2; ++cap) for (int r = 0; r < 4; ++r)
        for (int a = 0; a < 32; ++a) {
            const double radius = (cone ? (cap ? 1 : 3) : 2) * (r + 0.5) / 4;
            const double angle = (a + 0.5) * 2 * pi / 32;
            append(radius * std::cos(angle), radius * std::sin(angle), cap * 6, 0, 0, cap ? 1 : -1);
        }
    return cloud;
}

CadReconstructionResult run(PlyCloud cloud, int cancellation = 0,
    const std::vector<reconstruction::TemplateHint>& hints = {},
    reconstruction::LengthUnit unit = reconstruction::LengthUnit::Millimeter)
{
    CadReconstructionTask task;
    std::optional<CadReconstructionResult> result;
    bool callbacksOnOwner = true;
    bool busyDuringDelivery = false;
    int completions = 0;
    int heartbeats = 0;
    std::vector<CadReconstructionTask::Stage> stages;
    QObject::connect(&task, &CadReconstructionTask::finished, &task,
        [&](const CadReconstructionResult& value) {
            callbacksOnOwner &= QThread::currentThread() == task.thread();
            busyDuringDelivery = task.isRunning();
            result = value;
            ++completions;
        });
    QObject::connect(&task, &CadReconstructionTask::stageChanged, &task,
        [&](CadReconstructionTask::Stage stage) {
            callbacksOnOwner &= QThread::currentThread() == task.thread();
            stages.push_back(stage);
            if(cancellation==3 && stage==CadReconstructionTask::Stage::FittingTemplates) task.cancel();
        });
    require(task.start(cloud, unit,hints), "start worker");
    cloud.positions.clear();
    cloud.normals.clear();
    require(task.isRunning() && !result, "start returns before queued result delivery");
    require(!task.start({}, reconstruction::LengthUnit::Millimeter), "duplicate start rejected");
    if (cancellation == 2) {
        require(task.findChild<QThread*>()->wait(30000), "worker finished before queued result is delivered");
    }
    if (cancellation && cancellation!=3) {
        task.cancel();
        task.cancel();
        require(task.isCancellationRequested(), "cancellation immediately recorded");
    }
    QTimer heartbeat;
    QObject::connect(&heartbeat, &QTimer::timeout, &task, [&] { ++heartbeats; });
    heartbeat.start(0);
    QElapsedTimer elapsed;
    elapsed.start();
    while (task.isRunning() && elapsed.elapsed() < 30000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    require(!task.isRunning() && result && completions == 1, "worker completes once within deadline");
    require(callbacksOnOwner && busyDuringDelivery, "results delivered on owner thread before busy clears");
    require(heartbeats > 0, "event loop remains responsive while computation runs");
    if (!cancellation && result->outcome == CadReconstructionResult::Outcome::Success) {
        const auto expected=hints.empty() ? CadReconstructionTask::Stage::Recognizing : CadReconstructionTask::Stage::FittingTemplates;
        require(std::find(stages.begin(), stages.end(), expected) != stages.end(),
                "recognition stage reported");
    }
    return *result;
}
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    try {
        const auto success = run(boxCloud());
        require(success.outcome == CadReconstructionResult::Outcome::Success
                && success.preprocessingComplete && success.preprocessing.inputPointCount == 600
                && success.candidate.boxes.size() == 1, "snapshot reconstructs supported box");
        const auto& box = success.candidate.boxes.front().primitive;
        require(std::abs(box.sizeX - 6) < 0.01 && std::abs(box.sizeY - 6) < 0.01
                && std::abs(box.sizeZ - 6) < 0.01, "background fit preserves dimensions");
        reconstruction::TemplateHint planeHint{7,reconstruction::TemplateKind::Plane,{}};
        for(reconstruction::PointId id=1;id<=100;++id) planeHint.points.push_back(id);
        const auto guided=run(boxCloud(),0,{planeHint});
        if(guided.outcome!=CadReconstructionResult::Outcome::Success) throw std::runtime_error(guided.error.toStdString());
        require(guided.templateFits.size()==1 && guided.templateFits[0].regionId==7 && guided.templateFits[0].surface
                && guided.candidate.boxes.size()==1,"manual plane is fitted and whole closed box is recovered");
        auto meters = boxCloud();
        for (auto& point : meters.positions) for (auto& coordinate : point) coordinate /= 1000;
        const auto metric = run(meters, 0, {planeHint}, reconstruction::LengthUnit::Meter);
        require(metric.outcome == CadReconstructionResult::Outcome::Success
                && metric.preprocessing.appliedScaleToMillimeters == 1000
                && std::abs(metric.candidate.boxes[0].primitive.sizeX - 6) < 0.01,
                "guided observations in meters reconstruct millimeter CAD with stable hint IDs");
        auto boxHint=planeHint; boxHint.kind=reconstruction::TemplateKind::Box;
        const auto partialBox=run(boxCloud(),0,{boxHint});
        require(partialBox.outcome==CadReconstructionResult::Outcome::Success && partialBox.templateFits[0].box,
                "partial cuboid template completes only from observed global boundary");
        boxHint.points.clear(); for(reconstruction::PointId id=1;id<=600;++id) boxHint.points.push_back(id);
        const auto fullBox=run(boxCloud(),0,{boxHint});
        require(fullBox.outcome==CadReconstructionResult::Outcome::Success && fullBox.templateFits[0].box,
                "complete cuboid template is reconstructed before continuing globally");
        auto wrong=planeHint; wrong.kind=reconstruction::TemplateKind::Sphere;
        const auto conflict=run(boxCloud(),0,{wrong});
        require(conflict.outcome==CadReconstructionResult::Outcome::Unsupported && conflict.failedRegionId==7,
                "conflicting template rejected instead of silently ignored");
        wrong=planeHint; wrong.points.resize(3);
        require(run(boxCloud(),0,{wrong}).outcome==CadReconstructionResult::Outcome::Unsupported,
                "insufficient template support rejects entire reconstruction");
        require(run(boxCloud(),3,{planeHint}).outcome==CadReconstructionResult::Outcome::Cancelled,
                "cancellation also wins during guided fitting and queued delivery");

        auto mixed=boxCloud(); reconstruction::TemplateHint sphere{8,reconstruction::TemplateKind::Sphere,{}};
        const double pi=std::acos(-1.0);
        for(int z=0;z<12;++z) for(int a=0;a<48;++a) {
            const double nz=-1+(z+0.5)*2/12,angle=(a+0.5)*2*pi/48,r=std::sqrt(1-nz*nz);
            std::array<float,3> n{static_cast<float>(r*std::cos(angle)),static_cast<float>(r*std::sin(angle)),static_cast<float>(nz)};
            mixed.positions.push_back({20+2*n[0],2*n[1],2*n[2]}); mixed.normals.push_back(n);
            sphere.points.push_back(mixed.positions.size());
        }
        const auto continued=run(mixed,0,{sphere});
        if(continued.outcome!=CadReconstructionResult::Outcome::Success) throw std::runtime_error(continued.error.toStdString());
        require(continued.candidate.boxes.size()==1 && continued.candidate.curvedSolids.size()==1,
                "fixed manual sphere is retained while remaining box is reconstructed");
        auto spherePatch = sphere;
        spherePatch.points.resize(96);
        const auto partialSphere = run(mixed, 0, {spherePatch});
        if (partialSphere.outcome != CadReconstructionResult::Outcome::Success)
            throw std::runtime_error(partialSphere.error.toStdString());
        require(partialSphere.candidate.boxes.size() == 1 && partialSphere.candidate.curvedSolids.size() == 1,
                "local sphere patch completes from global observations before reconstructing remaining box");
        auto firstSphere = sphere, secondSphere = sphere;
        firstSphere.regionId = 9; secondSphere.regionId = 10;
        firstSphere.points.clear(); secondSphere.points.clear();
        for (auto id : sphere.points) (id % 2 ? firstSphere : secondSphere).points.push_back(id);
        const auto shared = run(mixed, 0, {firstSphere, secondSphere});
        require(shared.outcome == CadReconstructionResult::Outcome::Success
                && shared.templateFits.size() == 2 && shared.candidate.curvedSolids.size() == 1,
                "two manual groups observing the same closed sphere do not create duplicate solids");
        for (bool cone : {false, true}) {
            auto capped = cappedCloud(cone);
            reconstruction::TemplateHint hint{11, cone ? reconstruction::TemplateKind::Cone
                : reconstruction::TemplateKind::Cylinder, {}};
            for (size_t i = 0; i < capped.positions.size(); ++i) hint.points.push_back(i + 1);
            const auto complete = run(capped, 0, {hint});
            if (complete.outcome != CadReconstructionResult::Outcome::Success)
                throw std::runtime_error(complete.error.toStdString());
            require(complete.templateFits[0].solid && complete.candidate.curvedSolids.size() == 1,
                    "capped cylinder/cone templates produce complete observed solids");
            hint.points.resize(576);
            const auto side = run(capped, 0, {hint});
            if (side.outcome != CadReconstructionResult::Outcome::Success)
                throw std::runtime_error(side.error.toStdString());
            require(side.templateFits[0].surface && side.candidate.curvedSolids.size() == 1,
                    "partial curved side seeds global reconstruction using unassigned observed caps");
            auto scene = boxCloud();
            auto local = hint;
            for (auto& id : local.points) id += scene.positions.size();
            for (auto p : capped.positions) { p[0] += 20; scene.positions.push_back(p); }
            scene.normals.insert(scene.normals.end(), capped.normals.begin(), capped.normals.end());
            const auto joined = run(scene, 0, {local});
            if (joined.outcome != CadReconstructionResult::Outcome::Success)
                throw std::runtime_error(joined.error.toStdString());
            require(joined.candidate.boxes.size() == 1 && joined.candidate.curvedSolids.size() == 1,
                    "local cylinder/cone patch completes observed component before continuing with separate box");
            capped.positions.resize(576); capped.normals.resize(576);
            require(run(capped, 0, {hint}).outcome == CadReconstructionResult::Outcome::Unsupported,
                    "missing curved caps are rejected instead of invented");
        }
        PlyCloud ring;
        reconstruction::TemplateHint ringHint{12, reconstruction::TemplateKind::Torus, {}};
        for (int a = 0; a < 48; ++a) for (int b = 0; b < 24; ++b) {
            const double u = (a + 0.5) * 2 * pi / 48, v = (b + 0.5) * 2 * pi / 24;
            ring.positions.push_back({float((4 + std::cos(v)) * std::cos(u)),
                float((4 + std::cos(v)) * std::sin(u)), float(std::sin(v))});
            ring.normals.push_back({float(std::cos(v) * std::cos(u)),
                float(std::cos(v) * std::sin(u)), float(std::sin(v))});
            ringHint.points.push_back(ring.positions.size());
        }
        const auto ringResult = run(ring, 0, {ringHint});
        if (ringResult.outcome != CadReconstructionResult::Outcome::Success)
            throw std::runtime_error(ringResult.error.toStdString());
        require(ringResult.templateFits[0].solid && ringResult.candidate.curvedSolids.size() == 1,
                "manual ring template reconstructs a complete torus");
        auto ringScene = boxCloud();
        auto ringPatch = ringHint;
        ringPatch.points.resize(96);
        for (auto& id : ringPatch.points) id += ringScene.positions.size();
        for (auto p : ring.positions) { p[0] += 20; ringScene.positions.push_back(p); }
        ringScene.normals.insert(ringScene.normals.end(), ring.normals.begin(), ring.normals.end());
        const auto partialRing = run(ringScene, 0, {ringPatch});
        if (partialRing.outcome != CadReconstructionResult::Outcome::Success)
            throw std::runtime_error(partialRing.error.toStdString());
        require(partialRing.candidate.boxes.size() == 1 && partialRing.candidate.curvedSolids.size() == 1,
                "local torus patch completes from global observations before continuing with separate box");
        require(run(boxCloud(), 1).outcome == CadReconstructionResult::Outcome::Cancelled,
                "cancellation before computation prevents success");
        require(run(boxCloud(), 2).outcome == CadReconstructionResult::Outcome::Cancelled,
                "cancellation wins over pending successful delivery");
        PlyCloud plane;
        for (int x = 0; x < 10; ++x) for (int y = 0; y < 10; ++y) {
            plane.positions.push_back({static_cast<float>(x), static_cast<float>(y), 0});
            plane.normals.push_back({0, 0, 1});
        }
        const auto unsupported = run(plane);
        require(unsupported.outcome == CadReconstructionResult::Outcome::Unsupported
                && !unsupported.error.isEmpty(), "unsupported geometry reported without throwing");
        PlyCloud invalid;
        invalid.positions.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
        require(run(invalid).outcome == CadReconstructionResult::Outcome::PreprocessingFailed,
                "invalid observations produce preprocessing failure");

        // Destruction joins a worker even without an event loop delivering results.
        { CadReconstructionTask pending;
          require(pending.start(boxCloud(), reconstruction::LengthUnit::Millimeter), "start shutdown task"); }
        QCoreApplication::processEvents();
        // Optional paths replay real metric scans through the background task.
        for (int i = 1; i < argc; ++i) {
            PlyCloud scan;
            QString error;
            require(PlyReader::read(QString::fromLocal8Bit(argv[i]), scan, error), "read saved scan");
            const auto result = run(scan);
            require(result.outcome == CadReconstructionResult::Outcome::Success, "background task reconstructs saved scan");
            std::cout << "Saved scan: " << result.preprocessing.inputPointCount
                      << " points, " << result.candidate.boxes.size() << " boxes, "
                      << result.candidate.prisms.size() << " prisms, "
                      << result.candidate.throughHoles.size() << " holes.\n";
            require(!result.candidate.boxes.empty(), "saved mechanical scan has a box frame");
            const auto& frame = result.candidate.boxes[0].primitive.pose;
            reconstruction::TemplateHint patch{20, reconstruction::TemplateKind::Plane, {}};
            std::vector<reconstruction::PointId> eligible;
            for (size_t p = 0; p < scan.positions.size(); ++p) {
                const auto& q = scan.positions[p]; const auto& n = scan.normals[p];
                const double offset = (q[0] - frame.origin.x) * frame.xDirection.x
                    + (q[1] - frame.origin.y) * frame.xDirection.y
                    + (q[2] - frame.origin.z) * frame.xDirection.z;
                const double alignment = n[0] * frame.xDirection.x + n[1] * frame.xDirection.y
                    + n[2] * frame.xDirection.z;
                if (std::abs(offset) < 0.1 && alignment < -0.95) eligible.push_back(p + 1);
            }
            const auto count = std::min<size_t>(300, eligible.size());
            for (size_t p = 0; p < count; ++p) patch.points.push_back(eligible[p * eligible.size() / count]);
            require(patch.points.size() >= 20, "saved scan exposes an observed planar patch");
            const auto guidedScan = run(std::move(scan), 0, {patch});
            if (guidedScan.outcome != CadReconstructionResult::Outcome::Success)
                throw std::runtime_error(guidedScan.error.toStdString());
            require(guidedScan.candidate.boxes.size() == result.candidate.boxes.size()
                    && guidedScan.candidate.prisms.size() == result.candidate.prisms.size()
                    && guidedScan.candidate.throughHoles.size() == result.candidate.throughHoles.size(),
                    "manual patch continues whole saved scan reconstruction with all original components");
            std::cout << "Guided scan: " << patch.points.size() << " manual plane points, "
                      << guidedScan.preprocessing.outputPointCount << " retained source observations.\n";
        }
        std::cout << "CAD reconstruction task tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
