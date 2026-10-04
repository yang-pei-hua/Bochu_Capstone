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

CadReconstructionResult run(PlyCloud cloud, int cancellation = 0)
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
        });
    require(task.start(cloud, reconstruction::LengthUnit::Millimeter), "start worker");
    cloud.positions.clear();
    cloud.normals.clear();
    require(task.isRunning() && !result, "start returns before queued result delivery");
    require(!task.start({}, reconstruction::LengthUnit::Millimeter), "duplicate start rejected");
    if (cancellation == 2) {
        require(task.findChild<QThread*>()->wait(30000), "worker finished before queued result is delivered");
    }
    if (cancellation) {
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
        require(std::find(stages.begin(), stages.end(), CadReconstructionTask::Stage::Recognizing) != stages.end(),
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
            const auto result = run(std::move(scan));
            require(result.outcome == CadReconstructionResult::Outcome::Success, "background task reconstructs saved scan");
            std::cout << "Saved scan: " << result.preprocessing.inputPointCount
                      << " points, " << result.candidate.boxes.size() << " boxes, "
                      << result.candidate.prisms.size() << " prisms, "
                      << result.candidate.throughHoles.size() << " holes.\n";
        }
        std::cout << "CAD reconstruction task tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
