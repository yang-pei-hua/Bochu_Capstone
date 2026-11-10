#include "ModuleCli.h"

#include <reconstruction/detection/PrimitiveDetector.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonObject>

#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace {

QJsonArray vectorJson(const modeling::Vec3& value)
{
    return QJsonArray{value.x, value.y, value.z};
}

QJsonObject evidenceJson(const reconstruction::PrimitiveEvidence& evidence)
{
    QJsonObject output;
    std::visit(
        [&](const auto& value) {
            using Evidence = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Evidence, reconstruction::PlaneEvidence>) {
                output["kind"] = QStringLiteral("plane");
                output["normal"] = vectorJson(value.plane.normal);
                output["offset"] = value.plane.offset;
            } else if constexpr (std::is_same_v<Evidence, reconstruction::CylinderEvidence>) {
                output["kind"] = QStringLiteral("cylinder");
                output["axis_point"] = vectorJson(value.cylinder.axisPoint);
                output["axis_direction"] = vectorJson(value.cylinder.axisDirection);
                output["radius"] = value.cylinder.radius;
                output["axial_min"] = value.axialMinimum;
                output["axial_max"] = value.axialMaximum;
            } else if constexpr (std::is_same_v<Evidence, reconstruction::SphereEvidence>) {
                output["kind"] = QStringLiteral("sphere");
                output["center"] = vectorJson(value.sphere.center);
                output["radius"] = value.sphere.radius;
            } else if constexpr (std::is_same_v<Evidence, reconstruction::ConeEvidence>) {
                output["kind"] = QStringLiteral("cone");
                output["apex"] = vectorJson(value.cone.apex);
                output["axis_direction"] = vectorJson(value.cone.axisDirection);
                output["opening_angle_rad"] = value.cone.openingAngleRadians;
            } else {
                output["kind"] = QStringLiteral("torus");
                output["center"] = vectorJson(value.torus.center);
                output["axis_direction"] = vectorJson(value.torus.axisDirection);
                output["major_radius"] = value.torus.majorRadius;
                output["minor_radius"] = value.torus.minorRadius;
            }
            output["support_points"] = static_cast<qint64>(value.supportPointIds.size());
            output["coverage"] = value.coverage;
            output["confidence"] = value.confidence;
        },
        evidence);
    return output;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    const QString usage = QStringLiteral(
        "Usage: pcad_detect (--in cloud.ply | --synthetic box) [--report report.json] "
        "[--unit mm|m|arbitrary] [--config file] [--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("detection", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::detectionConfigPath() : arguments.config;
        auto document = reconstruction::makeDetectionSettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("detection");
        report["config"] = QString::fromStdU16String(config.u16string());
        report["settings"] = moduleSettingsJson(*document);

        QString error;
        auto cloud = loadModuleCloud(arguments, error);
        if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
        report["input"] = arguments.synthetic.isEmpty() ? arguments.input : QStringLiteral("synthetic:box");
        report["unit"] = lengthUnitName(cloud.store.unit());
        report["input_points"] = static_cast<qint64>(cloud.ply.positions.size());

        const auto preprocessingSettings = reconstruction::loadPreprocessingSettings();
        const auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud.ply, preprocessingSettings);
        reconstruction::ObservationSnapshot snapshot(std::move(cloud.store));
        reconstruction::PointStore processed({}, snapshot.observations().unit());
        reconstruction::PointCloudPreprocessingReport preprocessingReport;
        std::string failure;
        if (!reconstruction::preprocessPointCloud(snapshot, preprocessing, processed, preprocessingReport, failure))
            throw std::runtime_error(failure);

        const auto settings = reconstruction::loadDetectionSettings(config);
        auto options = settings.options;
        PointCloudAdapter::applyDetectionScale(options, cloudDiagonal(processed), processed.size(), settings);
        reconstruction::PrimitiveDetectionResult detection;
        if (!reconstruction::detectPrimitiveEvidence(processed, options, detection, failure))
            throw std::runtime_error(failure);

        QJsonArray evidence;
        for (const auto& entry : detection.evidence) evidence.append(evidenceJson(entry));
        report["working_points"] = static_cast<qint64>(processed.size());
        report["evidence"] = evidence;
        report["evidence_count"] = static_cast<qint64>(detection.evidence.size());
        report["unassigned_points"] = static_cast<qint64>(detection.unassignedPointIds.size());
        report["elapsed_ms"] = timer.elapsed();
        report["status"] = QStringLiteral("ok");
        writeModuleReport(arguments.report, report);
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
