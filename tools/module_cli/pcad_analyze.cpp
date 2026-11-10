#include "ModuleCli.h"

#include <reconstruction/analysis/SurfaceAnalysis.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonObject>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

QString kindName(reconstruction::PrimitiveKind kind)
{
    switch (kind) {
    case reconstruction::PrimitiveKind::Plane: return QStringLiteral("plane");
    case reconstruction::PrimitiveKind::Cylinder: return QStringLiteral("cylinder");
    case reconstruction::PrimitiveKind::Sphere: return QStringLiteral("sphere");
    case reconstruction::PrimitiveKind::Cone: return QStringLiteral("cone");
    case reconstruction::PrimitiveKind::Torus: return QStringLiteral("torus");
    }
    return QStringLiteral("unknown");
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    const QString usage = QStringLiteral(
        "Usage: pcad_analyze (--in cloud.ply | --synthetic box) [--report report.json] "
        "[--unit mm|m|arbitrary] [--config analysis.conf] [--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("analysis", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::analysisConfigPath() : arguments.config;
        auto document = reconstruction::makeAnalysisSettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("analysis");
        report["config"] = QString::fromStdU16String(config.u16string());
        report["settings"] = moduleSettingsJson(*document);

        QString error;
        auto cloud = loadModuleCloud(arguments, error);
        if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
        report["input"] = arguments.synthetic.isEmpty() ? arguments.input : QStringLiteral("synthetic:box");
        report["unit"] = lengthUnitName(cloud.store.unit());

        const auto preprocessingSettings = reconstruction::loadPreprocessingSettings();
        const auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud.ply, preprocessingSettings);
        reconstruction::ObservationSnapshot snapshot(std::move(cloud.store));
        reconstruction::PointStore processed({}, snapshot.observations().unit());
        reconstruction::PointCloudPreprocessingReport preprocessingReport;
        std::string failure;
        if (!reconstruction::preprocessPointCloud(snapshot, preprocessing, processed, preprocessingReport, failure))
            throw std::runtime_error(failure);

        const auto detectionSettings = reconstruction::loadDetectionSettings();
        auto detection = detectionSettings.options;
        PointCloudAdapter::applyDetectionScale(detection, cloudDiagonal(processed), processed.size(), detectionSettings);

        const auto settings = reconstruction::loadAnalysisSettings(config);
        reconstruction::SurfaceAnalysisOptions options;
        options.detection = detection;
        options.relations.distanceTolerance = settings.relationDistanceMm > 0
            ? settings.relationDistanceMm
            : detection.plane.distanceThreshold * settings.relationDistanceToleranceMultiplier;
        options.relations.angularToleranceRadians =
            settings.relationAngularToleranceDegrees * 3.14159265358979323846 / 180.0;
        options.connectivityDistance = settings.connectivityDistance;
        options.independentFamilies = settings.independentFamilies;
        options.recoverResidualSurfaces = settings.recoverResidualSurfaces;

        reconstruction::SurfaceAnalysisResult analysis;
        if (!reconstruction::analyzeSurfaces(processed, options, analysis, failure))
            throw std::runtime_error(failure);

        QJsonArray surfaces;
        for (const auto& surface : analysis.surfaces) {
            QJsonObject entry;
            entry["kind"] = kindName(reconstruction::primitiveKind(surface));
            entry["support_points"] = static_cast<qint64>(std::visit(
                [](const auto& value) { return value.supportPointIds.size(); }, surface));
            entry["confidence"] = std::visit([](const auto& value) { return value.confidence; }, surface);
            surfaces.append(entry);
        }
        QJsonArray patches;
        for (const auto& patch : analysis.patches) {
            QJsonObject entry;
            entry["id"] = static_cast<qint64>(patch.id);
            entry["surface_id"] = static_cast<qint64>(patch.surfaceId);
            entry["support_points"] = static_cast<qint64>(patch.supportPointIds.size());
            entry["orientation_known"] = patch.orientationKnown;
            patches.append(entry);
        }
        QJsonArray relations;
        for (const auto& relation : analysis.relations) {
            QJsonObject entry;
            entry["first"] = static_cast<qint64>(relation.first);
            entry["second"] = static_cast<qint64>(relation.second);
            entry["kind"] = QString::fromLatin1(reconstruction::surfaceRelationName(relation.kind));
            entry["error"] = relation.error;
            entry["confidence"] = relation.confidence;
            relations.append(entry);
        }
        report["working_points"] = static_cast<qint64>(processed.size());
        report["surfaces"] = surfaces;
        report["patches"] = patches;
        report["relations"] = relations;
        report["unassigned_points"] = static_cast<qint64>(analysis.unassignedPointIds.size());
        report["elapsed_ms"] = timer.elapsed();
        report["status"] = QStringLiteral("ok");
        writeModuleReport(arguments.report, report);
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
