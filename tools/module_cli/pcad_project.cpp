#include "ModuleCli.h"

#include <modeling/ModelingCore.h>
#include <modeling/StepExporter.h>
#include <reconstruction/recognition/ProjectionExtrusion.h>
#include <reconstruction/recognition/ProjectionSettings.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonObject>

#include <iostream>
#include <stdexcept>

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    const QString usage = QStringLiteral(
        "Usage: pcad_project (--in cloud.ply | --synthetic box) [--report report.json] "
        "[--step out.step] [--unit mm|m|arbitrary] [--path auto|single-axis|two-axis-intersection|segmented] "
        "[--config projection.conf] [--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("projection", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::projectionConfigPath() : arguments.config;
        auto document = reconstruction::makeProjectionSettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("projection");
        report["config"] = QString::fromStdU16String(config.u16string());
        report["settings"] = moduleSettingsJson(*document);

        QString error;
        auto cloud = loadModuleCloud(arguments, error);
        if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
        report["input"] = arguments.synthetic.isEmpty() ? arguments.input : QStringLiteral("synthetic:box");
        report["unit"] = lengthUnitName(cloud.store.unit());
        report["input_points"] = static_cast<qint64>(cloud.store.size());

        auto options = reconstruction::loadProjectionOptions(config);
        using Path = reconstruction::ProjectionPath;
        if (!arguments.path.isEmpty()) {
            if (arguments.path == QStringLiteral("single-axis")) options.path = Path::SingleAxis;
            else if (arguments.path == QStringLiteral("two-axis-intersection")) options.path = Path::TwoAxisIntersection;
            else if (arguments.path == QStringLiteral("segmented")) options.path = Path::Segmented;
            else if (arguments.path == QStringLiteral("auto")) options.path = Path::Automatic;
            else throw std::invalid_argument("Unknown --path value");
        }

        reconstruction::ObservationSnapshot snapshot(std::move(cloud.store));
        reconstruction::ProjectionExtrusionResult fit;
        const bool accepted = reconstruction::fitProjectionExtrusion(
            snapshot.observations(), options, fit, {}, {});

        QJsonObject quality;
        quality["scored_points"] = static_cast<qint64>(fit.quality.scoredPoints);
        quality["median_distance"] = fit.quality.medianDistance;
        quality["p95_distance"] = fit.quality.p95Distance;
        quality["surface_inlier_fraction"] = fit.quality.surfaceInlierFraction;
        quality["unsupported_face_area_fraction"] = fit.quality.unsupportedFaceAreaFraction;
        quality["mean_normal_error_degrees"] = fit.quality.meanNormalErrorDegrees;
        quality["score"] = fit.quality.score;
        report["accepted"] = accepted;
        report["reason"] = QString::fromStdString(fit.reason);
        report["distance_tolerance"] = fit.distanceTolerance;
        report["operations"] = static_cast<qint64>(fit.operations.size());
        report["axes"] = static_cast<qint64>(fit.axes.size());
        report["quality"] = quality;

        if (!arguments.step.isEmpty()) {
            if (!accepted) throw std::runtime_error("Refusing to export STEP from a rejected projection fit");
            modeling::ModelingCore model;
            std::string failure;
            if (!reconstruction::buildProjectionProgram(fit.operations, model, failure))
                throw std::runtime_error(failure);
            if (!modeling::exportStep(model.bodyShape(), std::filesystem::path(arguments.step.toStdU16String()), &failure))
                throw std::runtime_error(failure);
            report["step"] = arguments.step;
        }

        report["elapsed_ms"] = timer.elapsed();
        report["status"] = accepted ? QStringLiteral("ok") : QStringLiteral("unsupported");
        writeModuleReport(arguments.report, report);
        return accepted ? 0 : 1;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
