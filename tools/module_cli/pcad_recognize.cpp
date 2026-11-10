#include "ModuleCli.h"

#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/recognition/GuidedSolidRecognizer.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonObject>

#include <iostream>
#include <stdexcept>

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    const QString usage = QStringLiteral(
        "Usage: pcad_recognize (--in cloud.ply | --synthetic box) [--report report.json] "
        "[--unit mm|m] [--config recognition.conf] [--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("recognition", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::recognitionConfigPath() : arguments.config;
        auto document = reconstruction::makeRecognitionSettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("recognition");
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
        if (processed.unit() != reconstruction::LengthUnit::Millimeter)
            throw std::runtime_error("Recognition needs a millimetre cloud; pass --unit mm or provide mm metadata");

        const auto detectionSettings = reconstruction::loadDetectionSettings();
        auto detection = detectionSettings.options;
        PointCloudAdapter::applyDetectionScale(detection, cloudDiagonal(processed), processed.size(), detectionSettings);

        const auto settings = reconstruction::loadRecognitionSettings(config);
        reconstruction::GuidedSolidResult guided;
        const auto& source = snapshot.observations();
        if (!reconstruction::reconstructGuidedSolid(
                processed, detection.plane, detection.cylinder, {}, guided, failure, {},
                nullptr, {}, &source, &settings)) {
            report["error"] = QString::fromStdString(failure);
            report["status"] = QStringLiteral("unsupported");
            report["elapsed_ms"] = timer.elapsed();
            writeModuleReport(arguments.report, report);
            return 1;
        }

        QJsonObject candidate;
        candidate["boxes"] = static_cast<qint64>(guided.candidate.boxes.size());
        candidate["prisms"] = static_cast<qint64>(guided.candidate.prisms.size());
        candidate["through_holes"] = static_cast<qint64>(guided.candidate.throughHoles.size());
        candidate["curved_solids"] = static_cast<qint64>(guided.candidate.curvedSolids.size());
        candidate["profile_bodies"] = static_cast<qint64>(guided.candidate.profileBodies.size());
        candidate["projection_programs"] = static_cast<qint64>(guided.candidate.projectionPrograms.size());
        candidate["surface_coverage"] = guided.candidate.surfaceCoverage;
        candidate["confidence"] = guided.candidate.confidence;
        report["candidate"] = candidate;
        report["analysis_reused"] = guided.analysisReused;
        report["working_points"] = static_cast<qint64>(processed.size());
        report["elapsed_ms"] = timer.elapsed();
        report["status"] = QStringLiteral("ok");
        writeModuleReport(arguments.report, report);
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
