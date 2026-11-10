#include "ModuleCli.h"

#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <reconstruction/recognition/ProjectionSettings.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonObject>

#include <iostream>
#include <stdexcept>

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    const QString usage = QStringLiteral(
        "Usage: pcad_assemble (--in cloud.ply | --synthetic box) [--report report.json] "
        "[--unit mm|m] [--config assembly.conf] [--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("assembly", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::assemblyConfigPath() : arguments.config;
        auto document = reconstruction::makeAssemblySettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("assembly");
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
            throw std::runtime_error("Assembly needs a millimetre cloud; pass --unit mm or provide mm metadata");

        reconstruction::AssemblyOptions options = reconstruction::loadAssemblySettings(config);
        options.projectionOptions = reconstruction::loadProjectionOptions();
        reconstruction::AssemblyReconstructionResult assembly;
        if (!reconstruction::reconstructAssembly(processed, options, assembly, failure))
            throw std::runtime_error(failure);

        QJsonArray parts;
        for (const auto& part : assembly.parts) {
            QJsonObject entry;
            entry["id"] = static_cast<qint64>(part.id);
            entry["closed"] = part.closed;
            entry["status"] = QString::fromStdString(part.status);
            entry["points"] = static_cast<qint64>(part.pointIds.size());
            entry["working_points"] = static_cast<qint64>(part.workingPointIds.size());
            entry["patches"] = static_cast<qint64>(part.patchIds.size());
            entry["open_edges"] = static_cast<qint64>(part.openEdges);
            entry["nonmanifold_edges"] = static_cast<qint64>(part.nonmanifoldEdges);
            entry["confidence"] = part.confidence;
            parts.append(entry);
        }
        QJsonObject candidate;
        candidate["boxes"] = static_cast<qint64>(assembly.candidate.boxes.size());
        candidate["prisms"] = static_cast<qint64>(assembly.candidate.prisms.size());
        candidate["through_holes"] = static_cast<qint64>(assembly.candidate.throughHoles.size());
        candidate["curved_solids"] = static_cast<qint64>(assembly.candidate.curvedSolids.size());
        candidate["profile_bodies"] = static_cast<qint64>(assembly.candidate.profileBodies.size());
        candidate["projection_programs"] = static_cast<qint64>(assembly.candidate.projectionPrograms.size());
        candidate["surface_coverage"] = assembly.candidate.surfaceCoverage;
        report["parts"] = parts;
        report["candidate"] = candidate;
        report["patches"] = static_cast<qint64>(assembly.patches.size());
        report["unassigned_points"] = static_cast<qint64>(assembly.unassignedPointIds.size());
        report["working_points"] = static_cast<qint64>(assembly.workingPoints);
        report["elapsed_ms"] = timer.elapsed();
        report["status"] = assembly.parts.empty() ? QStringLiteral("unsupported") : QStringLiteral("ok");
        writeModuleReport(arguments.report, report);
        return assembly.parts.empty() ? 1 : 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
