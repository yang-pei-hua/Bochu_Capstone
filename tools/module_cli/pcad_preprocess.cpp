#include "ModuleCli.h"

#include "io/PlyWriter.h"

#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
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
        "Usage: pcad_preprocess (--in cloud.ply | --synthetic box) --out cloud-out.ply "
        "[--report report.json] [--unit mm|m|arbitrary] [--config file] "
        "[--set key=value] [--print-config]");
    try {
        const auto arguments = parseModuleArguments(QCoreApplication::arguments(), usage);
        prepareModuleOverrides("preprocessing", arguments.overrides);
        const auto config = arguments.config.empty()
            ? reconstruction::preprocessingConfigPath() : arguments.config;
        auto document = reconstruction::makePreprocessingSettings();
        document->load(config);
        document->applyOverrides();
        if (arguments.printConfig) {
            std::cout << document->serialize();
            return 0;
        }
        if (arguments.output.isEmpty())
            throw std::invalid_argument("--out is required");

        QElapsedTimer timer;
        timer.start();
        QJsonObject report;
        report["module"] = QStringLiteral("preprocessing");
        report["config"] = QString::fromStdU16String(config.u16string());
        report["settings"] = moduleSettingsJson(*document);

        QString error;
        auto cloud = loadModuleCloud(arguments, error);
        if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
        report["input"] = arguments.synthetic.isEmpty() ? arguments.input : QStringLiteral("synthetic:box");
        report["unit"] = lengthUnitName(cloud.store.unit());
        report["input_points"] = static_cast<qint64>(cloud.ply.positions.size());

        const auto settings = reconstruction::loadPreprocessingSettings(config);
        const auto options = PointCloudAdapter::preprocessingOptionsFor(cloud.ply, settings);
        reconstruction::ObservationSnapshot snapshot(std::move(cloud.store));
        reconstruction::PointStore processed({}, snapshot.observations().unit());
        reconstruction::PointCloudPreprocessingReport preprocessing;
        std::string failure;
        if (!reconstruction::preprocessPointCloud(snapshot, options, processed, preprocessing, failure))
            throw std::runtime_error(failure);

        PlyCloud outputCloud;
        outputCloud.positions.reserve(processed.size());
        for (const auto& point : processed.points()) {
            outputCloud.positions.push_back({float(point.position.x), float(point.position.y),
                                             float(point.position.z)});
            if (point.normal)
                outputCloud.normals.push_back({float(point.normal->x), float(point.normal->y),
                                               float(point.normal->z)});
        }
        if (!PlyWriter::write(outputCloud, arguments.output, error))
            throw std::runtime_error(error.toStdString());

        QJsonObject details;
        details["applied_scale_to_millimeters"] = preprocessing.appliedScaleToMillimeters;
        details["voxel_removed"] = static_cast<qint64>(preprocessing.removedByVoxel);
        details["outliers_removed"] = static_cast<qint64>(preprocessing.removedAsOutliers);
        details["normals_from_input"] = static_cast<qint64>(preprocessing.normalsFromInput);
        details["normals_estimated"] = static_cast<qint64>(preprocessing.normalsEstimated);
        details["normals_oriented"] = static_cast<qint64>(preprocessing.estimatedNormalsOriented);
        details["normals_without_orientation"] = static_cast<qint64>(preprocessing.normalsWithoutOrientation);
        report["preprocessing"] = details;
        report["output"] = arguments.output;
        report["output_points"] = static_cast<qint64>(processed.size());
        report["elapsed_ms"] = timer.elapsed();
        report["status"] = QStringLiteral("ok");
        writeModuleReport(arguments.report, report);
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        return 2;
    }
}
