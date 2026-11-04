#include "ReconstructionContractStages.h"
#include "ReconstructionContractData.h"
#include "io/PointCloudAdapter.h"

#include <QCryptographicHash>
#include <QFile>
#include <QJsonDocument>
#include <QSaveFile>

namespace {
QJsonObject readObject(const QString& path) {
    QFile file(path);
    contract::require(file.open(QIODevice::ReadOnly), "Cannot read contract JSON");
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(file.readAll(), &error);
    contract::require(error.error == QJsonParseError::NoError && document.isObject(), "Invalid contract JSON object");
    return document.object();
}

void saveObject(const QString& path, const QJsonObject& object) {
    QSaveFile file(path);
    contract::require(file.open(QIODevice::WriteOnly), "Cannot open contract artifact");
    const auto bytes = QJsonDocument(object).toJson(QJsonDocument::Compact);
    contract::require(file.write(bytes) == bytes.size() && file.commit(), "Cannot commit contract artifact");
}
}

void runReconstructionContractStage(const QStringList& args, QJsonObject& report) {
    using namespace reconstruction;
    using contract::require;
    require(args.size() == 4, "Usage: contract-stage request.json report.json");
    const auto request = readObject(args[2]);
    const int module = request["module"].toInt();
    report["module"] = module;
    const bool backfill = request["phase"].toString() == "backfill";
    require(!backfill || module == 19, "Backfill belongs to assembly module 19");
    if (backfill) report["phase"] = "backfill";
    require(module >= 1 && module <= 34, "Unsupported contract module");
    const auto unit = request["unit"].toString();
    require(unit == "mm" || unit == "m", "Contract point cloud requires mm or m units");
    PlyCloud cloud;
    QString error;
    if (!PlyReader::read(request["cloud"].toString(), cloud, error))
        throw std::runtime_error(error.toStdString());
    const auto sourceUnit = unit == "m" ? LengthUnit::Meter : LengthUnit::Millimeter;
    const ObservationSnapshot source(PointCloudAdapter::toPointStore(cloud, sourceUnit));
    const auto fingerprint = QString::number(source.fingerprint());
    const auto artifactPath = request["artifact"].toString();
    require(!artifactPath.isEmpty(), "Missing contract artifact destination");
    const auto inputPath = request["input"].toString();
    QJsonObject input;
    const bool rootStage = module == 1 || module == 18 || module == 22 || module == 34;
    int predecessor = rootStage ? 0 : module - 1;
    if (backfill) predecessor = 20;
    if (module == 6 || module == 15 || module == 17) predecessor = 4;
    if (module == 12 || module == 14) predecessor = 7;
    if (module == 16) predecessor = 8;
    if (!inputPath.isEmpty()) {
        input = readObject(inputPath);
        require(input["schema"].toString() == "paramcad.contract-stage" && input["version"].toInt() == 1
            && input["module"].toInt(-1) == predecessor, "Artifact is for a different upstream stage");
        if (input.contains("source_fingerprint")) {
            require(input["source_fingerprint"].toString() == fingerprint, "Artifact belongs to a different point snapshot");
        } else {
            QFile file(request["cloud"].toString());
            QCryptographicHash digest(QCryptographicHash::Sha256);
            require(file.open(QIODevice::ReadOnly) && digest.addData(&file) &&
                QString::fromLatin1(digest.result().toHex()) == input["source_sha256"].toString() &&
                input["source_unit"].toString() == unit, "Reference artifact belongs to a different cloud or unit");
        }
    } else {
        require(rootStage, "Missing upstream artifact");
    }
    QJsonObject metrics;
    const auto options = request["options"].toObject();
    QJsonObject artifact;
    if (module <= 17) {
        artifact = contract::analysisStage(module, source, options, std::move(input), metrics,
            cloud.positions.size() - source.observations().size());
    } else if (module <= 21) {
        artifact = contract::assemblyStage(module, source.observations(), options, std::move(input), metrics, backfill);
    } else if (module <= 27) {
        artifact = contract::projectionStage(module, source.observations(), options, std::move(input), metrics);
    } else if (module <= 33) {
        artifact = contract::modelingStage(module, std::move(input), metrics, artifactPath);
    } else {
        artifact = contract::workflowStage(std::move(cloud), sourceUnit, options, std::move(input), metrics);
    }
    artifact["schema"] = "paramcad.contract-stage";
    artifact["version"] = 1;
    artifact["module"] = backfill ? 20 : module;
    artifact["source_fingerprint"] = fingerprint;
    saveObject(artifactPath, artifact);
    report["metrics"] = metrics;
    report["status"] = "contract_stage_completed";
}
