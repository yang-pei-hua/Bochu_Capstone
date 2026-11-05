#include "ReconstructionContractData.h"

#include "app/CadReconstructionTask.h"
#include "io/FeatureCodec.h"
#include <reconstruction/modeling/ModelingAdapter.h>

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <QEventLoop>
#include <QThread>
#include <unordered_set>

namespace contract {
namespace {
PlyCloud selectObservations(PlyCloud source, const QJsonObject& input) {
    if (input.isEmpty()) return source;
    PlyCloud selected;
    const bool hasNormals = source.normals.size() == source.positions.size();
    const bool hasColors = source.colors.size() == source.positions.size();
    std::unordered_set<PointId> seen;
    for (const auto id : ids(input["observation_ids"])) {
        require(id <= source.positions.size() && seen.insert(id).second, "Invalid or duplicate workflow observation ID");
        const auto row = static_cast<std::size_t>(id - 1);
        selected.sourceRows.push_back(row);
        selected.positions.push_back(source.positions[row]);
        if (hasNormals) selected.normals.push_back(source.normals[row]);
        if (hasColors) selected.colors.push_back(source.colors[row]);
    }
    require(!selected.positions.empty(), "Empty workflow observation selection");
    return selected;
}

CadReconstructionResult runTask(PlyCloud cloud, reconstruction::LengthUnit unit, double tolerance,
    bool cancel, bool& ownerThread) {
    CadReconstructionTask task;
    QEventLoop loop;
    std::optional<CadReconstructionResult> result;
    QObject::connect(&task, &CadReconstructionTask::finished, &loop, [&](const CadReconstructionResult& value) {
        ownerThread = QThread::currentThread() == task.thread();
        result = value;
        loop.quit();
    });
    require(task.start(std::move(cloud), unit, {}, tolerance), "Cannot start production reconstruction task");
    if (cancel) task.cancel();
    if (!result) loop.exec();
    require(result.has_value(), "Production task returned no result");
    return std::move(*result);
}
} // namespace

QJsonObject workflowStage(PlyCloud cloud, reconstruction::LengthUnit unit, const QJsonObject& settings,
    QJsonObject input, QJsonObject& metrics) {
    auto active = selectObservations(std::move(cloud), input);
    const double tolerance = settings["workflow_tolerance_source"].toDouble();
    bool owner = false;
    auto result = runTask(active, unit, tolerance, false, owner);
    if (result.outcome != CadReconstructionResult::Outcome::Success)
        throw std::runtime_error(result.error.toStdString());
    using Branch = CadReconstructionResult::Branch;
    switch (result.branch) {
    case Branch::Conventional: metrics["branch"] = "conventional"; break;
    case Branch::Guided: metrics["branch"] = "guided"; break;
    case Branch::Assembly: metrics["branch"] = "assembly"; break;
    case Branch::None: metrics["branch"] = "none"; break;
    }
    bool cancelledOnOwner = false;
    const auto cancelled = runTask(std::move(active), unit, tolerance, true, cancelledOnOwner);
    metrics["cancelled"] = cancelled.outcome == CadReconstructionResult::Outcome::Cancelled;
    metrics["owner_thread_delivery"] = owner && cancelledOnOwner;
    metrics["partial_assembly"] = result.partialAssembly;
    metrics["preprocessed_points"] = qint64(result.preprocessing.outputPointCount);
    modeling::ModelingCore model;
    const auto commit = reconstruction::commitCompoundCandidate(result.candidate, model, model.revision());
    if (!commit.success) throw std::runtime_error(commit.error);
    require(!model.bodyShape().IsNull() && BRepCheck_Analyzer(model.bodyShape()).IsValid(),
        "Production candidate handoff produced no valid CAD");
    GProp_GProps properties;
    BRepGProp::VolumeProperties(model.bodyShape(), properties);
    metrics["volume_mm3"] = properties.Mass();
    metrics["feature_count"] = qint64(model.features().size());
    metrics["committed_revision"] = qint64(model.revision());
    input["features"] = FeatureCodec::encode(model.features());
    return input;
}
} // namespace contract
