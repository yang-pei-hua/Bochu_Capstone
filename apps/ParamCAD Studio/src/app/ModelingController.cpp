#include "app/ModelingController.h"
#include <modeling/PrimitiveEditing.h>

#include <modeling/FeatureParams.h>
#include <modeling/Sketch.h>

#include <reconstruction/modeling/ModelingAdapter.h>

namespace {

const TopoDS_Shape& nullShape()
{
    static const TopoDS_Shape shape;
    return shape;
}

}  // namespace

ModelingController::ModelingController(QObject* parent)
    : QObject(parent)
    , m_core(std::make_unique<modeling::ModelingCore>())
{
}

bool ModelingController::addSketch(const modeling::SketchFeatureParams& params,
                                   modeling::FeatureId& createdId)
{
    if (!addFeatureCommand(params, createdId)) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::editSketch(modeling::FeatureId sketchId,
                                    const modeling::SketchFeatureParams& params)
{
    if (!execute(modeling::EditFeatureCommand{sketchId, params})) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::addSketchEntity(modeling::FeatureId sketchId,
                                        const modeling::SketchGeometry& geometry,
                                        modeling::SketchEntityId& createdEntityId)
{
    modeling::SketchEntity entity;
    entity.geometry = geometry;
    if (!execute(modeling::AddSketchEntityCommand{sketchId, entity}, nullptr,
                 &createdEntityId)) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::editSketchEntity(modeling::FeatureId sketchId,
                                          modeling::SketchEntityId entityId,
                                          const modeling::SketchGeometry& geometry)
{
    if (!execute(modeling::EditSketchEntityCommand{sketchId, entityId, geometry})) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::removeSketchEntity(modeling::FeatureId sketchId,
                                            modeling::SketchEntityId entityId)
{
    if (!execute(modeling::RemoveSketchEntityCommand{sketchId, entityId})) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::extrude(modeling::FeatureId sketchId, double depth, bool reverse,
                                 modeling::ExtrudeOperation operation,
                                 modeling::FeatureId& createdId)
{
    modeling::ExtrudeFeatureParams params;
    params.sketchId = sketchId;
    params.depth = depth;
    params.reverse = reverse;
    params.operation = operation;
    if (!addFeatureCommand(params, createdId)) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::cut(modeling::FeatureId sketchId, double depth, bool throughAll,
                             bool reverse, modeling::FeatureId& createdId)
{
    modeling::CutFeatureParams params;
    params.sketchId = sketchId;
    params.depth = depth;
    params.throughAll = throughAll;
    params.reverse = reverse;
    if (!addFeatureCommand(params, createdId)) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::commitReconstructedBox(const reconstruction::BoxCandidate& candidate)
{
    if (!m_core) {
        return false;
    }

    // The patch carries rebuild = true, so the document is up to date as soon
    // as it reports success; the UI follows from that alone.
    const auto before = *m_core;
    const modeling::ModelPatchResult result =
        reconstruction::commitBoxCandidate(candidate, *m_core);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        emit modelError(m_lastError);
        return false;
    }

    m_lastError.clear();
    recordHistory(before);
    emit modelRebuilt();
    return true;
}

bool ModelingController::commitReconstructedCompound(const reconstruction::CompoundCandidate& candidate,
                                                     bool overwriteManual)
{
    if (!m_core) return false;
    const auto before = *m_core;
    const auto result = reconstruction::commitCompoundCandidate(candidate, *m_core,
        m_core->revision(), true, overwriteManual);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        emit modelError(m_lastError);
        return false;
    }
    m_lastError.clear();
    recordHistory(before);
    emit modelRebuilt();
    return true;
}

bool ModelingController::commitReconstructedBoxWithThroughHoles(
    const reconstruction::BoxWithThroughHolesCandidate& candidate)
{
    if (!m_core) {
        return false;
    }

    const auto before = *m_core;
    const modeling::ModelPatchResult result =
        reconstruction::commitBoxWithThroughHolesCandidate(candidate, *m_core);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        emit modelError(m_lastError);
        return false;
    }

    m_lastError.clear();
    recordHistory(before);
    emit modelRebuilt();
    return true;
}

bool ModelingController::removeFeature(modeling::FeatureId id, bool cascade)
{
    if (!execute(modeling::RemoveFeatureCommand{id, cascade})) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

std::vector<modeling::FeatureId> ModelingController::dependentsOf(
    modeling::FeatureId id) const
{
    return m_core ? m_core->document().dependentsOf(id, true)
                  : std::vector<modeling::FeatureId>{};
}

const modeling::SketchFeatureParams* ModelingController::sketchParams(
    modeling::FeatureId sketchId) const
{
    if (!m_core) {
        return nullptr;
    }
    const modeling::Feature* feature = m_core->document().findFeature(sketchId);
    if (feature == nullptr) {
        return nullptr;
    }
    return std::get_if<modeling::SketchFeatureParams>(&feature->params);
}

modeling::FeatureId ModelingController::latestExtrudeId() const noexcept
{
    modeling::FeatureId latest = modeling::kInvalidFeatureId;
    for (const modeling::Feature& feature : features()) {
        if (feature.type == modeling::FeatureType::Extrude && !feature.suppressed) {
            latest = feature.id;
        }
    }
    return latest;
}

const TopoDS_Shape& ModelingController::bodyShape() const
{
    return m_core ? m_core->document().bodyShape() : nullShape();
}

const std::vector<modeling::Feature>& ModelingController::features() const
{
    static const std::vector<modeling::Feature> emptyHistory;
    return m_core ? m_core->document().features() : emptyHistory;
}

const QString& ModelingController::lastError() const noexcept
{
    return m_lastError;
}

void ModelingController::adoptProject(modeling::ModelingCore&& prepared)
{
    m_core->adoptPrepared(std::move(prepared));
    clearHistory();
    m_lastError.clear();
    emit modelRebuilt();
}

modeling::ModelRevision ModelingController::revision() const noexcept
{
    return m_core->revision();
}

bool ModelingController::addFeatureCommand(const modeling::FeatureParams& params,
                                           modeling::FeatureId& createdId)
{
    return execute(modeling::AddFeatureCommand{params}, &createdId);
}

bool ModelingController::execute(const modeling::ModelCommand& command,
                                 modeling::FeatureId* createdId,
                                 modeling::SketchEntityId* createdEntityId)
{
    // Stage ID allocation; actual mutation and rebuild commit atomically.
    auto staging = *m_core;
    const modeling::ModelResult result = staging.execute(command);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        return false;
    }
    modeling::ModelPatch patch;
    patch.expectedRevision = m_core->revision();
    patch.commands.push_back(command);
    const auto* changed = staging.document().findFeature(result.featureId);
    if (changed && changed->reconstructed
        && !std::holds_alternative<modeling::AddFeatureCommand>(command)
        && !std::holds_alternative<modeling::SetFeatureSuppressedCommand>(command)) {
        patch.commands.push_back(modeling::EditFeatureCommand{changed->id, changed->params, true, changed->resizeAnchor});
    }
    const auto before = *m_core;
    const auto applied = m_core->apply(patch);
    if (!applied.success) {
        m_lastError = QString::fromStdString(applied.error);
        return false;
    }
    recordHistory(before);
    if (createdId != nullptr) {
        *createdId = result.featureId;
    }
    if (createdEntityId != nullptr) {
        *createdEntityId = result.sketchEntityId;
    }
    return true;
}

bool ModelingController::rebuildAndPublish()
{
    m_lastError.clear();
    emit modelRebuilt();
    return true;
}

void ModelingController::recordHistory(modeling::ModelingCore before)
{
    if (m_undo.size() >= 50) m_undo.erase(m_undo.begin());
    m_undo.push_back(std::move(before));
    m_redo.clear();
    emit historyChanged();
}

void ModelingController::clearHistory()
{
    m_undo.clear(); m_redo.clear();
    emit historyChanged();
}

bool ModelingController::undo()
{
    if (!canUndo()) return false;
    m_redo.push_back(*m_core);
    m_core->adoptPrepared(std::move(m_undo.back()));
    m_undo.pop_back(); m_lastError.clear();
    emit modelRebuilt(); emit historyChanged();
    return true;
}

bool ModelingController::redo()
{
    if (!canRedo()) return false;
    m_undo.push_back(*m_core);
    m_core->adoptPrepared(std::move(m_redo.back()));
    m_redo.pop_back(); m_lastError.clear();
    emit modelRebuilt(); emit historyChanged();
    return true;
}

bool ModelingController::hasProtectedReconstruction() const
{
    for (const auto& f : features()) if (f.reconstructed && f.manuallyEdited) return true;
    return false;
}

bool ModelingController::editPrimitive(modeling::FeatureId id, const std::array<double,3>& values,
                                      modeling::ResizeAnchor anchor,
                                      modeling::ModelRevision expectedRevision)
{
    const auto* feature = m_core->document().findFeature(id);
    modeling::FeatureParams params;
    std::string error;
    if (expectedRevision != revision()) m_lastError = tr("模型已变化，请重新选择特征后编辑。");
    else if (!feature || feature->suppressed) m_lastError = tr("特征不存在或已抑制。");
    else if (!modeling::adjustedPrimitive(feature->params, values, anchor, params, error))
        m_lastError = QString::fromStdString(error);
    else {
        std::array<double,3> current{1,1,1};
        size_t count=0;
        if (const auto* box=std::get_if<modeling::BoxPrimitiveParams>(&feature->params)) { current={box->sizeX,box->sizeY,box->sizeZ}; count=3; }
        else if (const auto* hole=std::get_if<modeling::ThroughHolePrimitiveParams>(&feature->params)) { current[0]=hole->radius*2; count=1; }
        else if (const auto* cylinder=std::get_if<modeling::CylinderPrimitiveParams>(&feature->params)) { current={cylinder->radius,cylinder->height,1}; count=2; }
        bool unchanged=feature->resizeAnchor==anchor;
        for (size_t i=0;i<count;++i) unchanged=unchanged && current[i]==values[i];
        if (unchanged) { m_lastError.clear(); return true; }
        if (execute(modeling::EditFeatureCommand{id,params,true,anchor})) return rebuildAndPublish();
    }
    emit modelError(m_lastError);
    return false;
}

bool ModelingController::validatePrimitiveEdit(modeling::FeatureId id, const std::array<double,3>& values,
    modeling::ResizeAnchor anchor, modeling::ModelRevision expectedRevision, QString& error) const
{
    error.clear();
    const auto* feature = m_core->document().findFeature(id);
    modeling::FeatureParams params;
    std::string failure;
    if (expectedRevision != revision()) error = tr("模型已变化，请重新预览文字修改。");
    else if (!feature || feature->suppressed || !feature->valid) error = tr("特征不存在、已抑制或无效。");
    else if (!modeling::adjustedPrimitive(feature->params, values, anchor, params, failure))
        error = QString::fromStdString(failure);
    else {
        auto trial = *m_core;
        modeling::ModelPatch patch;
        patch.expectedRevision = expectedRevision;
        patch.commands.push_back(modeling::EditFeatureCommand{id, params, true, anchor});
        const auto result = trial.apply(patch);
        if (result.success) return true;
        error = QString::fromStdString(result.error);
    }
    return false;
}
