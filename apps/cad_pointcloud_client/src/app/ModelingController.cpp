#include "app/ModelingController.h"

#include <modeling/FeatureParams.h>
#include <modeling/Sketch.h>

#include <reconstruction/ModelingAdapter.h>

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
    const modeling::ModelPatchResult result =
        reconstruction::commitBoxCandidate(candidate, *m_core);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        emit modelError(m_lastError);
        return false;
    }

    m_lastError.clear();
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
    return m_core ? m_core->document().dependentsOf(id)
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

bool ModelingController::addFeatureCommand(const modeling::FeatureParams& params,
                                           modeling::FeatureId& createdId)
{
    return execute(modeling::AddFeatureCommand{params}, &createdId);
}

bool ModelingController::execute(const modeling::ModelCommand& command,
                                 modeling::FeatureId* createdId,
                                 modeling::SketchEntityId* createdEntityId)
{
    const modeling::ModelResult result = m_core->execute(command);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        return false;
    }
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
    const bool rebuilt = m_core->document().rebuild();
    if (rebuilt) {
        m_lastError.clear();
    } else {
        m_lastError = QString::fromStdString(m_core->document().lastError());
    }

    // Publish even after a failed rebuild: the core keeps the last body that
    // rebuilt successfully, and the viewer shows exactly that shape.
    emit modelRebuilt();
    if (!rebuilt) {
        emit modelError(m_lastError);
    }
    return rebuilt;
}
