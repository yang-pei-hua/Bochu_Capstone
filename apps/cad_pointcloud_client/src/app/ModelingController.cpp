#include "app/ModelingController.h"

#include <modeling/FeatureParams.h>
#include <modeling/Sketch.h>

namespace {

const TopoDS_Shape& nullShape()
{
    static const TopoDS_Shape shape;
    return shape;
}

modeling::SketchFeatureParams baseSketchParams(const DemoModelParameters& parameters)
{
    modeling::SketchFeatureParams params;
    params.plane = modeling::datumPlane(modeling::DatumPlane::XY);
    params.entities = {
        {demo::kBaseRectangleId,
         modeling::Rectangle2D{0.0, 0.0, parameters.width, parameters.height}},
    };
    return params;
}

// The hole sketch is attached to the semantic EndFace of the extrusion, so the
// core re-resolves it whenever the extrusion height changes.
modeling::SketchFeatureParams holeSketchParams(const DemoModelParameters& parameters,
                                               modeling::FeatureId extrudeId)
{
    modeling::SketchFeatureParams params;
    params.plane = modeling::featureFace(extrudeId, modeling::FaceRole::EndFace);
    params.entities = {
        {demo::kHoleCircleId,
         modeling::Circle2D{parameters.holeCenterX, parameters.holeCenterY,
                            parameters.holeRadius}},
    };
    return params;
}

}  // namespace

ModelingController::ModelingController(QObject* parent)
    : QObject(parent)
{
}

bool ModelingController::createDemoModel(const DemoModelParameters& parameters)
{
    // A new document replaces the previous one entirely, so the whole feature
    // history is rebuilt from scratch by the core.
    m_core = std::make_unique<modeling::ModelingCore>();
    m_ids = demo::DemoModelIds{};
    m_lastError.clear();

    const bool created =
        addFeatureCommand(baseSketchParams(parameters), m_ids.baseSketch)
        && addFeatureCommand(modeling::ExtrudeFeatureParams{m_ids.baseSketch,
                                                            parameters.extrusionDepth,
                                                            false},
                             m_ids.extrude)
        && addFeatureCommand(holeSketchParams(parameters, m_ids.extrude), m_ids.holeSketch)
        && addFeatureCommand(modeling::CutFeatureParams{m_ids.holeSketch,
                                                        parameters.cutDepth,
                                                        false},
                             m_ids.cut);

    if (!created) {
        m_ids = demo::DemoModelIds{};
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
}

bool ModelingController::updateDemoModel(const DemoModelParameters& parameters)
{
    if (!m_core || !m_ids.isComplete()) {
        return createDemoModel(parameters);
    }

    const bool updated =
        execute(modeling::EditFeatureCommand{m_ids.baseSketch, baseSketchParams(parameters)})
        && execute(modeling::EditFeatureCommand{
               m_ids.extrude,
               modeling::ExtrudeFeatureParams{m_ids.baseSketch,
                                              parameters.extrusionDepth,
                                              false}})
        && execute(modeling::EditFeatureCommand{
               m_ids.holeSketch, holeSketchParams(parameters, m_ids.extrude)})
        && execute(modeling::EditFeatureCommand{
               m_ids.cut,
               modeling::CutFeatureParams{m_ids.holeSketch, parameters.cutDepth, false}});

    if (!updated) {
        emit modelError(m_lastError);
        return false;
    }
    return rebuildAndPublish();
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
                                 modeling::FeatureId* createdId)
{
    const modeling::ModelResult result = m_core->execute(command);
    if (!result.success) {
        m_lastError = QString::fromStdString(result.error);
        return false;
    }
    if (createdId != nullptr) {
        *createdId = result.featureId;
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
