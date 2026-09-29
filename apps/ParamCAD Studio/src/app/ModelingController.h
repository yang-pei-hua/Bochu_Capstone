#pragma once

#include <modeling/Feature.h>
#include <modeling/ModelCommand.h>
#include <modeling/ModelingCore.h>
#include <reconstruction/ReconstructionCandidate.h>

#include <TopoDS_Shape.hxx>

#include <QObject>
#include <QString>

#include <memory>
#include <vector>

// Front-end orchestration layer. It turns UI intents into modeling commands and
// publishes the resulting body; it contains no geometry code of its own.
//
// The document it owns starts empty: a feature either comes from the sketch
// workflow or from a recognized primitive, so the client never seeds geometry
// of its own.
class ModelingController final : public QObject
{
    Q_OBJECT

public:
    explicit ModelingController(QObject* parent = nullptr);

    bool addSketch(const modeling::SketchFeatureParams& params,
                   modeling::FeatureId& createdId);
    bool editSketch(modeling::FeatureId sketchId,
                    const modeling::SketchFeatureParams& params);

    // Single-entity authoring. The core allocates and validates the entity ids,
    // so the client never invents one.
    bool addSketchEntity(modeling::FeatureId sketchId,
                         const modeling::SketchGeometry& geometry,
                         modeling::SketchEntityId& createdEntityId);
    bool editSketchEntity(modeling::FeatureId sketchId,
                          modeling::SketchEntityId entityId,
                          const modeling::SketchGeometry& geometry);
    bool removeSketchEntity(modeling::FeatureId sketchId,
                            modeling::SketchEntityId entityId);

    bool extrude(modeling::FeatureId sketchId, double depth, bool reverse,
                 modeling::ExtrudeOperation operation,
                 modeling::FeatureId& createdId);
    bool cut(modeling::FeatureId sketchId, double depth, bool throughAll, bool reverse,
             modeling::FeatureId& createdId);

    // Commits a box recognized from observed points. Reconstruction is another
    // author of the same feature graph, so it enters through the core's atomic
    // patch boundary rather than through the sketch commands.
    bool commitReconstructedBox(const reconstruction::BoxCandidate& candidate);
    bool commitReconstructedBoxWithThroughHoles(
        const reconstruction::BoxWithThroughHolesCandidate& candidate);

    // Deleting a feature that other features consume has to cascade, otherwise
    // the document can no longer rebuild. dependentsOf() is what the UI shows
    // to the user before confirming.
    bool removeFeature(modeling::FeatureId id, bool cascade);
    std::vector<modeling::FeatureId> dependentsOf(modeling::FeatureId id) const;

    // Read-only lookups into the feature graph, so callers never keep a second
    // copy of the model state.
    const modeling::SketchFeatureParams* sketchParams(modeling::FeatureId sketchId) const;
    modeling::FeatureId latestExtrudeId() const noexcept;

    const TopoDS_Shape& bodyShape() const;
    const std::vector<modeling::Feature>& features() const;
    const QString& lastError() const noexcept;

signals:
    void modelRebuilt();
    void modelError(const QString& message);

private:
    bool addFeatureCommand(const modeling::FeatureParams& params,
                           modeling::FeatureId& createdId);
    bool execute(const modeling::ModelCommand& command,
                 modeling::FeatureId* createdId = nullptr,
                 modeling::SketchEntityId* createdEntityId = nullptr);
    bool rebuildAndPublish();

    std::unique_ptr<modeling::ModelingCore> m_core;
    QString m_lastError;
};
