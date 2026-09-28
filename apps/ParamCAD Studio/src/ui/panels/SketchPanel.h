#pragma once

#include "core/SketchFrame.h"
#include "modeling/FeatureParams.h"
#include "modeling/Id.h"
#include "modeling/Sketch.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QTreeWidget;

// Sketch dock.
//
// The sketch is drawn in the 3D viewport now, so this panel is the feature
// side of the workspace: it lists the entities the core holds, reports the
// core's verdict on the profile, and exposes Extrude and Cut.
//
// The panel is a view: the open sketch lives in the core's feature graph and is
// pushed in through setSketch(). Every user action is reported as an intent
// signal and the window turns it into a core command, so there is exactly one
// sketch source of truth.
class SketchPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit SketchPanel(QWidget* parent = nullptr);

    // Adopts the sketch the core currently holds and re-renders the entity list
    // from it.
    void setSketch(const modeling::SketchFeatureParams& params,
                   const sketchapp::PlaneFrame& frame);
    void clearSketch();
    bool hasSketch() const noexcept;
    void setPlaneText(const QString& text);

    const modeling::SketchEntity* findEntity(modeling::SketchEntityId entityId) const noexcept;
    modeling::SketchEntityId selectedEntityId() const noexcept;
    // Selects an entity row on behalf of the viewport; the invalid id clears it.
    void selectEntity(modeling::SketchEntityId entityId);
    void setStatusText(const QString& text);

signals:
    void entityRemoveRequested(modeling::SketchEntityId entityId);
    void entitySelected(modeling::SketchEntityId entityId);
    void entitySelectionCleared();
    void extrudeRequested(double depth, bool reverse, modeling::ExtrudeOperation operation);
    void cutRequested(double depth, bool throughAll, bool reverse);

private:
    void rebuildList();
    void updateStatus();
    void deleteSelected();
    double extrudeDepthValue() const;
    double cutDepthValue() const;

    modeling::SketchFeatureParams m_params;
    sketchapp::PlaneFrame m_frame;
    bool m_hasSketch = false;
    QLabel* m_planeLabel = nullptr;
    QLabel* m_status = nullptr;
    QTreeWidget* m_entities = nullptr;
    QComboBox* m_extrudeOperation = nullptr;
    QLineEdit* m_extrudeDepth = nullptr;
    QLineEdit* m_cutDepth = nullptr;
    QCheckBox* m_extrudeReverse = nullptr;
    QCheckBox* m_cutThroughAll = nullptr;
    QCheckBox* m_cutReverse = nullptr;
    bool m_refreshing = false;
};
