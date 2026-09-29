#pragma once

#include "core/SketchFrame.h"
#include "modeling/FeatureParams.h"
#include "modeling/Id.h"
#include "modeling/Sketch.h"

#include <vtkSmartPointer.h>

#include <vector>

class vtkActor;
class vtkCellArray;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;

// Draws the active sketch in the 3D viewport, on the plane the core will build
// it on. It is a display cache like BodyActor: the sketch itself lives in the
// feature graph and the frame is derived from that history.
//
// A second actor draws the transient preview of the geometry the active tool
// would create, and picking is resolved from the entities themselves in the
// sketch's own coordinates rather than from the drawn cells.
class SketchActor
{
public:
    SketchActor();

    void setSketch(const modeling::SketchFeatureParams& params,
                   const sketchapp::PlaneFrame& frame);
    void clear();
    bool hasSketch() const noexcept;
    void setVisible(bool visible);

    // Transient preview of a half-finished tool operation, drawn in the frame of
    // the sketch currently adopted through setSketch().
    void setDraft(const modeling::SketchGeometry& geometry);
    void clearDraft();

    // Entity whose geometry passes within tolerance of the given sketch point, or
    // kInvalidSketchEntityId when nothing is close enough. Sketch lines are
    // zero-width on screen, so a click is matched against the geometry in the
    // sketch's own coordinates instead of through a geometric picker.
    modeling::SketchEntityId entityAt(const modeling::Point2D& point,
                                      double tolerance) const noexcept;

    // Vertex of the sketch nearest to the given point within tolerance - a point
    // entity, a line endpoint, a rectangle corner or a circle centre - so a
    // drawing tool can anchor exactly on existing geometry. False when nothing is
    // close enough, which is what lets the caller fall back to a free point.
    bool snapPointAt(const modeling::Point2D& point, double tolerance,
                     modeling::Point2D& snapped) const noexcept;

    vtkActor* actor() const noexcept;
    vtkActor* draftActor() const noexcept;

private:
    void resetDraftArrays();

    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkCellArray> m_lines;
    vtkSmartPointer<vtkCellArray> m_vertices;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    std::vector<modeling::SketchEntity> m_entities;
    sketchapp::PlaneFrame m_frame;
    bool m_hasFrame = false;
    bool m_hasSketch = false;
    bool m_visible = true;

    vtkSmartPointer<vtkPoints> m_draftPoints;
    vtkSmartPointer<vtkCellArray> m_draftLines;
    vtkSmartPointer<vtkCellArray> m_draftVertices;
    vtkSmartPointer<vtkPolyData> m_draftPolyData;
    vtkSmartPointer<vtkPolyDataMapper> m_draftMapper;
    vtkSmartPointer<vtkActor> m_draftActor;
    bool m_hasDraft = false;
};
