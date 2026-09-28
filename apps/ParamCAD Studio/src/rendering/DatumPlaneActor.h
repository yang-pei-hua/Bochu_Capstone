#pragma once

#include "modeling/FaceReference.h"

#include <vtkSmartPointer.h>

#include <array>
#include <optional>

class vtkActor;
class vtkBillboardTextActor3D;
class vtkPolyDataMapper;
class vtkRenderer;

// Draws the three datum planes (XY, YZ, XZ) as translucent squares with an
// outline and a billboarded name, so a sketch plane is picked straight in the
// 3D scene instead of from a side panel.
//
// It is a display cache only, like BodyActor: which plane is selected lives in
// the window, which owns the pending sketch plane.
class DatumPlaneActor
{
public:
    DatumPlaneActor();

    // Adds the plane surfaces, their outlines and their labels to the renderer.
    void attach(vtkRenderer* renderer) const;

    void setVisible(bool visible);
    bool visible() const noexcept;

    // The highlighted plane is drawn more opaque and with a brighter outline.
    void setHighlightedPlane(std::optional<modeling::DatumPlane> plane);
    std::optional<modeling::DatumPlane> highlightedPlane() const noexcept;

    // Surface actor of one plane. It is the only prop the picker is given, so
    // the outlines and the labels can never be hit.
    vtkActor* surfaceActor(modeling::DatumPlane plane) const noexcept;

    // Datum plane the given actor draws, if it draws one.
    std::optional<modeling::DatumPlane> planeOfActor(const vtkActor* actor) const noexcept;

private:
    static constexpr int kPlaneCount = 3;

    struct PlaneVisual {
        modeling::DatumPlane plane = modeling::DatumPlane::XY;
        vtkSmartPointer<vtkPolyDataMapper> mapper;
        vtkSmartPointer<vtkActor> surface;
        vtkSmartPointer<vtkActor> outline;
        vtkSmartPointer<vtkBillboardTextActor3D> label;
    };

    const PlaneVisual* visualOf(modeling::DatumPlane plane) const noexcept;

    std::array<PlaneVisual, kPlaneCount> m_planes;
    bool m_visible = true;
    std::optional<modeling::DatumPlane> m_highlighted;
};