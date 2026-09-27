#pragma once

#include "rendering/OcctShapeTessellator.h"

#include <vtkSmartPointer.h>

#include <QColor>

class TopoDS_Shape;
class vtkActor;
class vtkFeatureEdges;
class vtkPolyData;
class vtkPolyDataMapper;

// Owns the VTK pipeline that draws one modeled body: a shaded surface plus a
// black outline of the sharp edges. It is a display cache only - the real
// model state lives in modeling::ModelingCore.
//
// Besides the display mesh it keeps the per-face index produced by the
// tessellator so the viewport can map a picked display cell back to an
// OpenCASCADE face and highlight it.
class BodyActor
{
public:
    BodyActor();

    void setShape(const TopoDS_Shape& shape);
    void clear();

    bool hasShape() const noexcept;

    void setVisible(bool visible);
    void setRepresentation(int representation);
    void setOpacity(double opacity);
    void setColor(const QColor& color);
    void setLightingEnabled(bool enabled);
    void setTransform(double px, double py, double pz,
                      double rx, double ry, double rz,
                      double sx, double sy, double sz);

    vtkActor* surfaceActor() const noexcept;
    vtkActor* edgeActor() const noexcept;

    // Face index lookups. A cell that carries no face index returns -1.
    int faceCount() const noexcept;
    const BodyFaceInfo* faceInfo(int faceIndex) const noexcept;
    int faceIdOfCell(vtkIdType cellId) const noexcept;

    // Draws a translucent overlay on the given face; pass a negative index to
    // clear it. The overlay follows the body transform and is polygon-offset so
    // it never z-fights with the shaded surface.
    void highlightFace(int faceIndex);
    void clearHighlight();
    vtkActor* highlightActor() const noexcept;

private:
    void updateEdgeVisibility();

    BodyTessellation m_tessellation;
    vtkSmartPointer<vtkPolyDataMapper> m_surfaceMapper;
    vtkSmartPointer<vtkActor> m_surfaceActor;
    vtkSmartPointer<vtkFeatureEdges> m_featureEdges;
    vtkSmartPointer<vtkPolyDataMapper> m_edgeMapper;
    vtkSmartPointer<vtkActor> m_edgeActor;
    vtkSmartPointer<vtkPolyDataMapper> m_highlightMapper;
    vtkSmartPointer<vtkActor> m_highlightActor;
    bool m_hasShape = false;
    bool m_visible = true;
    int m_representation = 0;
    int m_highlightedFace = -1;
};
