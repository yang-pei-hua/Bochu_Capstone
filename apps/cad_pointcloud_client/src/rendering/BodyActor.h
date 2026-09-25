#pragma once

#include <vtkSmartPointer.h>

#include <QColor>

class TopoDS_Shape;
class vtkActor;
class vtkFeatureEdges;
class vtkPolyDataMapper;

// Owns the VTK pipeline that draws one modeled body: a shaded surface plus a
// black outline of the sharp edges. It is a display cache only - the real
// model state lives in modeling::ModelingCore.
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

private:
    void updateEdgeVisibility();

    vtkSmartPointer<vtkPolyDataMapper> m_surfaceMapper;
    vtkSmartPointer<vtkActor> m_surfaceActor;
    vtkSmartPointer<vtkFeatureEdges> m_featureEdges;
    vtkSmartPointer<vtkPolyDataMapper> m_edgeMapper;
    vtkSmartPointer<vtkActor> m_edgeActor;
    bool m_hasShape = false;
    bool m_visible = true;
    int m_representation = 0;
};
