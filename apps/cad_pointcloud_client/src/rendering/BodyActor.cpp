#include "rendering/BodyActor.h"

#include "rendering/OcctShapeTessellator.h"

#include <vtkActor.h>
#include <vtkFeatureEdges.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <algorithm>

BodyActor::BodyActor()
{
    m_surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_surfaceMapper->ScalarVisibilityOff();

    m_surfaceActor = vtkSmartPointer<vtkActor>::New();
    m_surfaceActor->SetMapper(m_surfaceMapper);
    m_surfaceActor->GetProperty()->SetColor(0.28, 0.57, 0.84);
    m_surfaceActor->GetProperty()->SetInterpolationToPhong();
    m_surfaceActor->SetVisibility(0);

    // Every modeled face gets its own set of points during tessellation, so the
    // borders between faces are boundary edges rather than feature edges.
    m_featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
    m_featureEdges->FeatureEdgesOff();
    m_featureEdges->BoundaryEdgesOn();
    m_featureEdges->NonManifoldEdgesOff();
    m_featureEdges->ManifoldEdgesOff();
    m_featureEdges->ColoringOff();

    m_edgeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_edgeMapper->SetInputConnection(m_featureEdges->GetOutputPort());
    m_edgeMapper->ScalarVisibilityOff();

    m_edgeActor = vtkSmartPointer<vtkActor>::New();
    m_edgeActor->SetMapper(m_edgeMapper);
    m_edgeActor->GetProperty()->SetColor(0.06, 0.07, 0.09);
    m_edgeActor->GetProperty()->SetLineWidth(1.5);
    m_edgeActor->SetVisibility(0);
}

void BodyActor::setShape(const TopoDS_Shape& shape)
{
    vtkSmartPointer<vtkPolyData> mesh = OcctShapeTessellator::tessellate(shape);
    m_hasShape = mesh->GetNumberOfPoints() > 0 && mesh->GetNumberOfPolys() > 0;

    // Only the mapper input is refreshed so the camera stays where the user
    // left it; the tessellator returns a brand new data object every time.
    m_surfaceMapper->SetInputData(mesh);
    m_featureEdges->SetInputData(mesh);
    m_featureEdges->Update();
    m_surfaceMapper->Update();

    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    updateEdgeVisibility();
}

void BodyActor::clear()
{
    setShape(TopoDS_Shape());
}

bool BodyActor::hasShape() const noexcept
{
    return m_hasShape;
}

void BodyActor::setVisible(bool visible)
{
    m_visible = visible;
    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    updateEdgeVisibility();
}

void BodyActor::setRepresentation(int representation)
{
    m_representation = representation;
    switch (representation) {
    case 1:
        m_surfaceActor->GetProperty()->SetRepresentationToWireframe();
        break;
    case 2:
        m_surfaceActor->GetProperty()->SetRepresentationToPoints();
        m_surfaceActor->GetProperty()->SetPointSize(4.0F);
        break;
    default:
        m_surfaceActor->GetProperty()->SetRepresentationToSurface();
        break;
    }
    updateEdgeVisibility();
}

void BodyActor::setOpacity(double opacity)
{
    m_surfaceActor->GetProperty()->SetOpacity(std::clamp(opacity, 0.0, 1.0));
}

void BodyActor::setColor(const QColor& color)
{
    m_surfaceActor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
}

void BodyActor::setLightingEnabled(bool enabled)
{
    m_surfaceActor->GetProperty()->SetLighting(enabled ? 1 : 0);
}

void BodyActor::setTransform(double px, double py, double pz,
                             double rx, double ry, double rz,
                             double sx, double sy, double sz)
{
    vtkActor* actors[] = {m_surfaceActor, m_edgeActor};
    for (vtkActor* actor : actors) {
        actor->SetPosition(px, py, pz);
        actor->SetOrientation(rx, ry, rz);
        actor->SetScale(sx, sy, sz);
    }
}

vtkActor* BodyActor::surfaceActor() const noexcept
{
    return m_surfaceActor;
}

vtkActor* BodyActor::edgeActor() const noexcept
{
    return m_edgeActor;
}

void BodyActor::updateEdgeVisibility()
{
    const bool showEdges = m_visible && m_hasShape && m_representation == 0;
    m_edgeActor->SetVisibility(showEdges ? 1 : 0);
}
