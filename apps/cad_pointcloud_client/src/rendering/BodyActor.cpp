#include "rendering/BodyActor.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkFeatureEdges.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <algorithm>
#include <cstddef>

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

    m_highlightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_highlightMapper->ScalarVisibilityOff();
    m_highlightMapper->SetResolveCoincidentTopologyToPolygonOffset();
    m_highlightMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-2.0, -2.0);

    m_highlightActor = vtkSmartPointer<vtkActor>::New();
    m_highlightActor->SetMapper(m_highlightMapper);
    m_highlightActor->GetProperty()->SetColor(1.0, 0.65, 0.10);
    m_highlightActor->GetProperty()->SetOpacity(0.55);
    m_highlightActor->GetProperty()->SetInterpolationToFlat();
    m_highlightActor->SetVisibility(0);
}

void BodyActor::setShape(const TopoDS_Shape& shape)
{
    m_tessellation = OcctShapeTessellator::tessellate(shape);
    m_hasShape = !m_tessellation.empty();

    // Only the mapper input is refreshed so the camera stays where the user
    // left it; the tessellator returns a brand new data object every time.
    m_surfaceMapper->SetInputData(m_tessellation.mesh);
    m_featureEdges->SetInputData(m_tessellation.mesh);
    m_featureEdges->Update();
    m_surfaceMapper->Update();

    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    clearHighlight();
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
    m_highlightActor->SetVisibility(
        m_visible && m_hasShape && m_highlightedFace >= 0 ? 1 : 0);
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
    vtkActor* actors[] = {m_surfaceActor, m_edgeActor, m_highlightActor};
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

int BodyActor::faceCount() const noexcept
{
    return static_cast<int>(m_tessellation.faces.size());
}

const BodyFaceInfo* BodyActor::faceInfo(int faceIndex) const noexcept
{
    if (faceIndex < 0 || faceIndex >= faceCount()) {
        return nullptr;
    }
    return &m_tessellation.faces[static_cast<std::size_t>(faceIndex)];
}

int BodyActor::faceIdOfCell(vtkIdType cellId) const noexcept
{
    if (m_tessellation.mesh == nullptr || cellId < 0) {
        return -1;
    }
    vtkDataArray* ids = m_tessellation.mesh->GetCellData()->GetArray("FaceIds");
    if (ids == nullptr || cellId >= ids->GetNumberOfTuples()) {
        return -1;
    }
    return static_cast<int>(ids->GetComponent(cellId, 0));
}

void BodyActor::highlightFace(int faceIndex)
{
    const BodyFaceInfo* info = faceInfo(faceIndex);
    if (info == nullptr || info->cellCount <= 0) {
        clearHighlight();
        return;
    }

    vtkPolyData* source = m_tessellation.mesh;
    vtkNew<vtkCellArray> selection;
    const vtkIdType lastCell = info->firstCell + info->cellCount;
    for (vtkIdType cell = info->firstCell; cell < lastCell; ++cell) {
        vtkIdType pointCount = 0;
        const vtkIdType* pointIds = nullptr;
        source->GetCellPoints(cell, pointCount, pointIds);
        selection->InsertNextCell(pointCount, pointIds);
    }

    vtkNew<vtkPolyData> overlay;
    overlay->SetPoints(source->GetPoints());
    overlay->SetPolys(selection);
    m_highlightMapper->SetInputData(overlay);

    m_highlightedFace = faceIndex;
    m_highlightActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
}

void BodyActor::clearHighlight()
{
    m_highlightedFace = -1;
    m_highlightMapper->SetInputData(vtkSmartPointer<vtkPolyData>::New());
    m_highlightActor->SetVisibility(0);
}

vtkActor* BodyActor::highlightActor() const noexcept
{
    return m_highlightActor;
}

void BodyActor::updateEdgeVisibility()
{
    const bool showEdges = m_visible && m_hasShape && m_representation == 0;
    m_edgeActor->SetVisibility(showEdges ? 1 : 0);
}
