#include "rendering/PointCloudActor.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <array>

PointCloudActor::PointCloudActor()
{
    m_points = vtkSmartPointer<vtkPoints>::New();
    m_vertices = vtkSmartPointer<vtkCellArray>::New();

    m_polyData = vtkSmartPointer<vtkPolyData>::New();
    m_polyData->SetPoints(m_points);
    m_polyData->SetVerts(m_vertices);

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputData(m_polyData);
    m_mapper->ScalarVisibilityOff();

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetPointSize(2.0);
    m_actor->GetProperty()->SetColor(0.45, 0.72, 0.95);
    // An unoriented cloud has no normals, so lighting would only darken it.
    m_actor->GetProperty()->LightingOff();
    m_actor->SetVisibility(0);
}

bool PointCloudActor::setCloud(const PlyCloud& cloud)
{
    if (cloud.positions.empty()) {
        clear();
        return false;
    }

    const auto count = static_cast<vtkIdType>(cloud.positions.size());

    m_points->Initialize();
    m_points->SetNumberOfPoints(count);
    for (vtkIdType index = 0; index < count; ++index) {
        const std::array<float, 3>& position = cloud.positions[static_cast<std::size_t>(index)];
        m_points->SetPoint(index, position[0], position[1], position[2]);
    }
    m_points->Modified();

    // The vendored VTK ships without FiltersGeneral, so vtkVertexGlyphFilter is
    // unavailable and the one-point cells are built by hand.
    m_vertices->Initialize();
    m_vertices->AllocateEstimate(count, 1);
    for (vtkIdType index = 0; index < count; ++index) {
        m_vertices->InsertNextCell(1);
        m_vertices->InsertCellPoint(index);
    }
    m_vertices->Modified();

    if (cloud.colors.size() == cloud.positions.size()) {
        if (!m_colors) {
            m_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            m_colors->SetName("Colors");
            m_colors->SetNumberOfComponents(3);
        }
        m_colors->Initialize();
        m_colors->SetNumberOfTuples(count);
        for (vtkIdType index = 0; index < count; ++index) {
            const std::array<unsigned char, 3>& color = cloud.colors[static_cast<std::size_t>(index)];
            m_colors->SetTuple3(index, color[0], color[1], color[2]);
        }
        m_colors->Modified();
        m_polyData->GetPointData()->SetScalars(m_colors);
        m_mapper->ScalarVisibilityOn();
        m_mapper->SetScalarModeToUsePointData();
        // The array already holds 0-255 bytes, so it is used as-is.
        m_mapper->SetColorModeToDirectScalars();
    } else {
        m_polyData->GetPointData()->SetScalars(nullptr);
        m_mapper->ScalarVisibilityOff();
    }

    m_polyData->Modified();
    m_hasCloud = true;
    m_actor->SetVisibility(m_visible ? 1 : 0);
    return true;
}

void PointCloudActor::clear()
{
    m_points->Initialize();
    m_points->Modified();
    m_vertices->Initialize();
    m_vertices->Modified();
    if (m_colors) {
        m_colors->Initialize();
    }
    m_polyData->GetPointData()->SetScalars(nullptr);
    m_polyData->Modified();
    m_mapper->ScalarVisibilityOff();
    m_hasCloud = false;
    m_actor->SetVisibility(0);
}

bool PointCloudActor::hasCloud() const noexcept
{
    return m_hasCloud;
}

int PointCloudActor::pointCount() const noexcept
{
    return m_hasCloud ? static_cast<int>(m_points->GetNumberOfPoints()) : 0;
}

void PointCloudActor::setVisible(bool visible)
{
    m_visible = visible;
    m_actor->SetVisibility(m_visible && m_hasCloud ? 1 : 0);
}

void PointCloudActor::setPointSize(double size)
{
    m_actor->GetProperty()->SetPointSize(std::clamp(size, 0.5, 20.0));
}

void PointCloudActor::bounds(double out[6]) const
{
    if (!m_hasCloud) {
        for (int index = 0; index < 6; ++index) {
            out[index] = 0.0;
        }
        return;
    }
    m_polyData->GetBounds(out);
}

vtkActor* PointCloudActor::actor() const noexcept
{
    return m_actor;
}