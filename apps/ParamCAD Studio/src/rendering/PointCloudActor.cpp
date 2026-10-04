#include "rendering/PointCloudActor.h"
#include "core/PointCloudSelection.h"

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
    m_deleted.assign(cloud.positions.size(), 0);
    m_sourceColors = cloud.colors;

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
    m_deleted.clear();
    m_sourceColors.clear();
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
    return m_hasCloud ? static_cast<int>(std::count(m_deleted.begin(), m_deleted.end(), 0)) : 0;
}

void PointCloudActor::setVisible(bool visible)
{
    m_visible = visible;
    m_actor->SetVisibility(m_visible && m_hasCloud && pointCount() > 0 ? 1 : 0);
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
    bool first = true;
    double p[3];
    for (vtkIdType i = 0; i < m_points->GetNumberOfPoints(); ++i) {
        if (m_deleted[static_cast<size_t>(i)]) continue;
        m_points->GetPoint(i, p);
        for (int axis = 0; axis < 3; ++axis) {
            out[2*axis] = first ? p[axis] : std::min(out[2*axis], p[axis]);
            out[2*axis+1] = first ? p[axis] : std::max(out[2*axis+1], p[axis]);
        }
        first = false;
    }
    if (first) std::fill(out, out + 6, 0.0);
}

vtkActor* PointCloudActor::actor() const noexcept
{
    return m_actor;
}

void PointCloudActor::setPointMask(const std::vector<unsigned char>& deleted)
{
    if (deleted.size() != static_cast<size_t>(m_points->GetNumberOfPoints())) return;
    if (deleted == m_deleted) return;
    m_deleted = deleted;
    m_vertices->Initialize();
    m_vertices->AllocateEstimate(static_cast<vtkIdType>(deleted.size()), 1);
    for (vtkIdType i = 0; i < m_points->GetNumberOfPoints(); ++i) if (!deleted[static_cast<size_t>(i)]) {
        m_vertices->InsertNextCell(1);
        m_vertices->InsertCellPoint(i);
    }
    m_vertices->Modified();
    m_polyData->Modified();
    setVisible(m_visible);
}

void PointCloudActor::setAnnotations(const PointCloudSelection* selection)
{
    if (!m_hasCloud) return;
    const vtkIdType count = m_points->GetNumberOfPoints();
    if (!m_colors) {
        m_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        m_colors->SetName("Colors");
        m_colors->SetNumberOfComponents(3);
    }
    m_colors->SetNumberOfTuples(count);
    constexpr std::array<std::array<unsigned char, 3>, 8> palette{{
        {85, 205, 235}, {150, 220, 100}, {195, 130, 240}, {245, 120, 165},
        {80, 215, 170}, {230, 210, 90}, {125, 155, 245}, {215, 180, 145}}};
    for (vtkIdType i = 0; i < count; ++i) {
        auto color = m_sourceColors.size() == static_cast<size_t>(count)
            ? m_sourceColors[static_cast<size_t>(i)] : std::array<unsigned char, 3>{115, 184, 242};
        if (selection && selection->memberships().size() == static_cast<size_t>(count)) {
            const int region = selection->memberships()[static_cast<size_t>(i)];
            if (region > 0) color = palette[static_cast<size_t>(region - 1) % palette.size()];
        }
        m_colors->SetTypedTuple(i, color.data());
    }
    if (selection) {
        const unsigned char orange[3]{255, 145, 35};
        for (vtkIdType id : selection->selected()) {
            if (id >= 0 && id < count) m_colors->SetTypedTuple(id, orange);
        }
    }
    m_colors->Modified();
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_mapper->ScalarVisibilityOn();
    m_mapper->SetScalarModeToUsePointData();
    m_mapper->SetColorModeToDirectScalars();
}
