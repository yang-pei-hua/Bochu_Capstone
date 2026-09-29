#pragma once

#include "io/PlyReader.h"

#include <vtkSmartPointer.h>

class vtkActor;
class vtkCellArray;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkUnsignedCharArray;

// Owns the VTK pipeline that draws one reconstructed point cloud. Like
// BodyActor it is a display cache: the cloud on disk stays the source of truth.
class PointCloudActor
{
public:
    PointCloudActor();

    bool setCloud(const PlyCloud& cloud); // false for an empty cloud, which also clears
    void clear();
    bool hasCloud() const noexcept;
    int pointCount() const noexcept;

    void setVisible(bool visible);
    void setPointSize(double size);

    // Bounds of the loaded cloud, or six zeroes when there is none.
    void bounds(double out[6]) const;

    vtkActor* actor() const noexcept;

private:
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkCellArray> m_vertices;
    vtkSmartPointer<vtkUnsignedCharArray> m_colors;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    bool m_hasCloud = false;
    bool m_visible = true;
};