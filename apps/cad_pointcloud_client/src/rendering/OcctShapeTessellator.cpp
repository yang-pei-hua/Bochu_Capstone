#include "rendering/OcctShapeTessellator.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>

#include <utility>

vtkSmartPointer<vtkPolyData> OcctShapeTessellator::tessellate(
    const TopoDS_Shape& shape,
    double linearDeflection,
    double angularDeflection)
{
    vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
    if (shape.IsNull()) {
        return result;
    }

    // Let OpenCASCADE build the read-only triangulation cache for this shape.
    BRepMesh_IncrementalMesh mesher(shape, linearDeflection, Standard_False,
                                    angularDeflection);

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;

    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        const TopoDS_Face& face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull()) {
            continue;
        }

        const gp_Trsf transform = location.Transformation();
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        const vtkIdType nodeOffset = points->GetNumberOfPoints();

        for (Standard_Integer node = 1; node <= triangulation->NbNodes(); ++node) {
            const gp_Pnt point = triangulation->Node(node).Transformed(transform);
            points->InsertNextPoint(point.X(), point.Y(), point.Z());
        }

        for (Standard_Integer index = 1; index <= triangulation->NbTriangles(); ++index) {
            Standard_Integer first = 0;
            Standard_Integer second = 0;
            Standard_Integer third = 0;
            triangulation->Triangle(index).Get(first, second, third);
            if (reversed) {
                std::swap(second, third);
            }
            triangles->InsertNextCell(3);
            triangles->InsertCellPoint(nodeOffset + first - 1);
            triangles->InsertCellPoint(nodeOffset + second - 1);
            triangles->InsertCellPoint(nodeOffset + third - 1);
        }
    }

    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);
    mesh->SetPolys(triangles);

    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(mesh);
    normals->SplittingOn();
    normals->ComputePointNormalsOn();
    normals->Update();

    result->ShallowCopy(normals->GetOutput());
    return result;
}
