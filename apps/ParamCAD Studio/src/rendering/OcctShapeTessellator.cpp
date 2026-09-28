#include "rendering/OcctShapeTessellator.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>

#include <utility>

namespace {

// Records the exact plane of a planar face so a sketch can be anchored to it
// without guessing a plane from triangle normals. The face orientation is
// applied so the normal points away from the material, which is what makes an
// extrusion built on the plane grow outward.
void describeFace(const TopoDS_Face& face, BodyFaceInfo& info)
{
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) {
        return;
    }

    const gp_Pln plane = surface.Plane();
    const gp_Pnt location = plane.Location();
    gp_Dir normal = plane.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) {
        normal.Reverse();
    }
    const gp_Dir xDirection = plane.XAxis().Direction();

    info.planar = true;
    info.origin[0] = location.X();
    info.origin[1] = location.Y();
    info.origin[2] = location.Z();
    info.normal[0] = normal.X();
    info.normal[1] = normal.Y();
    info.normal[2] = normal.Z();
    info.xDirection[0] = xDirection.X();
    info.xDirection[1] = xDirection.Y();
    info.xDirection[2] = xDirection.Z();
}

}  // namespace

bool BodyTessellation::empty() const noexcept
{
    return mesh == nullptr || mesh->GetNumberOfPoints() == 0 ||
        mesh->GetNumberOfPolys() == 0;
}

BodyTessellation OcctShapeTessellator::tessellate(
    const TopoDS_Shape& shape,
    double linearDeflection,
    double angularDeflection)
{
    BodyTessellation result;
    result.mesh = vtkSmartPointer<vtkPolyData>::New();
    if (shape.IsNull()) {
        return result;
    }

    // Let OpenCASCADE build the read-only triangulation cache for this shape.
    BRepMesh_IncrementalMesh mesher(shape, linearDeflection, false,
                                    angularDeflection);

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;
    vtkNew<vtkIdTypeArray> faceIds;
    faceIds->SetName("FaceIds");

    int faceIndex = 0;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
         explorer.Next(), ++faceIndex) {
        const TopoDS_Face& face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);

        BodyFaceInfo info;
        info.firstCell = static_cast<vtkIdType>(triangles->GetNumberOfCells());
        describeFace(face, info);
        result.faces.push_back(info);

        if (triangulation.IsNull()) {
            continue;
        }

        const gp_Trsf transform = location.Transformation();
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        const vtkIdType nodeOffset = points->GetNumberOfPoints();

        for (int node = 1; node <= triangulation->NbNodes(); ++node) {
            const gp_Pnt point = triangulation->Node(node).Transformed(transform);
            points->InsertNextPoint(point.X(), point.Y(), point.Z());
        }

        for (int index = 1; index <= triangulation->NbTriangles(); ++index) {
            int first = 0;
            int second = 0;
            int third = 0;
            triangulation->Triangle(index).Get(first, second, third);
            if (reversed) {
                std::swap(second, third);
            }
            triangles->InsertNextCell(3);
            triangles->InsertCellPoint(nodeOffset + first - 1);
            triangles->InsertCellPoint(nodeOffset + second - 1);
            triangles->InsertCellPoint(nodeOffset + third - 1);
            faceIds->InsertNextValue(faceIndex);
        }

        result.faces.back().cellCount =
            static_cast<vtkIdType>(triangles->GetNumberOfCells()) - info.firstCell;
    }

    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);
    mesh->SetPolys(triangles);

    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(mesh);
    normals->SplittingOn();
    normals->ComputePointNormalsOn();
    normals->Update();

    // The normals filter only splits points; it neither reorders nor adds cells,
    // so the cell order (and therefore the face index array) stays valid.
    result.mesh->ShallowCopy(normals->GetOutput());
    result.mesh->GetCellData()->AddArray(faceIds);
    return result;
}
