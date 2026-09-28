#pragma once

#include <TopoDS_Shape.hxx>

#include <vtkSmartPointer.h>

#include <vector>

class vtkPolyData;

// Display-side description of one OpenCASCADE face of a modeled body. The cells
// of the mesh carry a "FaceIds" array whose values index into a vector of these,
// so a picked display cell can be mapped back to the face - and to the plane -
// it came from.
struct BodyFaceInfo {
    bool planar = false;
    double origin[3]{0.0, 0.0, 0.0};
    double normal[3]{0.0, 0.0, 1.0};
    double xDirection[3]{1.0, 0.0, 0.0};
    vtkIdType firstCell = 0;
    vtkIdType cellCount = 0;
};

struct BodyTessellation {
    vtkSmartPointer<vtkPolyData> mesh;
    std::vector<BodyFaceInfo> faces;

    bool empty() const noexcept;
};

// Read-only adapter that turns a modeled OpenCASCADE shape into a display mesh
// plus a per-face index. It never mutates the shape and never feeds data back
// into the core.
class OcctShapeTessellator
{
public:
    static BodyTessellation tessellate(const TopoDS_Shape& shape,
                                       double linearDeflection = 0.1,
                                       double angularDeflection = 0.5);
};
