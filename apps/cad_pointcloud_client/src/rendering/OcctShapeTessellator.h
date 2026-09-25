#pragma once

#include <TopoDS_Shape.hxx>

#include <vtkSmartPointer.h>

class vtkPolyData;

// Read-only adapter that turns a modeled OpenCASCADE shape into a display
// mesh. It never mutates the shape and never feeds data back into the core.
class OcctShapeTessellator
{
public:
    static vtkSmartPointer<vtkPolyData> tessellate(const TopoDS_Shape& shape,
                                                   double linearDeflection = 0.1,
                                                   double angularDeflection = 0.5);
};
