#pragma once
#include <modeling/PrimitiveParams.h>
#include <vtkSmartPointer.h>
#include <array>
class vtkActor;
class vtkMatrix4x4;
class vtkRenderer;

// Semantic primitive faces; independent of the final B-Rep face IDs.
class BoxReferenceActor
{
public:
    void attach(vtkRenderer* renderer);
    void setBox(const modeling::BoxPrimitiveParams& box);
    void setTransform(vtkMatrix4x4* matrix);
    void setVisible(bool visible);
    int pick(vtkRenderer* renderer, int x, int y) const;
private:
    std::array<vtkSmartPointer<vtkActor>,7> m_actors;
};
