#pragma once

#include "rendering/BodyActor.h"
#include "io/PlyReader.h"

#include <memory>
#include <vector>

class vtkRenderer;
namespace reconstruction { struct ProjectionOperation; }

// Transient debug geometry. It is kept separate from the CAD body and the
// editable sketch actors so previewing an operation cannot change either.
class ProjectionPreviewActor
{
public:
    ProjectionPreviewActor();
    ~ProjectionPreviewActor();
    void attach(vtkRenderer* scene, vtkRenderer* guides);
    void setBody(const TopoDS_Shape& shape, double scaleToCloud);
    void setBodyVisible(bool visible);
    void setOperations(const std::vector<reconstruction::ProjectionOperation>& operations,
                       const PlyCloud& cloud, double scaleToCloud, int selectedStep);
    void clear();

private:
    BodyActor m_body;
    class OperationGuide;
    std::vector<std::unique_ptr<OperationGuide>> m_guides;
    vtkRenderer* m_guideRenderer = nullptr;
};
