#include "core/CameraController.h"

#include <vtkCamera.h>

CameraParameters CameraController::parameters(vtkCamera* camera)
{
    CameraParameters result;
    camera->GetPosition(result.position);
    camera->GetFocalPoint(result.target);
    camera->GetViewUp(result.up);
    result.fieldOfView = camera->GetViewAngle();
    result.parallelProjection = camera->GetParallelProjection() != 0;
    return result;
}

void CameraController::apply(vtkCamera* camera, const CameraParameters& parameters)
{
    camera->SetPosition(parameters.position);
    camera->SetFocalPoint(parameters.target);
    camera->SetViewUp(parameters.up);
    camera->SetViewAngle(parameters.fieldOfView);
    camera->SetParallelProjection(parameters.parallelProjection ? 1 : 0);
    camera->OrthogonalizeViewUp();
}

void CameraController::applyStandardView(vtkCamera* camera, StandardView view, double distance)
{
    double position[3]{0.0, 0.0, 0.0};
    double up[3]{0.0, 0.0, 1.0};

    switch (view) {
    case StandardView::Front:
        position[1] = -distance;
        break;
    case StandardView::Back:
        position[1] = distance;
        break;
    case StandardView::Left:
        position[0] = -distance;
        break;
    case StandardView::Right:
        position[0] = distance;
        break;
    case StandardView::Top:
        position[2] = distance;
        up[0] = 0.0;
        up[1] = 1.0;
        up[2] = 0.0;
        break;
    case StandardView::Bottom:
        position[2] = -distance;
        up[0] = 0.0;
        up[1] = -1.0;
        up[2] = 0.0;
        break;
    }

    camera->SetPosition(position);
    camera->SetFocalPoint(0.0, 0.0, 0.0);
    camera->SetViewUp(up);
    camera->OrthogonalizeViewUp();
}
