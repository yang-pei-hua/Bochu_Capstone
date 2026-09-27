#include "ui/viewport/SketchInteractorStyle.h"

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

vtkStandardNewMacro(SketchInteractorStyle);

void SketchInteractorStyle::OnLeftButtonDown()
{
    if (this->Interactor == nullptr) {
        return;
    }
    if (leftButtonPressed != nullptr) {
        const int* position = this->Interactor->GetEventPosition();
        leftButtonPressed(position[0], position[1]);
    }
    // Deliberately not forwarded to the camera style: a left click always means
    // "pick or place", never "rotate".
}

void SketchInteractorStyle::OnMiddleButtonDown()
{
    if (this->Interactor == nullptr) {
        return;
    }
    this->FindPokedRenderer(this->Interactor->GetEventPosition()[0],
                            this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }
    this->GrabFocus(this->EventCallbackCommand);

    if (this->Interactor->GetControlKey()) {
        this->StartPan();
    } else if (this->Interactor->GetShiftKey()) {
        this->StartDolly();
    } else {
        this->StartRotate();
    }
}

void SketchInteractorStyle::OnRightButtonDown()
{
    if (cancelRequested != nullptr) {
        cancelRequested();
    }
}

void SketchInteractorStyle::OnMouseMove()
{
    if (mouseMoved != nullptr && this->Interactor != nullptr) {
        const int* position = this->Interactor->GetEventPosition();
        mouseMoved(position[0], position[1]);
    }
    vtkInteractorStyleTrackballCamera::OnMouseMove();
}
