#include "ui/viewport/SketchInteractorStyle.h"

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <cstdlib>

namespace {

// A right press that moves less than this many display pixels stays a click, so
// the cancel the right button has always sent survives the pan binding.
constexpr int kRightClickSlop = 3;

}  // namespace

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
    this->StartRotate();
}

void SketchInteractorStyle::OnMiddleButtonUp()
{
    // The camera only follows the mouse while the button is down, so the rotate
    // started on the press is closed here no matter what else is going on.
    if (this->State == VTKIS_ROTATE) {
        this->EndRotate();
    }
    vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
}

void SketchInteractorStyle::OnRightButtonDown()
{
    if (this->Interactor == nullptr) {
        return;
    }
    this->FindPokedRenderer(this->Interactor->GetEventPosition()[0],
                            this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }
    const int* position = this->Interactor->GetEventPosition();
    m_rightPressX = position[0];
    m_rightPressY = position[1];
    this->GrabFocus(this->EventCallbackCommand);
    this->StartPan();
}

void SketchInteractorStyle::OnRightButtonUp()
{
    if (this->State == VTKIS_PAN) {
        this->EndPan();
    }
    vtkInteractorStyleTrackballCamera::OnRightButtonUp();

    if (this->Interactor == nullptr || cancelRequested == nullptr) {
        return;
    }
    // Moving the camera is what a drag meant, so it must not also cancel the
    // tool the user is in the middle of.
    const int* position = this->Interactor->GetEventPosition();
    if (std::abs(position[0] - m_rightPressX) <= kRightClickSlop
        && std::abs(position[1] - m_rightPressY) <= kRightClickSlop) {
        cancelRequested();
    }
}

void SketchInteractorStyle::OnMouseMove()
{
    if (this->Interactor != nullptr) {
        const int* position = this->Interactor->GetEventPosition();
        if (mouseMoved != nullptr) {
            mouseMoved(position[0], position[1]);
        }
        // Before the camera moves, so that whatever would answer the pan with a
        // camera of its own has already stood down by the time it is applied.
        if (panMoved != nullptr && this->State == VTKIS_PAN) {
            panMoved();
        }
    }
    vtkInteractorStyleTrackballCamera::OnMouseMove();
}

void SketchInteractorStyle::OnKeyPress()
{
    if (keyPressed == nullptr || this->Interactor == nullptr) {
        return;
    }
    const char* keySym = this->Interactor->GetKeySym();
    if (keySym != nullptr) {
        keyPressed(std::string(keySym));
    }
}
