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

// The base camera style renders synchronously from Rotate/Pan/Dolly and from
// EndState. Suppress only those calls, restoring the previous interactor flag
// before handing the render request back to the Qt-owned viewer.
class DeferredCameraRender
{
public:
    DeferredCameraRender(vtkRenderWindowInteractor* interactor, bool defer)
        : m_interactor(defer ? interactor : nullptr)
        , m_enabled(m_interactor ? m_interactor->GetEnableRender() : false)
    {
        if (m_interactor) m_interactor->EnableRenderOff();
    }
    ~DeferredCameraRender()
    {
        if (m_interactor) m_interactor->SetEnableRender(m_enabled);
    }
private:
    vtkRenderWindowInteractor* m_interactor;
    bool m_enabled;
};

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
    if (cameraInteractionStarted) cameraInteractionStarted();
    this->StartRotate();
}

void SketchInteractorStyle::OnMiddleButtonUp()
{
    // The camera only follows the mouse while the button is down, so the rotate
    // started on the press is closed here no matter what else is going on.
    const bool rotating = this->State == VTKIS_ROTATE;
    {
        DeferredCameraRender defer(this->Interactor, bool(renderRequested));
        if (rotating) this->EndRotate();
        vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
    }
    // The base middle-button handler only releases a pan, whereas this binding
    // owns a rotate. Release the focus grabbed on our middle-button press.
    if (rotating) {
        this->ReleaseFocus();
        if (cameraInteractionFinished) cameraInteractionFinished();
    }
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
    if (cameraInteractionStarted) cameraInteractionStarted();
    this->StartPan();
}

void SketchInteractorStyle::OnRightButtonUp()
{
    const bool panning = this->State == VTKIS_PAN;
    {
        DeferredCameraRender defer(this->Interactor, bool(renderRequested));
        if (panning) this->EndPan();
        vtkInteractorStyleTrackballCamera::OnRightButtonUp();
    }
    if (panning) {
        this->ReleaseFocus();
        if (cameraInteractionFinished) cameraInteractionFinished();
    }

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
    const bool navigating = this->State == VTKIS_ROTATE || this->State == VTKIS_PAN
        || this->State == VTKIS_DOLLY || this->State == VTKIS_SPIN;
    {
        DeferredCameraRender defer(this->Interactor, navigating && bool(renderRequested));
        vtkInteractorStyleTrackballCamera::OnMouseMove();
    }
    if (navigating && renderRequested) renderRequested();
}

void SketchInteractorStyle::OnMouseWheelForward()
{
    {
        DeferredCameraRender defer(this->Interactor, bool(renderRequested));
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
    }
    if (renderRequested) renderRequested();
}

void SketchInteractorStyle::OnMouseWheelBackward()
{
    {
        DeferredCameraRender defer(this->Interactor, bool(renderRequested));
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
    if (renderRequested) renderRequested();
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
