#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <functional>
#include <string>

// SolidWorks-like navigation for the modeling viewport.
//
// The left button is reserved for modeling (pick a face, place a sketch point),
// so it never moves the camera. The camera lives on the middle button: drag to
// rotate, Ctrl+drag to pan, Shift+drag to zoom, and the wheel still zooms.
//
// The style itself knows nothing about picking; it only reports the raw events
// through the callbacks below, and the viewer decides what they mean.
class SketchInteractorStyle final : public vtkInteractorStyleTrackballCamera
{
public:
    static SketchInteractorStyle* New();
    vtkTypeMacro(SketchInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override;
    void OnMiddleButtonDown() override;
    void OnRightButtonDown() override;
    void OnMouseMove() override;
    void OnKeyPress() override;

    // Display-pixel position of the left-button press.
    std::function<void(int, int)> leftButtonPressed;
    // Display-pixel position of the cursor, reported before the camera update.
    std::function<void(int, int)> mouseMoved;
    // The right button cancels the pending tool operation.
    std::function<void()> cancelRequested;
    // VTK key symbol of a key pressed while the viewport has focus, such as
    // "Delete". The style only reports it; the viewer decides what it means.
    std::function<void(const std::string&)> keyPressed;

protected:
    SketchInteractorStyle() = default;
    ~SketchInteractorStyle() override = default;

private:
    SketchInteractorStyle(const SketchInteractorStyle&) = delete;
    void operator=(const SketchInteractorStyle&) = delete;
};
