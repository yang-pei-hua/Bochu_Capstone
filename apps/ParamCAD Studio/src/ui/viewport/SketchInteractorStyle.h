#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <functional>
#include <string>

// SolidWorks-like navigation for the modeling viewport.
//
// The left button is reserved for modeling (pick a face, place a sketch point),
// so it never moves the camera. The camera follows the mouse only while a button
// is held: hold the middle button to rotate, hold the right button to pan, and
// the wheel zooms. No modifier key is involved.
//
// A right press that does not drag is still a cancel, so the button keeps its
// old meaning whenever it is not being used to move the camera.
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
    void OnMiddleButtonUp() override;
    void OnRightButtonDown() override;
    void OnRightButtonUp() override;
    void OnMouseMove() override;
    void OnKeyPress() override;

    // Display-pixel position of the left-button press.
    std::function<void(int, int)> leftButtonPressed;
    // Display-pixel position of the cursor, reported before the camera update.
    std::function<void(int, int)> mouseMoved;
    // The right button cancels the pending tool operation when it is clicked,
    // but not when it was dragged to pan the camera.
    std::function<void()> cancelRequested;
    // A pan drag is under way and the camera is about to move. Reported before
    // the camera does, so a listener can get out of the way of the pan.
    std::function<void()> panMoved;
    // VTK key symbol of a key pressed while the viewport has focus, such as
    // "Delete". The style only reports it; the viewer decides what it means.
    std::function<void(const std::string&)> keyPressed;

protected:
    SketchInteractorStyle() = default;
    ~SketchInteractorStyle() override = default;

private:
    SketchInteractorStyle(const SketchInteractorStyle&) = delete;
    void operator=(const SketchInteractorStyle&) = delete;

    // Where the right button went down, so a release can tell a cancel click
    // from a pan drag.
    int m_rightPressX = 0;
    int m_rightPressY = 0;
};
