#pragma once

class vtkCamera;

struct CameraParameters
{
    double position[3]{3.0, 3.0, 3.0};
    double target[3]{0.0, 0.0, 0.0};
    double up[3]{0.0, 0.0, 1.0};
    double fieldOfView = 30.0;
    bool parallelProjection = false;
};

class CameraController
{
public:
    enum class StandardView
    {
        Front,
        Back,
        Left,
        Right,
        Top,
        Bottom
    };

    static CameraParameters parameters(vtkCamera* camera);
    static void apply(vtkCamera* camera, const CameraParameters& parameters);
    static void applyStandardView(vtkCamera* camera, StandardView view, double distance = 5.0);
};
