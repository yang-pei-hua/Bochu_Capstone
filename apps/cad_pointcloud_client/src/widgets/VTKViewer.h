#pragma once

#include "core/CameraController.h"

#include <QColor>
#include <QString>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkSmartPointer.h>

class vtkActor;
class vtkAxesActor;
class vtkCallbackCommand;
class vtkGenericOpenGLRenderWindow;
class vtkObject;
class vtkOrientationMarkerWidget;
class vtkRenderer;

class VTKViewer final : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    explicit VTKViewer(QWidget* parent = nullptr);
    ~VTKViewer() override;

    CameraParameters cameraParameters() const;
    bool captureImage(const QString& filePath, int width, int height);

public slots:
    void resetCamera();
    void setStandardView(CameraController::StandardView view);
    void applyCamera(const CameraParameters& parameters);
    void setParallelProjection(bool enabled);

    void setObjectTransform(double px, double py, double pz,
                            double rx, double ry, double rz,
                            double sx, double sy, double sz);
    void setObjectVisible(bool visible);
    void setObjectRepresentation(int representation);
    void setObjectOpacity(double opacity);
    void setObjectColor(const QColor& color);
    void setBackgroundColor(const QColor& color);
    void setLightingEnabled(bool enabled);

signals:
    void cameraChanged(const CameraParameters& parameters);

private:
    static void onCameraModified(vtkObject* caller, unsigned long eventId,
                                 void* clientData, void* callData);
    void render();
    void createDemoCube();
    void createOrientationAxes();

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkActor> m_cubeActor;
    vtkSmartPointer<vtkAxesActor> m_axesActor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_orientationWidget;
    vtkSmartPointer<vtkCallbackCommand> m_cameraCallback;
    unsigned long m_cameraObserverTag = 0;
};
