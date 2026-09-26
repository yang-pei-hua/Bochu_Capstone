#pragma once

#include "core/CameraController.h"
#include "rendering/BodyActor.h"
#include "rendering/PointCloudActor.h"

#include <QColor>
#include <QSize>
#include <QString>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkSmartPointer.h>

class TopoDS_Shape;
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

    // Pixel size of the render window that captureImage() resizes from. It
    // differs from the requested output size, which is what makes the
    // horizontal and vertical focal lengths of a capture differ.
    QSize captureSourceSize() const;

    // Replaces the displayed body with the triangulation of the given core
    // shape. The viewer only consumes a shape; it never interprets features.
    void setBodyShape(const TopoDS_Shape& shape);
    void clearBody();
    bool hasBody() const noexcept;
    void renderNow();

    // Loads a PLY point cloud from disk. It coexists with the body actor.
    bool loadPointCloud(const QString& plyPath, QString& error);
    void clearPointCloud();
    bool hasPointCloud() const noexcept;
    int pointCloudPointCount() const noexcept;
    void resetCameraToPointCloud();

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
    void setPointCloudVisible(bool visible);
    void setPointCloudPointSize(double size);

signals:
    void cameraChanged(const CameraParameters& parameters);

private:
    static void onCameraModified(vtkObject* caller, unsigned long eventId,
                                 void* clientData, void* callData);
    void createOrientationAxes();

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    BodyActor m_bodyActor;
    PointCloudActor m_pointCloudActor;
    vtkSmartPointer<vtkAxesActor> m_axesActor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_orientationWidget;
    vtkSmartPointer<vtkCallbackCommand> m_cameraCallback;
    unsigned long m_cameraObserverTag = 0;
};
