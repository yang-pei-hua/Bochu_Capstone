#include "widgets/VTKViewer.h"

#include <QByteArray>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageResize.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>

VTKViewer::VTKViewer(QWidget* parent)
    : QVTKOpenGLNativeWidget(parent)
    , m_renderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
    , m_renderer(vtkSmartPointer<vtkRenderer>::New())
{
    setObjectName(QStringLiteral("vtkViewport"));
    setMinimumSize(480, 320);
    setFocusPolicy(Qt::StrongFocus);

    setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);
    m_renderer->SetBackground(0.10, 0.11, 0.13);
    m_renderer->SetBackground2(0.17, 0.18, 0.21);
    m_renderer->GradientBackgroundOn();

    vtkNew<vtkInteractorStyleTrackballCamera> interactionStyle;
    m_renderWindow->GetInteractor()->SetInteractorStyle(interactionStyle);

    m_renderer->AddActor(m_bodyActor.surfaceActor());
    m_renderer->AddActor(m_bodyActor.edgeActor());
    createOrientationAxes();

    CameraParameters initialCamera;
    CameraController::apply(m_renderer->GetActiveCamera(), initialCamera);

    m_cameraCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_cameraCallback->SetClientData(this);
    m_cameraCallback->SetCallback(&VTKViewer::onCameraModified);
    m_cameraObserverTag = m_renderer->GetActiveCamera()->AddObserver(
        vtkCommand::ModifiedEvent, m_cameraCallback);

    renderNow();
}
VTKViewer::~VTKViewer()
{
    if (m_renderer && m_renderer->GetActiveCamera() && m_cameraObserverTag != 0) {
        m_renderer->GetActiveCamera()->RemoveObserver(m_cameraObserverTag);
    }
}

CameraParameters VTKViewer::cameraParameters() const
{
    return CameraController::parameters(m_renderer->GetActiveCamera());
}

void VTKViewer::setBodyShape(const TopoDS_Shape& shape)
{
    m_bodyActor.setShape(shape);
    m_renderer->ResetCameraClippingRange();
}

void VTKViewer::clearBody()
{
    m_bodyActor.clear();
    m_renderer->ResetCameraClippingRange();
}

bool VTKViewer::hasBody() const noexcept
{
    return m_bodyActor.hasShape();
}

void VTKViewer::renderNow()
{
    m_renderWindow->Render();
}

bool VTKViewer::captureImage(const QString& filePath, int width, int height)
{
    if (filePath.isEmpty() || width <= 0 || height <= 0) {
        return false;
    }

    renderNow();

    vtkNew<vtkWindowToImageFilter> windowCapture;
    windowCapture->SetInput(m_renderWindow);
    windowCapture->SetInputBufferTypeToRGB();
    windowCapture->ReadFrontBufferOff();
    windowCapture->Update();

    vtkNew<vtkImageResize> resize;
    resize->SetInputConnection(windowCapture->GetOutputPort());
    resize->SetResizeMethodToOutputDimensions();
    resize->SetOutputDimensions(width, height, 1);
    resize->InterpolateOn();

    vtkNew<vtkPNGWriter> writer;
    const QByteArray nativePath = filePath.toLocal8Bit();
    writer->SetFileName(nativePath.constData());
    writer->SetInputConnection(resize->GetOutputPort());
    writer->Write();
    return writer->GetErrorCode() == 0;
}

void VTKViewer::resetCamera()
{
    m_renderer->ResetCamera();
    m_renderer->ResetCameraClippingRange();
    renderNow();
    emit cameraChanged(cameraParameters());
}

void VTKViewer::setStandardView(CameraController::StandardView view)
{
    CameraController::applyStandardView(m_renderer->GetActiveCamera(), view);
    m_renderer->ResetCamera();
    m_renderer->ResetCameraClippingRange();
    renderNow();
    emit cameraChanged(cameraParameters());
}

void VTKViewer::applyCamera(const CameraParameters& parameters)
{
    CameraController::apply(m_renderer->GetActiveCamera(), parameters);
    m_renderer->ResetCameraClippingRange();
    renderNow();
    emit cameraChanged(cameraParameters());
}

void VTKViewer::setParallelProjection(bool enabled)
{
    m_renderer->GetActiveCamera()->SetParallelProjection(enabled ? 1 : 0);
    renderNow();
    emit cameraChanged(cameraParameters());
}

void VTKViewer::setObjectTransform(double px, double py, double pz,
                                   double rx, double ry, double rz,
                                   double sx, double sy, double sz)
{
    m_bodyActor.setTransform(px, py, pz, rx, ry, rz, sx, sy, sz);
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

void VTKViewer::setObjectVisible(bool visible)
{
    m_bodyActor.setVisible(visible);
    renderNow();
}

void VTKViewer::setObjectRepresentation(int representation)
{
    m_bodyActor.setRepresentation(representation);
    renderNow();
}

void VTKViewer::setObjectOpacity(double opacity)
{
    m_bodyActor.setOpacity(std::clamp(opacity, 0.0, 1.0));
    renderNow();
}

void VTKViewer::setObjectColor(const QColor& color)
{
    m_bodyActor.setColor(color);
    renderNow();
}

void VTKViewer::setBackgroundColor(const QColor& color)
{
    m_renderer->GradientBackgroundOff();
    m_renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
    renderNow();
}

void VTKViewer::setLightingEnabled(bool enabled)
{
    m_bodyActor.setLightingEnabled(enabled);
    renderNow();
}

void VTKViewer::onCameraModified(vtkObject*, unsigned long, void* clientData, void*)
{
    auto* viewer = static_cast<VTKViewer*>(clientData);
    if (viewer) {
        emit viewer->cameraChanged(viewer->cameraParameters());
    }
}

void VTKViewer::createOrientationAxes()
{
    m_axesActor = vtkSmartPointer<vtkAxesActor>::New();
    m_axesActor->SetTotalLength(0.9, 0.9, 0.9);
    m_axesActor->SetShaftTypeToCylinder();

    m_orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_orientationWidget->SetOrientationMarker(m_axesActor);
    m_orientationWidget->SetInteractor(m_renderWindow->GetInteractor());
    m_orientationWidget->SetViewport(0.0, 0.0, 0.18, 0.25);
    m_orientationWidget->SetEnabled(1);
    m_orientationWidget->InteractiveOff();
}
