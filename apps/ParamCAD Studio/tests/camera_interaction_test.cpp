#include "app/MainWindow.h"
#include "ui/panels/CameraPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QImage>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
bool sameVector(const double a[3], const double b[3])
{
    for (int i = 0; i < 3; ++i) if (std::abs(a[i] - b[i]) > 1e-8) return false;
    return true;
}
void drainEvents(int milliseconds)
{
    QElapsedTimer timer;
    timer.start();
    do {
        QApplication::processEvents(QEventLoop::AllEvents, 1);
        QThread::msleep(1);
    } while (timer.elapsed() < milliseconds);
}
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    try {
        MainWindow window;
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        app.processEvents();
        auto* viewer = window.findChild<VTKViewer*>();
        auto* panel = window.findChild<CameraPanel*>();
        auto* renderWindow = viewer->renderWindow();
        auto* interactor = renderWindow->GetInteractor();
        auto* style = static_cast<SketchInteractorStyle*>(interactor->GetInteractorStyle());
        panel->setOriginLock(false);
        viewer->setBodyShape(BRepPrimAPI_MakeBox(30, 40, 50).Shape());
        viewer->resetCamera();
        drainEvents(25);

        int renders = 0;
        int notifications = 0;
        CameraParameters lastNotified;
        vtkNew<vtkCallbackCommand> renderCounter;
        renderCounter->SetClientData(&renders);
        renderCounter->SetCallback([](vtkObject*, unsigned long, void* data, void*) {
            ++*static_cast<int*>(data);
        });
        const auto renderTag = renderWindow->AddObserver(vtkCommand::StartEvent, renderCounter);
        const auto connection = QObject::connect(viewer, &VTKViewer::cameraChanged,
            &window, [&](const CameraParameters& camera) {
                ++notifications;
                lastNotified = camera;
            });

        auto initial = viewer->cameraParameters();
        notifications = 0;
        viewer->applyCamera(initial);
        require(notifications == 1, "camera commands publish one complete pose synchronously");
        drainEvents(25);
        require(notifications == 1, "camera commands do not leave a duplicate queued notification");

        const int cx = renderWindow->GetSize()[0] / 2;
        const int cy = renderWindow->GetSize()[1] / 2;
        interactor->SetEventPosition(cx, cy);
        style->OnMiddleButtonDown();
        renders = notifications = 0;
        const auto beforeRotate = viewer->cameraParameters();
        for (int i = 1; i <= 80; ++i) {
            interactor->SetEventPosition(cx + i, cy + i / 3);
            style->OnMouseMove();
        }
        require(!sameVector(beforeRotate.position, viewer->cameraParameters().position),
                "camera follows every input without waiting for a scheduled frame");
        require(renders == 0 && notifications == 0,
                "a burst of mouse events does not synchronously render or rewrite camera fields");
        require(interactor->GetEnableRender(), "camera style restores the interactor render flag");
        style->OnMiddleButtonUp();
        require(renders == 1 && notifications == 1,
                "release immediately publishes and renders the final pose of a short gesture");
        require(sameVector(lastNotified.position, viewer->cameraParameters().position)
                    && sameVector(lastNotified.up, viewer->cameraParameters().up),
                "release feedback uses the final camera rather than an intermediate pose");
        drainEvents(70);
        require(renders == 1 && notifications == 1, "release cancels stale queued work");

        interactor->SetEventPosition(cx, cy);
        style->OnMiddleButtonDown();
        renders = notifications = 0;
        int moves = 0;
        QElapsedTimer drag;
        drag.start();
        while (drag.elapsed() < 280) {
            ++moves;
            interactor->SetEventPosition(cx + moves % 90, cy + moves % 27);
            style->OnMouseMove();
            app.processEvents(QEventLoop::AllEvents, 1);
            QThread::msleep(1);
        }
        const auto duration = drag.elapsed();
        require(notifications >= 2 && notifications <= duration / 50 + 2,
                "long gestures refresh camera feedback periodically at a bounded rate");
        require(renders >= 3 && renders <= moves + 1,
                "long gestures keep producing frames as input arrives");
        style->OnMiddleButtonUp();
        require(sameVector(lastNotified.position, viewer->cameraParameters().position),
                "long gesture flushes its final camera feedback on release");

        // A request immediately after a completed frame must not wait behind a
        // fixed 60 Hz limiter on a high-refresh display. Burst coalescing above
        // is still required, but a new event-loop turn can render immediately.
        viewer->renderNow();
        interactor->SetEventPosition(cx, cy);
        style->OnMiddleButtonDown();
        renders = 0;
        interactor->SetEventPosition(cx + 3, cy + 1);
        style->OnMouseMove();
        app.processEvents();
        require(renders >= 1, "a fresh frame is available on the next turn without a software frame cap");
        style->OnMiddleButtonUp();

        panel->setOriginLock(true);
        const auto beforePan = viewer->cameraParameters();
        interactor->SetEventPosition(cx, cy);
        style->OnRightButtonDown();
        interactor->SetEventPosition(cx + 40, cy + 20);
        style->OnMouseMove();
        style->OnRightButtonUp();
        require(!panel->originLocked()
                    && !sameVector(beforePan.target, viewer->cameraParameters().target),
                "deferred feedback preserves right-button panning away from the origin");
        require(sameVector(lastNotified.target, viewer->cameraParameters().target),
                "pan release reports the translated focal point");

        drainEvents(25);
        const auto beforeZoom = viewer->cameraParameters();
        renders = notifications = 0;
        interactor->SetEventPosition(cx, cy);
        for (int i = 0; i < 4; ++i) style->OnMouseWheelForward();
        require(renders == 0, "wheel bursts share the deferred renderer");
        require(!sameVector(beforeZoom.position, viewer->cameraParameters().position),
                "wheel zoom changes the camera immediately");
        drainEvents(30);
        require(renders == 1 && notifications == 1,
                "wheel burst renders and publishes the completed pose once");

        QTemporaryDir temp;
        require(temp.isValid(), "temporary capture directory");
        interactor->SetEventPosition(cx, cy);
        style->OnMiddleButtonDown();
        interactor->SetEventPosition(cx + 60, cy + 15);
        style->OnMouseMove();
        const auto cameraAtCapture = viewer->cameraParameters();
        const auto first = temp.filePath("pending-frame.png");
        require(viewer->captureImage(first, 320, 240), "capture flushes pending navigation frames");
        require(sameVector(cameraAtCapture.position, viewer->cameraParameters().position),
                "capture does not change the pending camera pose");
        style->OnMiddleButtonUp();
        const auto second = temp.filePath("settled-frame.png");
        require(viewer->captureImage(second, 320, 240), "capture after navigation settles");
        const QImage pending(first), settled(second);
        require(!pending.isNull() && pending == settled,
                "capture during a pending frame matches the final rendered model");

        QObject::disconnect(connection);
        renderWindow->RemoveObserver(renderTag);
        std::cout << "Camera interaction scheduling tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
