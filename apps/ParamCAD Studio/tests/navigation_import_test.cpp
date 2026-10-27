#include "app/MainWindow.h"
#include "app/ModelViewController.h"
#include "app/CadReconstructionTask.h"
#include "core/PointCloudDocument.h"
#include "ui/panels/CameraPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <QApplication>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFile>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QUrl>
#include <algorithm>
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
double distance(const CameraParameters& camera)
{
    double squared = 0;
    for (int i = 0; i < 3; ++i) squared += std::pow(camera.position[i] - camera.target[i], 2);
    return std::sqrt(squared);
}
double angle(const CameraParameters& a, const CameraParameters& b)
{
    double dot = 0;
    for (int i = 0; i < 3; ++i)
        dot += (a.position[i] - a.target[i]) * (b.position[i] - b.target[i]);
    return std::acos(std::clamp(dot / (distance(a) * distance(b)), -1.0, 1.0));
}
void press(QWidget* target, int key, Qt::KeyboardModifiers modifiers = Qt::NoModifier,
           bool repeat = false)
{
    QKeyEvent event(QEvent::KeyPress, key, modifiers, QString(), repeat);
    QApplication::sendEvent(target, &event);
}
bool drop(QWidget* target, const QList<QUrl>& urls)
{
    QMimeData mime;
    mime.setUrls(urls);
    QDragEnterEvent enter(QPoint(10, 10), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(target, &enter);
    if (!enter.isAccepted()) return false;
    QDragMoveEvent move(QPoint(10, 10), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(target, &move);
    require(move.isAccepted(), "accepted drag stays accepted while moving");
    QDropEvent event(QPointF(10, 10), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(target, &event);
    return event.isAccepted();
}
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary fixtures");
        MainWindow window;
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        window.activateWindow();
        app.processEvents();
        auto* viewer = window.findChild<VTKViewer*>();
        auto* panel = window.findChild<CameraPanel*>();
        auto* stepLabel = window.findChild<QLabel*>("keyboardRotationStep");
        panel->setOriginLock(false);
        CameraParameters base;
        base.target[0] = 10; base.target[1] = 20; base.target[2] = 30;
        base.position[0] = 13; base.position[1] = 24; base.position[2] = 35;
        base.up[0] = 0; base.up[1] = 0; base.up[2] = 1;
        viewer->applyCamera(base);
        base = viewer->cameraParameters();
        viewer->setFocus();

        press(viewer, Qt::Key_Left);
        const auto rotated = viewer->cameraParameters();
        require(angle(base, rotated) > 0.01 && sameVector(base.target, rotated.target)
                && std::abs(distance(base) - distance(rotated)) < 1e-8,
                "single arrow orbits the off-origin target without zooming");
        press(viewer, Qt::Key_Right);
        require(sameVector(base.position, viewer->cameraParameters().position),
                "opposite horizontal arrow reverses rotation");
        press(viewer, Qt::Key_Up);
        const auto pitched = viewer->cameraParameters();
        require(angle(base, pitched) > 0.01, "up arrow pitches camera");
        press(viewer, Qt::Key_Down);
        require(sameVector(base.position, viewer->cameraParameters().position),
                "opposite vertical arrow reverses rotation");
        for (int i = 0; i < 12; ++i) press(viewer, Qt::Key_Left, Qt::NoModifier, true);
        require(angle(base, viewer->cameraParameters()) > 0.5, "held-key repeat keeps rotating");
        viewer->applyCamera(base);
        const auto labelBefore = stepLabel->text();
        press(viewer, Qt::Key_Up, Qt::ControlModifier);
        require(stepLabel->text() != labelBefore && sameVector(base.position, viewer->cameraParameters().position),
                "Ctrl+Up changes sensitivity without rotating");
        press(viewer, Qt::Key_Left);
        require(angle(base, viewer->cameraParameters()) > angle(base, rotated),
                "increased sensitivity increases horizontal rotation");
        viewer->applyCamera(base);
        press(viewer, Qt::Key_Up);
        require(angle(base, viewer->cameraParameters()) > angle(base, pitched),
                "same sensitivity controls vertical rotation");
        viewer->applyCamera(base);
        press(viewer, Qt::Key_Down, Qt::ControlModifier);
        require(stepLabel->text() == labelBefore, "Ctrl+Down reduces sensitivity");
        for (int i = 0; i < 40; ++i) press(viewer, Qt::Key_Up, Qt::ControlModifier, true);
        require(stepLabel->text().contains("45.00"), "sensitivity has upper bound");
        for (int i = 0; i < 80; ++i) press(viewer, Qt::Key_Down, Qt::ControlModifier, true);
        require(stepLabel->text().contains("0.10"), "sensitivity has lower bound");

        viewer->setParallelProjection(true);
        viewer->setParallelScale(2.5);
        press(viewer, Qt::Key_Up);
        require(viewer->cameraParameters().parallelProjection && std::abs(viewer->parallelScale() - 2.5) < 1e-8,
                "orthographic rotation preserves zoom and projection");
        const auto beforeEdit = viewer->cameraParameters();
        const auto sensitivityBeforeEdit = stepLabel->text();
        QLineEdit editor(&window);
        editor.setText("123"); editor.show(); editor.setFocus();
        press(&editor, Qt::Key_Left);
        press(&editor, Qt::Key_Up, Qt::ControlModifier);
        require(sameVector(beforeEdit.position, viewer->cameraParameters().position)
                && stepLabel->text() == sensitivityBeforeEdit,
                "editing text does not rotate camera or alter sensitivity");
        viewer->setFocus();

        const auto cloudPath = temp.filePath(QString::fromUtf8("测试 cloud.PLY"));
        QFile file(cloudPath);
        require(file.open(QIODevice::WriteOnly), "write PLY fixture");
        file.write("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n10 20 30\n11 20 30\n10 21 30\n");
        file.close();
        panel->setOriginLock(true);
        require(drop(viewer, {QUrl::fromLocalFile(cloudPath)}), "drop PLY over VTK viewport");
        auto* document = viewer->pointCloudDocument();
        require(document->pointCount() == 3 && document->sourcePath() == cloudPath
                && !panel->originLocked() && viewer->hasFocus(), "PLY drop loads and focuses viewport");
        const auto fitted = viewer->cameraParameters();
        require(std::abs(fitted.target[0] - 10.5) < 1e-8
                && std::abs(fitted.target[1] - 20.5) < 1e-8
                && std::abs(fitted.target[2] - 30) < 1e-8,
                "off-origin PLY fits its own bounds instead of looking at origin");
        require(!window.findChild<CadReconstructionTask*>()->isRunning(), "file drop does not fit shapes");
        const auto revision = document->revision();
        require(!drop(&window, {QUrl::fromLocalFile(temp.path())}), "reject folder drop");
        require(!drop(&window, {QUrl("https://example.com/cloud.ply")}), "reject remote URL");
        require(!drop(&window, {QUrl::fromLocalFile(temp.filePath("missing.ply"))}), "reject missing file");
        require(!drop(&window, {QUrl::fromLocalFile(cloudPath), QUrl::fromLocalFile(cloudPath)}),
                "reject multi-file drop without replacing scene");
        require(document->revision() == revision, "rejected drops leave cloud intact");

        auto* modelView = window.findChild<ModelViewController*>();
        viewer->setBodyShape(BRepPrimAPI_MakeBox(2, 3, 4).Shape());
        const auto stepPath = temp.filePath(QString::fromUtf8("测试 model.STP"));
        QString error;
        require(modelView->exportStepTo(stepPath, error), "write STEP fixture");
        viewer->clearBody();
        require(drop(&editor, {QUrl::fromLocalFile(stepPath)}), "STEP drop over text field routes to loader");
        require(viewer->hasBody() && !modelView->currentBody().IsNull() && document->pointCount() == 3,
                "STEP import uses existing model loader and preserves cloud");
        require(editor.text() == "123", "drop never inserts file path into text input");
        std::cout << "Navigation and drag/drop tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
