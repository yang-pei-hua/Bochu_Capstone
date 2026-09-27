#include "ui/viewport/VTKViewer.h"

#include "io/PlyReader.h"

#include <QByteArray>
#include <QSize>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageResize.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPiOver180 = 3.14159265358979323846 / 180.0;

// How far from a sketch entity a click still counts as hitting it, in screen
// pixels. Sketch lines are hairline-thin, so a few pixels is what makes them
// selectable with an ordinary mouse while staying precise.
constexpr double kSketchPickPixelRadius = 6.0;

// A drawing tool anchors on an existing vertex from this many pixels away. It is
// a little wider than a pick so snapping feels sticky without being grabby.
constexpr double kSketchSnapPixelRadius = 12.0;

// Extent of the 2D sketch view. The datum planes are drawn 80 mm across, so this
// frames one of them with a small margin.
constexpr double kSketchViewHalfExtent = 44.0;

// Distance of the sketch camera from the plane. An orthographic projection
// ignores it for scale, so it only has to sit clear of the geometry.
constexpr double kSketchViewDistance = 1000.0;

void cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

bool normalize(double v[3])
{
    const double length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (length <= 1.0e-12) {
        return false;
    }
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    return true;
}

}  // namespace

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

    // SolidWorks-style navigation: the left button is reserved for picking and
    // placing geometry, so the camera lives on the middle button and the wheel.
    m_interactorStyle = vtkSmartPointer<SketchInteractorStyle>::New();
    m_interactorStyle->leftButtonPressed = [this](int x, int y) {
        onLeftButtonPressed(x, y);
    };
    m_interactorStyle->mouseMoved = [this](int x, int y) { onMouseMoved(x, y); };
    m_interactorStyle->cancelRequested = [this]() { onCancelRequested(); };
    m_renderWindow->GetInteractor()->SetInteractorStyle(m_interactorStyle);

    m_renderer->AddActor(m_bodyActor.surfaceActor());
    m_renderer->AddActor(m_bodyActor.edgeActor());
    m_renderer->AddActor(m_bodyActor.highlightActor());
    m_renderer->AddActor(m_pointCloudActor.actor());
    m_renderer->AddActor(m_sketchActor.actor());
    m_renderer->AddActor(m_sketchActor.draftActor());
    m_datumPlanes.attach(m_renderer);
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

void VTKViewer::setSketch(const modeling::SketchFeatureParams& params,
                          const sketchapp::PlaneFrame& frame)
{
    m_sketchActor.setSketch(params, frame);
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

void VTKViewer::clearSketch()
{
    m_sketchActor.clear();
    renderNow();
}

bool VTKViewer::hasSketch() const noexcept
{
    return m_sketchActor.hasSketch();
}

void VTKViewer::setViewportTool(ViewportTool tool)
{
    m_tool = tool;
    if (tool != ViewportTool::Line && tool != ViewportTool::Rectangle &&
        tool != ViewportTool::Circle) {
        clearSketchDraft();
    }
}

ViewportTool VTKViewer::viewportTool() const noexcept
{
    return m_tool;
}

void VTKViewer::setSketchInteractionEnabled(bool enabled)
{
    m_sketchInteraction = enabled;
    if (!enabled) {
        clearSketchDraft();
    }
}

bool VTKViewer::sketchInteractionEnabled() const noexcept
{
    return m_sketchInteraction;
}

void VTKViewer::setSketchPlane(const sketchapp::PlaneFrame& frame)
{
    m_sketchPlane = frame;
    m_hasSketchPlane = true;
}

void VTKViewer::clearSketchPlane()
{
    m_hasSketchPlane = false;
}

void VTKViewer::alignToSketchPlane(const sketchapp::PlaneFrame& frame)
{
    vtkCamera* camera = m_renderer->GetActiveCamera();

    // The camera sits on the +normal side looking back at the plane origin with
    // the plane's own Y axis up, so the sketch's X/Y axes land on the screen's
    // X/Y axes and the drawing reads flat.
    const double position[3] = {
        frame.origin[0] + frame.normal[0] * kSketchViewDistance,
        frame.origin[1] + frame.normal[1] * kSketchViewDistance,
        frame.origin[2] + frame.normal[2] * kSketchViewDistance};
    camera->SetPosition(position);
    camera->SetFocalPoint(frame.origin[0], frame.origin[1], frame.origin[2]);
    camera->SetViewUp(frame.yDirection[0], frame.yDirection[1], frame.yDirection[2]);
    camera->OrthogonalizeViewUp();
    camera->SetParallelProjection(1);
    camera->SetParallelScale(kSketchViewHalfExtent);
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

bool VTKViewer::hasSketchVertexAt(const modeling::Point2D& point,
                                  double tolerance) const
{
    modeling::Point2D vertex;
    return m_sketchActor.snapPointAt(point, tolerance, vertex);
}

void VTKViewer::highlightFace(int faceId)
{
    m_bodyActor.highlightFace(faceId);
    renderNow();
}

void VTKViewer::clearFaceHighlight()
{
    m_bodyActor.clearHighlight();
    renderNow();
}

void VTKViewer::setDatumPlanesVisible(bool visible)
{
    m_datumPlanes.setVisible(visible);
    renderNow();
}

void VTKViewer::highlightDatumPlane(modeling::DatumPlane plane)
{
    m_datumPlanes.setHighlightedPlane(plane);
    renderNow();
}

void VTKViewer::clearDatumPlaneHighlight()
{
    m_datumPlanes.setHighlightedPlane(std::nullopt);
    renderNow();
}

void VTKViewer::setSketchDraftAnchor(const modeling::Point2D& anchor)
{
    m_draftAnchor = anchor;
}

void VTKViewer::clearSketchDraft()
{
    m_draftAnchor.reset();
    m_sketchActor.clearDraft();
}

// Intersects the display ray of one pixel with the active sketch plane and
// reports the hit in sketch millimetres. This is what makes a click land on the
// face the sketch was created on, whatever its orientation is.
//
// The ray is built from the camera basis rather than through
// vtkViewport::SetDisplayPoint(), whose pixel convention differs between
// render-window and widget coordinates once the display is scaled.
bool VTKViewer::sketchPointAt(int x, int y, modeling::Point2D& point)
{
    if (!m_hasSketchPlane) {
        return false;
    }
    int* const size = m_renderWindow->GetSize();
    if (size == nullptr || size[0] <= 0 || size[1] <= 0) {
        return false;
    }
    const double width = static_cast<double>(size[0]);
    const double height = static_cast<double>(size[1]);

    vtkCamera* camera = m_renderer->GetActiveCamera();
    double forward[3] = {0.0, 0.0, -1.0};
    double upHint[3] = {0.0, 1.0, 0.0};
    camera->GetDirectionOfProjection(forward);
    camera->GetViewUp(upHint);

    // right = forward x up, then up = right x forward, which is the screen basis
    // VTK itself uses; rebuilding it here keeps the mapping explicit.
    double right[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    cross(forward, upHint, right);
    if (!normalize(right)) {
        return false;
    }
    cross(right, forward, up);
    if (!normalize(up)) {
        return false;
    }

    // The event position has its origin at the bottom-left of the render window,
    // which is the sign convention used for the view-plane offsets below.
    const double nx = 2.0 * static_cast<double>(x) / width - 1.0;
    const double ny = 2.0 * static_cast<double>(y) / height - 1.0;
    const double aspect = width / height;

    double origin[3] = {0.0, 0.0, 0.0};
    camera->GetPosition(origin);
    double direction[3] = {0.0, 0.0, -1.0};
    if (camera->GetParallelProjection()) {
        const double scale = camera->GetParallelScale();
        for (int axis = 0; axis < 3; ++axis) {
            origin[axis] += nx * aspect * scale * right[axis] + ny * scale * up[axis];
            direction[axis] = forward[axis];
        }
    } else {
        const double slope = std::tan(camera->GetViewAngle() * kPiOver180 / 2.0);
        for (int axis = 0; axis < 3; ++axis) {
            direction[axis] = forward[axis] + nx * aspect * slope * right[axis] +
                              ny * slope * up[axis];
        }
    }

    // The ray direction is normalized so the plane distance below is a real
    // length; vtkPlane::IntersectWithLine cannot be used here because it treats
    // its parameter as a fraction along the segment and would reject any hit
    // farther than the segment it is given.
    if (!normalize(direction)) {
        return false;
    }
    const double denominator = m_sketchPlane.normal[0] * direction[0] +
                               m_sketchPlane.normal[1] * direction[1] +
                               m_sketchPlane.normal[2] * direction[2];
    if (std::abs(denominator) < 1.0e-9) {
        // The ray runs parallel to the sketch plane, so there is nothing to hit.
        return false;
    }
    const double numerator =
        m_sketchPlane.normal[0] * (m_sketchPlane.origin[0] - origin[0]) +
        m_sketchPlane.normal[1] * (m_sketchPlane.origin[1] - origin[1]) +
        m_sketchPlane.normal[2] * (m_sketchPlane.origin[2] - origin[2]);
    const double distance = numerator / denominator;
    if (distance <= 0.0) {
        return false;
    }
    const double hit[3] = {origin[0] + distance * direction[0],
                           origin[1] + distance * direction[1],
                           origin[2] + distance * direction[2]};

    const double offset[3] = {hit[0] - m_sketchPlane.origin[0],
                              hit[1] - m_sketchPlane.origin[1],
                              hit[2] - m_sketchPlane.origin[2]};
    point.x = offset[0] * m_sketchPlane.xDirection[0] +
              offset[1] * m_sketchPlane.xDirection[1] +
              offset[2] * m_sketchPlane.xDirection[2];
    point.y = offset[0] * m_sketchPlane.yDirection[0] +
              offset[1] * m_sketchPlane.yDirection[1] +
              offset[2] * m_sketchPlane.yDirection[2];
    return true;
}

bool VTKViewer::snapSketchPointAt(int x, int y, modeling::Point2D& point)
{
    modeling::Point2D raw;
    modeling::Point2D neighbour;
    if (!sketchPointAt(x, y, raw) || !sketchPointAt(x + 1, y, neighbour)) {
        return false;
    }
    point = raw;

    // One pixel measured in sketch units keeps the snap radius constant on
    // screen, whatever the zoom or how oblique the plane is.
    const double unitsPerPixel = std::hypot(neighbour.x - raw.x, neighbour.y - raw.y);
    if (unitsPerPixel <= 0.0) {
        return true;
    }
    modeling::Point2D vertex;
    if (m_sketchActor.snapPointAt(raw, unitsPerPixel * kSketchSnapPixelRadius, vertex)) {
        point = vertex;
    }
    return true;
}

bool VTKViewer::pickBodyFaceAt(int x, int y, int& faceId,
                               sketchapp::PlaneFrame& plane, bool& planar)
{
    if (!m_bodyActor.hasShape()) {
        return false;
    }

    vtkNew<vtkCellPicker> picker;
    picker->PickFromListOn();
    picker->AddPickList(m_bodyActor.surfaceActor());
    picker->Pick(static_cast<double>(x), static_cast<double>(y), 0.0, m_renderer);

    const vtkIdType cellId = picker->GetCellId();
    if (cellId < 0) {
        return false;
    }
    const int hitFace = m_bodyActor.faceIdOfCell(cellId);
    const BodyFaceInfo* info = m_bodyActor.faceInfo(hitFace);
    if (info == nullptr) {
        return false;
    }

    faceId = hitFace;
    planar = info->planar;
    if (!info->planar) {
        return true;
    }

    // The body can be transformed for display, and the plane has to follow: the
    // frame is what the user sees, and the sketch must be built exactly there.
    vtkNew<vtkTransform> placement;
    placement->SetMatrix(m_bodyActor.surfaceActor()->GetMatrix());

    const double localOrigin[3] = {info->origin[0], info->origin[1], info->origin[2]};
    const double localNormal[3] = {info->normal[0], info->normal[1], info->normal[2]};
    const double localX[3] = {info->xDirection[0], info->xDirection[1],
                              info->xDirection[2]};
    double worldNormal[3] = {0.0, 0.0, 1.0};
    double worldX[3] = {1.0, 0.0, 0.0};
    placement->TransformPoint(localOrigin, plane.origin);
    placement->TransformNormal(localNormal, worldNormal);
    placement->TransformVector(localX, worldX);

    double normalLength = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        normalLength += worldNormal[axis] * worldNormal[axis];
    }
    normalLength = std::sqrt(normalLength);
    if (normalLength <= 1.0e-12) {
        return false;
    }

    // X is orthogonalized against the world normal and renormalized, then Y is
    // derived so X x Y = normal - the same rule the core uses for Plane3d.
    const double projection = (worldX[0] * worldNormal[0] + worldX[1] * worldNormal[1] +
                               worldX[2] * worldNormal[2]) /
                              (normalLength * normalLength);
    double xLength = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        plane.normal[axis] = worldNormal[axis] / normalLength;
        plane.xDirection[axis] = worldX[axis] - projection * worldNormal[axis];
        xLength += plane.xDirection[axis] * plane.xDirection[axis];
    }
    xLength = std::sqrt(xLength);
    if (xLength <= 1.0e-12) {
        return false;
    }
    for (double& value : plane.xDirection) {
        value /= xLength;
    }
    plane.yDirection[0] = plane.normal[1] * plane.xDirection[2] -
                          plane.normal[2] * plane.xDirection[1];
    plane.yDirection[1] = plane.normal[2] * plane.xDirection[0] -
                          plane.normal[0] * plane.xDirection[2];
    plane.yDirection[2] = plane.normal[0] * plane.xDirection[1] -
                          plane.normal[1] * plane.xDirection[0];
    return true;
}

bool VTKViewer::pickDatumPlaneAt(int x, int y, modeling::DatumPlane& plane)
{
    if (!m_datumPlanes.visible()) {
        return false;
    }

    // Only the translucent surfaces are offered to the picker, so the outlines
    // and the name labels cannot steal a click from the plane behind them.
    vtkNew<vtkCellPicker> picker;
    picker->PickFromListOn();
    for (modeling::DatumPlane candidate : {modeling::DatumPlane::XY, modeling::DatumPlane::YZ,
                                           modeling::DatumPlane::XZ}) {
        picker->AddPickList(m_datumPlanes.surfaceActor(candidate));
    }
    picker->Pick(static_cast<double>(x), static_cast<double>(y), 0.0, m_renderer);

    const std::optional<modeling::DatumPlane> hit = m_datumPlanes.planeOfActor(picker->GetActor());
    if (!hit.has_value()) {
        return false;
    }
    plane = *hit;
    return true;
}

bool VTKViewer::pickSketchEntityAt(int x, int y, modeling::SketchEntityId& entityId)
{
    if (!m_sketchActor.hasSketch()) {
        return false;
    }

    modeling::Point2D click;
    modeling::Point2D neighbour;
    if (!sketchPointAt(x, y, click) || !sketchPointAt(x + 1, y, neighbour)) {
        return false;
    }

    // Sketch entities are zero-width on screen, so a click is accepted from a
    // narrow band around them. Measuring one pixel in sketch units keeps that
    // band constant on screen no matter how far away or how oblique the plane is,
    // and it avoids a geometric picker that a hairline never satisfies.
    const double unitsPerPixel = std::hypot(neighbour.x - click.x, neighbour.y - click.y);
    if (unitsPerPixel <= 0.0) {
        return false;
    }

    const modeling::SketchEntityId hit =
        m_sketchActor.entityAt(click, unitsPerPixel * kSketchPickPixelRadius);
    if (hit == modeling::kInvalidSketchEntityId) {
        return false;
    }
    entityId = hit;
    return true;
}

void VTKViewer::onLeftButtonPressed(int x, int y)
{
    if (m_sketchInteraction) {
        if (m_tool == ViewportTool::Select) {
            modeling::SketchEntityId entityId = modeling::kInvalidSketchEntityId;
            if (pickSketchEntityAt(x, y, entityId)) {
                emit sketchEntityPicked(entityId);
            } else {
                emit sketchSelectionCleared();
            }
            return;
        }

        // Every drawing tool anchors on existing vertices: the click is snapped
        // onto the nearest one, and only a click that misses them all reaches the
        // window as a free point.
        modeling::Point2D point;
        if (snapSketchPointAt(x, y, point)) {
            emit sketchPointRequested(point);
        }
        return;
    }

    int faceId = -1;
    bool planar = false;
    sketchapp::PlaneFrame plane;
    if (pickBodyFaceAt(x, y, faceId, plane, planar)) {
        emit facePicked(faceId, planar, plane);
        return;
    }

    // The modeled body wins over the datum planes, which run through it: only a
    // click that misses the body can mean "sketch on this reference plane".
    modeling::DatumPlane datumPlane = modeling::DatumPlane::XY;
    if (pickDatumPlaneAt(x, y, datumPlane)) {
        emit datumPlanePicked(datumPlane);
        return;
    }

    emit emptyPicked();
}

void VTKViewer::onMouseMoved(int x, int y)
{
    if (!m_sketchInteraction || !m_draftAnchor.has_value()) {
        return;
    }
    modeling::Point2D cursor;
    if (!snapSketchPointAt(x, y, cursor)) {
        return;
    }

    const modeling::Point2D anchor = *m_draftAnchor;
    switch (m_tool) {
    case ViewportTool::Line:
        m_sketchActor.setDraft(
            modeling::Line2D{anchor.x, anchor.y, cursor.x, cursor.y});
        break;
    case ViewportTool::Rectangle: {
        const double x0 = std::min(anchor.x, cursor.x);
        const double y0 = std::min(anchor.y, cursor.y);
        m_sketchActor.setDraft(modeling::Rectangle2D{
            x0, y0, std::abs(cursor.x - anchor.x), std::abs(cursor.y - anchor.y)});
        break;
    }
    case ViewportTool::Circle: {
        const double radius = std::hypot(cursor.x - anchor.x, cursor.y - anchor.y);
        m_sketchActor.setDraft(modeling::Circle2D{anchor.x, anchor.y, radius});
        break;
    }
    default:
        return;
    }
    renderNow();
}

void VTKViewer::onCancelRequested()
{
    clearSketchDraft();
    renderNow();
    emit sketchCancelled();
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

QSize VTKViewer::captureSourceSize() const
{
    int* const size = m_renderWindow->GetSize();
    if (size == nullptr || size[0] <= 0 || size[1] <= 0) {
        return QSize();
    }
    return QSize(size[0], size[1]);
}

bool VTKViewer::loadPointCloud(const QString& plyPath, QString& error)
{
    PlyCloud cloud;
    if (!PlyReader::read(plyPath, cloud, error)) {
        return false;
    }
    if (!m_pointCloudActor.setCloud(cloud)) {
        error = QStringLiteral("point cloud is empty: %1").arg(plyPath);
        return false;
    }
    m_renderer->ResetCameraClippingRange();
    renderNow();
    return true;
}

void VTKViewer::clearPointCloud()
{
    m_pointCloudActor.clear();
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

bool VTKViewer::hasPointCloud() const noexcept
{
    return m_pointCloudActor.hasCloud();
}

int VTKViewer::pointCloudPointCount() const noexcept
{
    return m_pointCloudActor.pointCount();
}

void VTKViewer::resetCameraToPointCloud()
{
    if (!m_pointCloudActor.hasCloud()) {
        return;
    }
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_pointCloudActor.bounds(bounds);
    m_renderer->ResetCameraClippingRange();
    m_renderer->ResetCamera(bounds);
    m_renderer->ResetCameraClippingRange();
    renderNow();
    emit cameraChanged(cameraParameters());
}

void VTKViewer::setPointCloudVisible(bool visible)
{
    m_pointCloudActor.setVisible(visible);
    renderNow();
}

void VTKViewer::setPointCloudPointSize(double size)
{
    m_pointCloudActor.setPointSize(size);
    renderNow();
}

void VTKViewer::resetCamera()
{
    // The datum planes are large fixed-size reference geometry, so they are left
    // out of the fit: framing them would shrink the model itself by half.
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (sceneBounds(bounds)) {
        m_renderer->ResetCamera(bounds);
    } else {
        m_renderer->ResetCamera();
    }
    m_renderer->ResetCameraClippingRange();
    renderNow();
    emit cameraChanged(cameraParameters());
}

bool VTKViewer::sceneBounds(double bounds[6])
{
    bool hasBounds = false;
    if (m_bodyActor.hasShape() && m_bodyActor.surfaceActor()->GetVisibility() != 0) {
        m_bodyActor.surfaceActor()->GetBounds(bounds);
        hasBounds = true;
    }
    if (m_pointCloudActor.hasCloud()) {
        double cloud[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        m_pointCloudActor.bounds(cloud);
        if (!hasBounds) {
            std::copy(cloud, cloud + 6, bounds);
            return true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            bounds[axis * 2] = std::min(bounds[axis * 2], cloud[axis * 2]);
            bounds[axis * 2 + 1] = std::max(bounds[axis * 2 + 1], cloud[axis * 2 + 1]);
        }
    }
    return hasBounds;
}

void VTKViewer::setStandardView(CameraController::StandardView view)
{
    CameraController::applyStandardView(m_renderer->GetActiveCamera(), view);
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (sceneBounds(bounds)) {
        m_renderer->ResetCamera(bounds);
    } else {
        m_renderer->ResetCamera();
    }
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
