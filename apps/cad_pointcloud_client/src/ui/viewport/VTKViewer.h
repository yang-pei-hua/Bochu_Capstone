#pragma once

#include "core/CameraController.h"
#include "rendering/BodyActor.h"
#include "rendering/DatumPlaneActor.h"
#include "rendering/PointCloudActor.h"
#include "rendering/SketchActor.h"
#include "ui/viewport/SketchInteractorStyle.h"
#include "ui/viewport/ViewportTool.h"

#include <QColor>
#include <QSize>
#include <QString>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkSmartPointer.h>

#include <optional>
#include <string>

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

    // Mirrors the sketch that is currently open in the feature graph, on the
    // plane the core will build it on. The viewer only consumes the sketch and
    // its derived frame; it never interprets either.
    void setSketch(const modeling::SketchFeatureParams& params,
                   const sketchapp::PlaneFrame& frame);
    void clearSketch();
    bool hasSketch() const noexcept;

    // Loads a PLY point cloud from disk. It coexists with the body actor.
    bool loadPointCloud(const QString& plyPath, QString& error);
    void clearPointCloud();
    bool hasPointCloud() const noexcept;
    int pointCloudPointCount() const noexcept;
    void resetCameraToPointCloud();

    // Texture projection. The image is cast onto the body from one direction
    // instead of being unwrapped onto it, so any picture lands on any model
    // without an authoring step. Axis 0 lets the body pick the direction, 1/2/3
    // pin it to X/Y/Z.
    bool loadBodyTexture(const QString& imagePath, QString& error);
    void clearBodyTexture();
    void setBodyTextureEnabled(bool enabled);
    bool hasBodyTexture() const noexcept;
    void setTextureProjection(int axis);
    int textureProjection() const noexcept;

    // --- In-viewport modeling interaction ---
    //
    // The viewer never decides what a click means: the left button becomes
    // either a body-face pick or a point on the active sketch plane, and both
    // are reported as signals. The tool state machine lives in the window.
    void setViewportTool(ViewportTool tool);
    ViewportTool viewportTool() const noexcept;

    // While sketch interaction is on, a left click resolves against the sketch
    // instead of the body.
    void setSketchInteractionEnabled(bool enabled);
    bool sketchInteractionEnabled() const noexcept;

    // Plane that drawing tools project a click onto while a sketch is open.
    void setSketchPlane(const sketchapp::PlaneFrame& frame);
    void clearSketchPlane();

    // Turns the camera onto the sketch plane: it looks straight down the plane
    // normal in an orthographic projection, with the plane's own Y axis pointing
    // up, so a sketch reads as a flat 2D drawing. The window restores the 3D view
    // it saved before the sketch on exit.
    void alignToSketchPlane(const sketchapp::PlaneFrame& frame);

    // Whether the open sketch already owns a vertex at the given sketch position.
    // A drawing tool uses it to tell a click that landed on existing geometry from
    // one that has to become a new point entity.
    bool hasSketchVertexAt(const modeling::Point2D& point, double tolerance) const;

    // Translucent overlay on a picked body face; pass a negative index to clear.
    void highlightFace(int faceId);
    void clearFaceHighlight();

    // The three datum planes are the sketch-plane picking targets of the model
    // mode, so they are shown whenever no sketch is being edited and hidden as
    // soon as drawing starts on them.
    void setDatumPlanesVisible(bool visible);
    void highlightDatumPlane(modeling::DatumPlane plane);
    void clearDatumPlaneHighlight();

    // Rubber band: after the first click of a two-click tool the viewer follows
    // the cursor and previews the geometry the tool would create.
    void setSketchDraftAnchor(const modeling::Point2D& anchor);
    void clearSketchDraft();

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

    // Body face under the cursor, with its exact plane when that face is
    // planar. Only emitted while sketch interaction is off.
    void facePicked(int faceId, bool planar, const sketchapp::PlaneFrame& plane);
    void emptyPicked();

    // One of the three datum planes was clicked in the viewport.
    void datumPlanePicked(modeling::DatumPlane plane);

    // Sketch hit under the cursor (Select tool), and a point on the sketch
    // plane (drawing tools).
    void sketchEntityPicked(modeling::SketchEntityId entityId);
    void sketchSelectionCleared();
    void sketchPointRequested(modeling::Point2D point);
    void sketchCancelled();

    // The Delete key was pressed while a sketch is being edited. The viewer has
    // no idea what is selected - the window owns the selection - so it only asks
    // for the removal and lets the window resolve it.
    void sketchDeleteRequested();

private:
    static void onCameraModified(vtkObject* caller, unsigned long eventId,
                                 void* clientData, void* callData);
    void createOrientationAxes();

    void onLeftButtonPressed(int x, int y);
    void onMouseMoved(int x, int y);
    void onCancelRequested();
    void onKeyPressed(const std::string& keySym);

    bool sketchPointAt(int x, int y, modeling::Point2D& point);
    // Projection of a display position onto the sketch plane, snapped onto an
    // existing vertex when one is within a few pixels. A snapped point is that
    // vertex's exact coordinates, so the caller can recognise it as existing.
    bool snapSketchPointAt(int x, int y, modeling::Point2D& point);
    bool pickBodyFaceAt(int x, int y, int& faceId, sketchapp::PlaneFrame& plane,
                        bool& planar);
    bool pickDatumPlaneAt(int x, int y, modeling::DatumPlane& plane);
    bool pickSketchEntityAt(int x, int y, modeling::SketchEntityId& entityId);

    // Bounds of everything the user actually modeled, which is what the camera
    // is fitted to. False when the scene has no such geometry yet.
    bool sceneBounds(double bounds[6]);

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<SketchInteractorStyle> m_interactorStyle;
    BodyActor m_bodyActor;
    PointCloudActor m_pointCloudActor;
    SketchActor m_sketchActor;
    DatumPlaneActor m_datumPlanes;
    vtkSmartPointer<vtkAxesActor> m_axesActor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_orientationWidget;
    vtkSmartPointer<vtkCallbackCommand> m_cameraCallback;
    unsigned long m_cameraObserverTag = 0;

    ViewportTool m_tool = ViewportTool::Select;
    bool m_sketchInteraction = false;
    bool m_hasSketchPlane = false;
    sketchapp::PlaneFrame m_sketchPlane;
    std::optional<modeling::Point2D> m_draftAnchor;

    // Editing aids, kept so captureImage() can put them back exactly as they
    // were after taking them out of the photographed frame.
    int m_highlightedFace = -1;
    bool m_datumPlanesVisible = false;
};
