#include "rendering/DatumPlaneActor.h"

#include <vtkActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

#include <array>
#include <cstddef>

namespace {

// Half the edge length of every plane, in millimetres. Big enough to read
// around the model, small enough that it does not dominate it.
constexpr double kHalfExtent = 40.0;

constexpr double kSurfaceOpacity = 0.12;
constexpr double kHighlightedSurfaceOpacity = 0.34;
constexpr double kOutlineLineWidth = 1.2;
constexpr double kHighlightedOutlineLineWidth = 2.2;

constexpr int kLabelFontSize = 18;

// Source points of one plane, in the order vtkPlaneSource wants them.
struct PlaneGeometry {
    double origin[3];
    double point1[3];
    double point2[3];
};

// All three arrays below share one order: XY, YZ, XZ - the order the core's
// modeling::DatumPlane declares.
constexpr std::array<PlaneGeometry, 3> kGeometry{{
    {{-kHalfExtent, -kHalfExtent, 0.0},
     {kHalfExtent, -kHalfExtent, 0.0},
     {-kHalfExtent, kHalfExtent, 0.0}},
    {{0.0, -kHalfExtent, -kHalfExtent},
     {0.0, kHalfExtent, -kHalfExtent},
     {0.0, -kHalfExtent, kHalfExtent}},
    {{-kHalfExtent, 0.0, -kHalfExtent},
     {kHalfExtent, 0.0, -kHalfExtent},
     {-kHalfExtent, 0.0, kHalfExtent}},
}};

struct PlaneStyle {
    double color[3];
    double labelCorner[3];
    const char* name;
};

constexpr std::array<PlaneStyle, 3> kStyles{{
    {{0.30, 0.58, 0.95}, {kHalfExtent, -kHalfExtent, 0.0}, "XY"},
    {{0.35, 0.82, 0.45}, {0.0, -kHalfExtent, kHalfExtent}, "YZ"},
    {{0.95, 0.48, 0.32}, {kHalfExtent, 0.0, kHalfExtent}, "XZ"},
}};

constexpr std::array<modeling::DatumPlane, 3> kPlaneOrder{
    modeling::DatumPlane::XY,
    modeling::DatumPlane::YZ,
    modeling::DatumPlane::XZ,
};

int indexOf(modeling::DatumPlane plane) noexcept
{
    for (std::size_t i = 0; i < kPlaneOrder.size(); ++i) {
        if (kPlaneOrder[i] == plane) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace

DatumPlaneActor::DatumPlaneActor()
{
    for (std::size_t i = 0; i < kPlaneOrder.size(); ++i) {
        PlaneVisual& visual = m_planes[i];
        visual.plane = kPlaneOrder[i];
        const PlaneStyle& style = kStyles[i];

        vtkNew<vtkPlaneSource> source;
        source->SetOrigin(kGeometry[i].origin[0], kGeometry[i].origin[1],
                          kGeometry[i].origin[2]);
        source->SetPoint1(kGeometry[i].point1[0], kGeometry[i].point1[1],
                          kGeometry[i].point1[2]);
        source->SetPoint2(kGeometry[i].point2[0], kGeometry[i].point2[1],
                          kGeometry[i].point2[2]);
        source->SetXResolution(1);
        source->SetYResolution(1);

        visual.mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        visual.mapper->SetInputConnection(source->GetOutputPort());

        // The surface and its outline share one mapper: the outline is only the
        // same quad drawn as a wireframe, which is one polygon and therefore
        // shows exactly the four border edges.
        visual.surface = vtkSmartPointer<vtkActor>::New();
        visual.surface->SetMapper(visual.mapper);
        visual.surface->GetProperty()->SetColor(style.color[0], style.color[1],
                                                style.color[2]);
        visual.surface->GetProperty()->SetLighting(false);

        visual.outline = vtkSmartPointer<vtkActor>::New();
        visual.outline->SetMapper(visual.mapper);
        visual.outline->GetProperty()->SetRepresentationToWireframe();
        visual.outline->GetProperty()->SetColor(style.color[0], style.color[1],
                                                style.color[2]);
        visual.outline->GetProperty()->SetLighting(false);
        visual.outline->SetPickable(false);

        visual.label = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        visual.label->SetInput(style.name);
        visual.label->SetPosition(style.labelCorner[0], style.labelCorner[1],
                                  style.labelCorner[2]);
        visual.label->GetTextProperty()->SetFontSize(kLabelFontSize);
        visual.label->GetTextProperty()->SetBold(1);
        visual.label->GetTextProperty()->SetColor(style.color[0], style.color[1],
                                                  style.color[2]);
        visual.label->SetPickable(false);
    }

    setVisible(m_visible);
    setHighlightedPlane(std::nullopt);
}

void DatumPlaneActor::attach(vtkRenderer* renderer) const
{
    if (renderer == nullptr) {
        return;
    }
    for (const PlaneVisual& visual : m_planes) {
        renderer->AddActor(visual.surface);
        renderer->AddActor(visual.outline);
        renderer->AddActor(visual.label);
    }
}

void DatumPlaneActor::setVisible(bool visible)
{
    m_visible = visible;
    for (const PlaneVisual& visual : m_planes) {
        visual.surface->SetVisibility(visible ? 1 : 0);
        visual.outline->SetVisibility(visible ? 1 : 0);
        visual.label->SetVisibility(visible ? 1 : 0);
    }
}

bool DatumPlaneActor::visible() const noexcept
{
    return m_visible;
}

void DatumPlaneActor::setHighlightedPlane(std::optional<modeling::DatumPlane> plane)
{
    m_highlighted = plane;
    for (const PlaneVisual& visual : m_planes) {
        const bool highlighted = plane.has_value() && *plane == visual.plane;
        visual.surface->GetProperty()->SetOpacity(highlighted ? kHighlightedSurfaceOpacity
                                                              : kSurfaceOpacity);
        visual.outline->GetProperty()->SetLineWidth(highlighted ? kHighlightedOutlineLineWidth
                                                                : kOutlineLineWidth);
        visual.outline->GetProperty()->SetOpacity(highlighted ? 1.0 : 0.75);
    }
}

std::optional<modeling::DatumPlane> DatumPlaneActor::highlightedPlane() const noexcept
{
    return m_highlighted;
}

const DatumPlaneActor::PlaneVisual* DatumPlaneActor::visualOf(
    modeling::DatumPlane plane) const noexcept
{
    const int index = indexOf(plane);
    return index < 0 ? nullptr : &m_planes[static_cast<std::size_t>(index)];
}

vtkActor* DatumPlaneActor::surfaceActor(modeling::DatumPlane plane) const noexcept
{
    const PlaneVisual* visual = visualOf(plane);
    return visual == nullptr ? nullptr : visual->surface;
}

std::optional<modeling::DatumPlane> DatumPlaneActor::planeOfActor(
    const vtkActor* actor) const noexcept
{
    if (actor == nullptr) {
        return std::nullopt;
    }
    for (const PlaneVisual& visual : m_planes) {
        if (visual.surface == actor) {
            return visual.plane;
        }
    }
    return std::nullopt;
}