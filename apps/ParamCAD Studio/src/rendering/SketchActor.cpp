#include "rendering/SketchActor.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

// Segment count used to approximate a circle. The sketch is a display-only
// preview, so the flat wire from the core stays the source of truth.
constexpr int kCircleSegments = 48;
constexpr double kPi = 3.14159265358979323846;

void appendSegment(vtkPoints* points, vtkCellArray* lines,
                   const sketchapp::PlaneFrame& frame,
                   double x1, double y1, double x2, double y2)
{
    const auto first = static_cast<vtkIdType>(points->GetNumberOfPoints());
    const double coordinates[2][2] = {{x1, y1}, {x2, y2}};
    for (const auto& pair : coordinates) {
        points->InsertNextPoint(
            frame.origin[0] + frame.xDirection[0] * pair[0] + frame.yDirection[0] * pair[1],
            frame.origin[1] + frame.xDirection[1] * pair[0] + frame.yDirection[1] * pair[1],
            frame.origin[2] + frame.xDirection[2] * pair[0] + frame.yDirection[2] * pair[1]);
    }
    lines->InsertNextCell(2);
    lines->InsertCellPoint(first);
    lines->InsertCellPoint(first + 1);
}

void appendVertex(vtkPoints* points, vtkCellArray* vertices,
                  const sketchapp::PlaneFrame& frame, double x, double y)
{
    const auto index = static_cast<vtkIdType>(points->GetNumberOfPoints());
    points->InsertNextPoint(
        frame.origin[0] + frame.xDirection[0] * x + frame.yDirection[0] * y,
        frame.origin[1] + frame.xDirection[1] * x + frame.yDirection[1] * y,
        frame.origin[2] + frame.xDirection[2] * x + frame.yDirection[2] * y);
    vertices->InsertNextCell(1);
    vertices->InsertCellPoint(index);
}

// Appends the drawing cells of one entity. The entities are also kept verbatim so
// a click can be resolved against them in the sketch's own coordinates.
void appendEntity(const modeling::SketchGeometry& geometry,
                  const sketchapp::PlaneFrame& frame,
                  vtkPoints* points,
                  vtkCellArray* lines,
                  vtkCellArray* vertices)
{
    if (const auto* point = std::get_if<modeling::Point2D>(&geometry)) {
        appendVertex(points, vertices, frame, point->x, point->y);
        return;
    }
    if (const auto* line = std::get_if<modeling::Line2D>(&geometry)) {
        appendSegment(points, lines, frame, line->x1, line->y1, line->x2, line->y2);
        return;
    }
    if (const auto* rectangle = std::get_if<modeling::Rectangle2D>(&geometry)) {
        const double x0 = rectangle->x;
        const double y0 = rectangle->y;
        const double x1 = rectangle->x + rectangle->width;
        const double y1 = rectangle->y + rectangle->height;
        appendSegment(points, lines, frame, x0, y0, x1, y0);
        appendSegment(points, lines, frame, x1, y0, x1, y1);
        appendSegment(points, lines, frame, x1, y1, x0, y1);
        appendSegment(points, lines, frame, x0, y1, x0, y0);
        return;
    }

    const auto& circle = std::get<modeling::Circle2D>(geometry);
    double previousX = circle.x + circle.radius;
    double previousY = circle.y;
    for (int step = 1; step <= kCircleSegments; ++step) {
        const double angle = 2.0 * kPi * static_cast<double>(step) / kCircleSegments;
        const double nextX = circle.x + circle.radius * std::cos(angle);
        const double nextY = circle.y + circle.radius * std::sin(angle);
        appendSegment(points, lines, frame, previousX, previousY, nextX, nextY);
        previousX = nextX;
        previousY = nextY;
    }
}

double distanceToSegment(const modeling::Point2D& p,
                         const modeling::Point2D& a, const modeling::Point2D& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double lengthSquared = dx * dx + dy * dy;
    double t = 0.0;
    if (lengthSquared > 0.0) {
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared;
        t = std::max(0.0, std::min(1.0, t));
    }
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// Distance from a sketch point to an entity's geometry, all in sketch units.
double distanceToGeometry(const modeling::SketchGeometry& geometry,
                          const modeling::Point2D& p)
{
    if (const auto* point = std::get_if<modeling::Point2D>(&geometry)) {
        return std::hypot(p.x - point->x, p.y - point->y);
    }
    if (const auto* line = std::get_if<modeling::Line2D>(&geometry)) {
        return distanceToSegment(p, {line->x1, line->y1}, {line->x2, line->y2});
    }
    if (const auto* rectangle = std::get_if<modeling::Rectangle2D>(&geometry)) {
        const modeling::Point2D corners[4] = {
            {rectangle->x, rectangle->y},
            {rectangle->x + rectangle->width, rectangle->y},
            {rectangle->x + rectangle->width, rectangle->y + rectangle->height},
            {rectangle->x, rectangle->y + rectangle->height}};
        double best = distanceToSegment(p, corners[0], corners[1]);
        for (int edge = 1; edge < 4; ++edge) {
            best = std::min(best, distanceToSegment(p, corners[edge], corners[(edge + 1) % 4]));
        }
        return best;
    }

    const auto& circle = std::get<modeling::Circle2D>(geometry);
    return std::abs(std::hypot(p.x - circle.x, p.y - circle.y) - circle.radius);
}

// Vertices of one entity that a drawing tool can anchor on. A rectangle
// contributes its four corners and a circle its centre; nothing contributes the
// midpoints of its edges, so snapping only ever lands on a real vertex.
void appendSnapVertices(const modeling::SketchGeometry& geometry,
                        std::vector<modeling::Point2D>& out)
{
    if (const auto* point = std::get_if<modeling::Point2D>(&geometry)) {
        out.push_back(*point);
        return;
    }
    if (const auto* line = std::get_if<modeling::Line2D>(&geometry)) {
        out.push_back(modeling::Point2D{line->x1, line->y1});
        out.push_back(modeling::Point2D{line->x2, line->y2});
        return;
    }
    if (const auto* rectangle = std::get_if<modeling::Rectangle2D>(&geometry)) {
        out.push_back(modeling::Point2D{rectangle->x, rectangle->y});
        out.push_back(
            modeling::Point2D{rectangle->x + rectangle->width, rectangle->y});
        out.push_back(modeling::Point2D{rectangle->x + rectangle->width,
                                        rectangle->y + rectangle->height});
        out.push_back(
            modeling::Point2D{rectangle->x, rectangle->y + rectangle->height});
        return;
    }

    const auto& circle = std::get<modeling::Circle2D>(geometry);
    out.push_back(modeling::Point2D{circle.x, circle.y});
}

}  // namespace

SketchActor::SketchActor()
{
    m_points = vtkSmartPointer<vtkPoints>::New();
    m_lines = vtkSmartPointer<vtkCellArray>::New();
    m_vertices = vtkSmartPointer<vtkCellArray>::New();

    m_polyData = vtkSmartPointer<vtkPolyData>::New();
    m_polyData->SetPoints(m_points);
    m_polyData->SetLines(m_lines);
    m_polyData->SetVerts(m_vertices);

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputData(m_polyData);
    m_mapper->ScalarVisibilityOff();

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetColor(0.44, 0.82, 1.0);
    m_actor->GetProperty()->SetLineWidth(2.0);
    m_actor->GetProperty()->SetPointSize(7.0);
    // A single flat sketch has no meaningful normals to shade.
    m_actor->GetProperty()->SetLighting(0);
    m_actor->SetVisibility(0);

    m_draftPoints = vtkSmartPointer<vtkPoints>::New();
    m_draftLines = vtkSmartPointer<vtkCellArray>::New();
    m_draftVertices = vtkSmartPointer<vtkCellArray>::New();
    m_draftPolyData = vtkSmartPointer<vtkPolyData>::New();
    m_draftPolyData->SetPoints(m_draftPoints);
    m_draftPolyData->SetLines(m_draftLines);
    m_draftPolyData->SetVerts(m_draftVertices);

    m_draftMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_draftMapper->SetInputData(m_draftPolyData);
    m_draftMapper->ScalarVisibilityOff();

    m_draftActor = vtkSmartPointer<vtkActor>::New();
    m_draftActor->SetMapper(m_draftMapper);
    m_draftActor->GetProperty()->SetColor(1.0, 0.85, 0.30);
    m_draftActor->GetProperty()->SetLineWidth(2.0);
    m_draftActor->GetProperty()->SetPointSize(7.0);
    m_draftActor->GetProperty()->SetLighting(0);
    m_draftActor->SetVisibility(0);
}

void SketchActor::setSketch(const modeling::SketchFeatureParams& params,
                            const sketchapp::PlaneFrame& frame)
{
    m_points->Initialize();
    m_lines->Initialize();
    m_vertices->Initialize();
    m_entities = params.entities;

    for (const modeling::SketchEntity& entity : params.entities) {
        appendEntity(entity.geometry, frame, m_points, m_lines, m_vertices);
    }

    m_points->Modified();
    m_lines->Modified();
    m_vertices->Modified();
    m_polyData->Modified();

    m_frame = frame;
    m_hasFrame = true;
    m_hasSketch = m_points->GetNumberOfPoints() > 0;
    m_actor->SetVisibility(m_visible && m_hasSketch ? 1 : 0);
}

void SketchActor::clear()
{
    m_points->Initialize();
    m_lines->Initialize();
    m_vertices->Initialize();
    m_entities.clear();
    m_points->Modified();
    m_lines->Modified();
    m_vertices->Modified();
    m_polyData->Modified();
    m_hasSketch = false;
    m_hasFrame = false;
    m_actor->SetVisibility(0);
    clearDraft();
}

bool SketchActor::snapPointAt(const modeling::Point2D& point, double tolerance,
                              modeling::Point2D& snapped) const noexcept
{
    if (!m_hasSketch) {
        return false;
    }

    bool found = false;
    double nearestDistance = tolerance;
    std::vector<modeling::Point2D> vertices;
    for (const modeling::SketchEntity& entity : m_entities) {
        vertices.clear();
        appendSnapVertices(entity.geometry, vertices);
        for (const modeling::Point2D& vertex : vertices) {
            const double distance = std::hypot(point.x - vertex.x, point.y - vertex.y);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                snapped = vertex;
                found = true;
            }
        }
    }
    return found;
}

bool SketchActor::hasSketch() const noexcept
{
    return m_hasSketch;
}

void SketchActor::setVisible(bool visible)
{
    m_visible = visible;
    m_actor->SetVisibility(m_visible && m_hasSketch ? 1 : 0);
    m_draftActor->SetVisibility(m_visible && m_hasDraft ? 1 : 0);
}

void SketchActor::setDraft(const modeling::SketchGeometry& geometry)
{
    if (!m_hasFrame) {
        clearDraft();
        return;
    }

    resetDraftArrays();
    appendEntity(geometry, m_frame, m_draftPoints, m_draftLines, m_draftVertices);

    m_draftPoints->Modified();
    m_draftLines->Modified();
    m_draftVertices->Modified();
    m_draftPolyData->Modified();

    m_hasDraft = m_draftPoints->GetNumberOfPoints() > 0;
    m_draftActor->SetVisibility(m_visible && m_hasDraft ? 1 : 0);
}

void SketchActor::clearDraft()
{
    if (!m_hasDraft) {
        return;
    }
    resetDraftArrays();
    m_hasDraft = false;
    m_draftActor->SetVisibility(0);
}

modeling::SketchEntityId SketchActor::entityAt(const modeling::Point2D& point,
                                               double tolerance) const noexcept
{
    if (!m_hasSketch || !m_hasFrame) {
        return modeling::kInvalidSketchEntityId;
    }

    modeling::SketchEntityId nearest = modeling::kInvalidSketchEntityId;
    double nearestDistance = tolerance;
    for (const modeling::SketchEntity& entity : m_entities) {
        const double distance = distanceToGeometry(entity.geometry, point);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = entity.id;
        }
    }
    return nearest;
}

vtkActor* SketchActor::actor() const noexcept
{
    return m_actor;
}

vtkActor* SketchActor::draftActor() const noexcept
{
    return m_draftActor;
}

void SketchActor::resetDraftArrays()
{
    m_draftPoints->Initialize();
    m_draftLines->Initialize();
    m_draftVertices->Initialize();
    m_draftPoints->Modified();
    m_draftLines->Modified();
    m_draftVertices->Modified();
    m_draftPolyData->Modified();
}
