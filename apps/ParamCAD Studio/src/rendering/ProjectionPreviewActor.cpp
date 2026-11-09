#include "rendering/ProjectionPreviewActor.h"
#include "rendering/PointCloudActor.h"
#include "rendering/SketchActor.h"

#include <reconstruction/recognition/ProjectionExtrusion.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

#include <algorithm>
#include <array>
#include <limits>

namespace {
using modeling::Point2D;
using Position = std::array<double, 3>;

struct SketchBounds {
    double minX = std::numeric_limits<double>::infinity(), minY = minX;
    double maxX = -minX, maxY = -minX;

    void add(double x, double y)
    {
        minX = std::min(minX,x); minY = std::min(minY,y);
        maxX = std::max(maxX,x); maxY = std::max(maxY,y);
    }
    void add(const Point2D& p) { add(p.x,p.y); }
    void add(const modeling::Line2D& line) { add(line.x1,line.y1); add(line.x2,line.y2); }
    void add(const modeling::Rectangle2D& rect)
    {
        add(rect.x,rect.y); add(rect.x+rect.width,rect.y+rect.height);
    }
    void add(const modeling::Circle2D& circle)
    {
        add(circle.x-circle.radius,circle.y-circle.radius);
        add(circle.x+circle.radius,circle.y+circle.radius);
    }
};

Position at(const sketchapp::PlaneFrame& frame, double x, double y, double depth = 0)
{
    Position point;
    for (int i = 0; i < 3; ++i)
        point[i] = frame.origin[i]+x*frame.xDirection[i]+y*frame.yDirection[i]+depth*frame.normal[i];
    return point;
}

QColor guideColor(modeling::ExtrudeOperation operation)
{
    using Operation = modeling::ExtrudeOperation;
    switch (operation) {
    case Operation::NewBody: return QColor(255, 183, 64);
    case Operation::Join: return QColor(107, 224, 143);
    case Operation::Cut: return QColor(255, 105, 101);
    case Operation::Intersect: return QColor(205, 149, 255);
    }
    return Qt::white;
}

void configureOverlay(vtkActor* actor)
{
    actor->PickableOff();
    actor->UseBoundsOff();
    actor->GetProperty()->LightingOff();
}

void setGeometry(vtkActor* actor, vtkPolyData* data)
{
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(data);
    actor->SetMapper(mapper);
    actor->SetVisibility(true);
}

PlyCloud supportCloud(const PlyCloud& cloud, const std::vector<reconstruction::PointId>& ids)
{
    PlyCloud points;
    constexpr std::size_t limit = 4000;
    const std::size_t stride = std::max<std::size_t>(1,(ids.size()+limit-1)/limit);
    points.positions.reserve(std::min(ids.size(),limit));
    for (std::size_t i = 0; i < ids.size(); i += stride) {
        if (!ids[i]) continue;
        auto index = ids[i]-1;
        if (!cloud.sourceRows.empty()) {
            const auto match = std::lower_bound(cloud.sourceRows.begin(),cloud.sourceRows.end(),index);
            if (match == cloud.sourceRows.end() || *match != index) continue;
            index = match-cloud.sourceRows.begin();
        }
        if (index < cloud.positions.size()) points.positions.push_back(cloud.positions[index]);
    }
    return points;
}
}

class ProjectionPreviewActor::OperationGuide
{
public:
    explicit OperationGuide(vtkRenderer* renderer);
    void setOperation(const reconstruction::ProjectionOperation& operation,
                      const PlyCloud& cloud, double scaleToCloud);
    void clear();

private:
    SketchActor m_start;
    SketchActor m_end;
    PointCloudActor m_support;
    vtkSmartPointer<vtkActor> m_plane;
    vtkSmartPointer<vtkActor> m_arrow;
};

ProjectionPreviewActor::OperationGuide::OperationGuide(vtkRenderer* renderer)
    : m_plane(vtkSmartPointer<vtkActor>::New()), m_arrow(vtkSmartPointer<vtkActor>::New())
{
    m_support.setPointSize(4);
    for (auto* actor : {m_plane.GetPointer(), m_start.actor(), m_end.actor(), m_arrow.GetPointer(), m_support.actor()}) {
        configureOverlay(actor);
        renderer->AddActor(actor);
    }
    m_start.actor()->GetProperty()->SetLineWidth(3);
    m_end.actor()->GetProperty()->SetOpacity(.45);
    m_plane->GetProperty()->SetOpacity(.15);
    m_arrow->GetProperty()->SetLineWidth(3);
    m_plane->VisibilityOff();
    m_arrow->VisibilityOff();
}

ProjectionPreviewActor::ProjectionPreviewActor()
{
    m_body.setColor(QColor(100, 220, 235));
    m_body.setOpacity(.38);
    configureOverlay(m_body.surfaceActor());
    configureOverlay(m_body.edgeActor());
    m_body.surfaceActor()->UseBoundsOn();
}

ProjectionPreviewActor::~ProjectionPreviewActor() = default;

void ProjectionPreviewActor::attach(vtkRenderer* scene, vtkRenderer* guides)
{
    scene->AddActor(m_body.surfaceActor());
    scene->AddActor(m_body.edgeActor());
    m_guideRenderer = guides;
}

void ProjectionPreviewActor::setBody(const TopoDS_Shape& shape, double scaleToCloud)
{
    m_body.setShape(shape);
    m_body.setTransform(0,0,0,0,0,0,scaleToCloud,scaleToCloud,scaleToCloud);
    m_body.setVisible(true);
}

void ProjectionPreviewActor::setBodyVisible(bool visible) { m_body.setVisible(visible); }

void ProjectionPreviewActor::setOperations(const std::vector<reconstruction::ProjectionOperation>& operations,
                                           const PlyCloud& cloud, double scaleToCloud, int selectedStep)
{
    for (auto& guide : m_guides) guide->clear();
    for (std::size_t i = 0; i < operations.size(); ++i) {
        if (selectedStep >= 0 && i != std::size_t(selectedStep)) continue;
        while (m_guides.size() <= i)
            m_guides.push_back(std::make_unique<OperationGuide>(m_guideRenderer));
        m_guides[i]->setOperation(operations[i], cloud, scaleToCloud);
    }
}

void ProjectionPreviewActor::OperationGuide::setOperation(const reconstruction::ProjectionOperation& operation,
                                                          const PlyCloud& cloud, double scaleToCloud)
{
    const auto* plane = std::get_if<modeling::Plane3d>(&operation.sketch.plane);
    if (!plane || operation.sketch.entities.empty()) return;
    const auto frame = sketchapp::planeFrameOf(*plane);
    const double depth = operation.reverse ? -operation.depth : operation.depth;
    const double direction = operation.reverse ? -1 : 1;
    const auto color = guideColor(operation.operation);
    m_start.setSketch(operation.sketch,frame);
    m_end.setSketch(operation.sketch,sketchapp::translatedFrame(frame,frame.normal,depth));
    m_support.setCloud(supportCloud(cloud,operation.supportPointIds));
    for (auto* actor : {m_start.actor(), m_end.actor(), m_plane.GetPointer(), m_arrow.GetPointer()}) {
        actor->SetScale(scaleToCloud);
        actor->GetProperty()->SetColor(color.redF(),color.greenF(),color.blueF());
    }
    m_support.actor()->GetProperty()->SetColor(color.redF(),color.greenF(),color.blueF());

    SketchBounds bounds;
    for (const auto& entity : operation.sketch.entities)
        std::visit([&](const auto& geometry) { bounds.add(geometry); },entity.geometry);
    const double span = std::max(bounds.maxX-bounds.minX,bounds.maxY-bounds.minY);
    const double pad = .08*span;
    vtkNew<vtkPoints> corners;
    corners->SetDataTypeToDouble();
    for (const auto& p : {at(frame,bounds.minX-pad,bounds.minY-pad), at(frame,bounds.maxX+pad,bounds.minY-pad),
                          at(frame,bounds.maxX+pad,bounds.maxY+pad), at(frame,bounds.minX-pad,bounds.maxY+pad)})
        corners->InsertNextPoint(p.data());
    vtkNew<vtkCellArray> faces;
    const vtkIdType indices[] = {0,1,2,3};
    faces->InsertNextCell(4,indices);
    vtkNew<vtkPolyData> planeData;
    planeData->SetPoints(corners);
    planeData->SetPolys(faces);
    setGeometry(m_plane,planeData);

    const double x = (bounds.minX+bounds.maxX)/2, y = (bounds.minY+bounds.maxY)/2;
    const double head = std::min(.23*operation.depth,.15*std::max(span,operation.depth*.25));
    const auto base = at(frame,x,y);
    const auto tip = at(frame,x,y,depth);
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->InsertNextPoint(base.data());
    points->InsertNextPoint(tip.data());
    vtkNew<vtkCellArray> lines;
    const vtkIdType shaft[] = {0,1};
    lines->InsertNextCell(2,shaft);
    for (const auto& point : {at(frame,x-head*.5,y,depth-direction*head),at(frame,x+head*.5,y,depth-direction*head),
                              at(frame,x,y-head*.5,depth-direction*head),at(frame,x,y+head*.5,depth-direction*head)}) {
        const vtkIdType edge[] = {1,points->InsertNextPoint(point.data())};
        lines->InsertNextCell(2,edge);
    }
    vtkNew<vtkPolyData> arrowData;
    arrowData->SetPoints(points);
    arrowData->SetLines(lines);
    setGeometry(m_arrow,arrowData);
}

void ProjectionPreviewActor::OperationGuide::clear()
{
    m_start.clear();
    m_end.clear();
    m_support.clear();
    for (auto* actor : {m_plane.GetPointer(),m_arrow.GetPointer()}) {
        actor->VisibilityOff();
        actor->SetMapper(nullptr);
    }
}

void ProjectionPreviewActor::clear()
{
    m_body.clear();
    for (auto& guide : m_guides) guide->clear();
}
