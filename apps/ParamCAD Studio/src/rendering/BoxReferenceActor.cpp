#include "rendering/BoxReferenceActor.h"
#include <modeling/PrimitiveEditing.h>
#include <vtkActor.h>
#include <vtkCellPicker.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <algorithm>
#include <cmath>

void BoxReferenceActor::attach(vtkRenderer* renderer)
{
    for (auto& actor : m_actors) {
        actor=vtkSmartPointer<vtkActor>::New(); actor->SetVisibility(false);
        actor->GetProperty()->LightingOff(); renderer->AddActor(actor);
    }
}

void BoxReferenceActor::setBox(const modeling::BoxPrimitiveParams& box)
{
    const auto axes=modeling::boxAxes(box);
    const std::array<double,3> size{box.sizeX,box.sizeY,box.sizeZ};
    const auto point=[&](const std::array<double,3>& offset) {
        std::array<double,3> p{box.pose.origin.x,box.pose.origin.y,box.pose.origin.z};
        for (int i=0;i<3;++i) {
            p[0]+=axes[i].x*offset[i]; p[1]+=axes[i].y*offset[i]; p[2]+=axes[i].z*offset[i];
        }
        return p;
    };
    double colors[3][3]{{1.0,0.35,0.25},{0.25,0.9,0.4},{0.3,0.6,1.0}};
    for (int side=0;side<6;++side) {
        const int normal=side/2, u=(normal+1)%3, v=(normal+2)%3;
        std::array<double,3> offset{}; offset[normal]=side%2 ? size[normal] : 0;
        auto origin=point(offset);
        offset[u]=size[u]; auto first=point(offset);
        offset[u]=0; offset[v]=size[v]; auto second=point(offset);
        vtkNew<vtkPlaneSource> plane;
        plane->SetOrigin(origin.data()); plane->SetPoint1(first.data()); plane->SetPoint2(second.data());
        vtkNew<vtkPolyDataMapper> mapper; mapper->SetInputConnection(plane->GetOutputPort());
        auto* actor=m_actors[side+1].GetPointer(); actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(colors[normal]); actor->GetProperty()->SetOpacity(0.25);
        actor->GetProperty()->EdgeVisibilityOn(); actor->GetProperty()->SetEdgeColor(colors[normal]);
        actor->GetProperty()->SetLineWidth(2);
    }
    const auto center=point({size[0]/2,size[1]/2,size[2]/2});
    vtkNew<vtkSphereSource> sphere; sphere->SetCenter(center.data());
    sphere->SetRadius(std::max(0.01,std::hypot(size[0],size[1],size[2])*0.025));
    vtkNew<vtkPolyDataMapper> mapper; mapper->SetInputConnection(sphere->GetOutputPort());
    m_actors[0]->SetMapper(mapper); m_actors[0]->GetProperty()->SetColor(1.0,0.85,0.15);
    setVisible(true);
}

void BoxReferenceActor::setTransform(vtkMatrix4x4* matrix)
{
    for (auto& actor:m_actors) actor->SetUserMatrix(matrix);
}

void BoxReferenceActor::setVisible(bool visible)
{
    for (auto& actor:m_actors) actor->SetVisibility(visible);
}

int BoxReferenceActor::pick(vtkRenderer* renderer, int x, int y) const
{
    // A center click must not be intercepted by a face in front of the marker.
    for (int pass=0;pass<2;++pass) {
        vtkNew<vtkCellPicker> picker; picker->PickFromListOn();
        for (int i=pass==0 ? 0 : 1;i<(pass==0 ? 1 : 7);++i) picker->AddPickList(m_actors[i]);
        picker->Pick(x,y,0,renderer);
        if (picker->GetCellId()<0) continue;
        for (int i=0;i<7;++i) if (picker->GetActor()==m_actors[i].GetPointer()) return i;
    }
    return -1;
}
