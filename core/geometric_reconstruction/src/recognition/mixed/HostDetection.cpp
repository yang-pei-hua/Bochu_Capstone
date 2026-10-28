#include "detail/MixedInternal.h"

namespace reconstruction::detail::mixed {
bool hostFrame(const PointStore& points,const SurfaceAnalysisResult& analysis,Frame& frame,BoxCandidate& host) {
    const auto tolerance=analysis.options.detection.plane.distanceThreshold;
    std::vector<PlaneEvidence> planes;
    for(const auto& e:analysis.surfaces) if(auto p=std::get_if<PlaneEvidence>(&e)) planes.push_back(*p);
    if(planes.size()<6) return false;
    std::sort(planes.begin(),planes.end(),[](const auto& a,const auto& b){return a.supportPointIds.size()>b.supportPointIds.size();});
    const auto& first=planes.front(); frame.z=canonicalDirection(first.plane.normal);
    const PlaneEvidence* second=nullptr;
    for(std::size_t i=1;i<planes.size();++i) {
        const auto& p=planes[i]; const double alignment=dot(p.plane.normal,frame.z);
        if(std::abs(alignment)<0.995 || p.supportPointIds.size()<first.supportPointIds.size()/5) continue;
        if(!hasSurfaceRelation(analysis,first.id,p.id,SurfaceRelationKind::Parallel)) continue;
        const double c=-p.plane.offset/alignment;
        const double c0=-first.plane.offset/dot(first.plane.normal,frame.z);
        if(std::abs(c-c0)>tolerance*4) {second=&p;break;}
    }
    if(!second) return false;
    frame.c={-first.plane.offset/dot(first.plane.normal,frame.z),-second->plane.offset/dot(second->plane.normal,frame.z)};
    if(frame.c[0]>frame.c[1]) std::swap(frame.c[0],frame.c[1]);
    std::size_t positive=0,negative=0;
    for(const auto& p:points.points()) {
        positive += dot(p.position,frame.z)>frame.c[1]+tolerance*3;
        negative += dot(p.position,frame.z)<frame.c[0]-tolerance*3;
    }
    if(negative>positive) {frame.z=scale(frame.z,-1);frame.c={-frame.c[1],-frame.c[0]};std::swap(positive,negative);}
    if(positive<30 || negative>points.size()/100) return false;
    std::unordered_map<PointId,const PointSample*> byId;
    for(const auto& p:points.points()) byId[p.id]=&p;
    std::vector<const PlaneEvidence*> sides;
    for(const auto& e:planes) {
        if(std::abs(dot(e.plane.normal,frame.z))>0.1) continue;
        std::size_t eligible=0;
        for(auto id:e.supportPointIds) {
            const double t=dot(byId.at(id)->position,frame.z);
            eligible += t>=frame.c[0]-tolerance*2 && t<=frame.c[1]+tolerance*2;
        }
        if(eligible>=20 && eligible>=e.supportPointIds.size()*0.8) sides.push_back(&e);
    }
    if(sides.empty()) return false;
    frame.x=normalized(subtract(sides.front()->plane.normal,scale(frame.z,dot(sides.front()->plane.normal,frame.z))));
    frame.y=normalized(cross(frame.z,frame.x));
    std::vector<PlaneEvidence> selected{first,*second};
    for(int k=0;k<2;++k) {
        const auto axis=k==0?frame.x:frame.y;
        std::vector<double> coordinates;
        for(auto p:sides) if(std::abs(dot(p->plane.normal,axis))>0.995) {
            coordinates.push_back(-p->plane.offset/dot(p->plane.normal,axis));selected.push_back(*p);
        }
        if(coordinates.size()<2) return false;
        const auto mm=std::minmax_element(coordinates.begin(),coordinates.end());
        (k==0?frame.a:frame.b)={*mm.first,*mm.second};
    }
    if(std::min(frame.a[1]-frame.a[0],frame.b[1]-frame.b[0])<5*(frame.c[1]-frame.c[0])) return false;
    host.primitive={ {frame.world(frame.a[0],frame.b[0],frame.c[0]),frame.x,frame.z},
        frame.a[1]-frame.a[0],frame.b[1]-frame.b[0],frame.c[1]-frame.c[0],true};
    host.sourceSurfaces=selected;host.confidence=0.95;
    return true;
}
} // namespace reconstruction::detail::mixed
