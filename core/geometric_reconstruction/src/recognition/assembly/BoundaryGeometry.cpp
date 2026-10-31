#include "BoundaryGeometry.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>
#include <algorithm>
#include <cmath>
#include <map>

namespace reconstruction::detail::assembly {
namespace {
using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;
using Polygon=CGAL::Polygon_2<Kernel>;
using P=modeling::Point2D;
double distance(P a,P b){return std::hypot(a.x-b.x,a.y-b.y);}
Polygon polygon(const BoundaryLoop& loop) {
    Polygon result;
    for(auto p:loop)result.push_back({p.x,p.y});
    return result;
}
struct Contact {bool touching=false,invalid=false;};
Contact contact(const BoundaryLoop& first,const BoundaryLoop& second) {
    Contact result;
    for(size_t i=0;i<first.size();++i)for(size_t j=0;j<second.size();++j) {
        const auto a=first[i],b=first[(i+1)%first.size()],c=second[j],d=second[(j+1)%second.size()];
        const Kernel::Point_2 p{a.x,a.y},q{b.x,b.y},r{c.x,c.y},s{d.x,d.y};
        if(!CGAL::do_intersect(Kernel::Segment_2(p,q),Kernel::Segment_2(r,s)))continue;
        result.touching=true;
        const auto a0=CGAL::orientation(p,q,r),a1=CGAL::orientation(p,q,s);
        const auto b0=CGAL::orientation(r,s,p),b1=CGAL::orientation(r,s,q);
        if(a0!=CGAL::COLLINEAR && a1!=CGAL::COLLINEAR && a0!=a1 &&
           b0!=CGAL::COLLINEAR && b1!=CGAL::COLLINEAR && b0!=b1)result.invalid=true;
        if(a0==CGAL::COLLINEAR && a1==CGAL::COLLINEAR) {
            const bool horizontal=std::abs(a.x-b.x)>=std::abs(a.y-b.y);
            const double p0=horizontal?a.x:a.y,p1=horizontal?b.x:b.y;
            const double q0=horizontal?c.x:c.y,q1=horizontal?d.x:d.y;
            if(std::min(std::max(p0,p1),std::max(q0,q1))>std::max(std::min(p0,p1),std::min(q0,q1)))result.invalid=true;
        }
        if(result.invalid)return result;
    }
    return result;
}
bool compatibleShape(const BoundaryLoop& original,const BoundaryLoop& candidate) {
    if(!simpleBoundary(candidate))return false;
    const double before=boundaryArea(original),after=boundaryArea(candidate);
    if(before*after<=0)return false;
    const double ratio=std::abs(after/before);
    return ratio>=.5 && ratio<=2;
}
double segmentDistance(P p,P a,P b) {
    const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
    const double t=length2>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length2,0.0,1.0):0;
    return distance(p,{a.x+t*dx,a.y+t*dy});
}
BoundaryLoop simplifyAt(const BoundaryLoop& loop,double tolerance) {
    if(loop.size()<4)return loop;
    size_t pivot=1;
    for(size_t i=2;i<loop.size();++i)
        if(distance(loop[0],loop[i])>distance(loop[0],loop[pivot]))pivot=i;
    auto chain=[&](size_t first,size_t last) {
        BoundaryLoop points;
        for(size_t i=first;i<=last;++i)points.push_back(loop[i%loop.size()]);
        std::vector<bool> keep(points.size());keep.front()=keep.back()=true;
        std::vector<std::pair<size_t,size_t>> pending{{0,points.size()-1}};
        while(!pending.empty()) {
            const auto range=pending.back();pending.pop_back();
            double maximum=tolerance;size_t selected=0;
            for(size_t i=range.first+1;i<range.second;++i) {
                const double d=segmentDistance(points[i],points[range.first],points[range.second]);
                if(d>maximum){maximum=d;selected=i;}
            }
            if(selected) {
                keep[selected]=true;
                pending.emplace_back(range.first,selected);pending.emplace_back(selected,range.second);
            }
        }
        BoundaryLoop result;
        for(size_t i=0;i+1<points.size();++i)if(keep[i])result.push_back(points[i]);
        return result;
    };
    auto result=chain(0,pivot);const auto rest=chain(pivot,loop.size());
    result.insert(result.end(),rest.begin(),rest.end());
    return result;
}
}

double boundaryArea(const BoundaryLoop& loop) {
    if(loop.size()<3)return 0;
    const auto origin=loop.front();double result=0;
    for(size_t i=0;i<loop.size();++i) {
        const auto a=loop[i],b=loop[(i+1)%loop.size()];
        result+=(a.x-origin.x)*(b.y-origin.y)-(b.x-origin.x)*(a.y-origin.y);
    }
    return result*.5;
}
bool simpleBoundary(const BoundaryLoop& loop) {
    if(loop.size()<3)return false;
    for(auto p:loop)if(!std::isfinite(p.x)||!std::isfinite(p.y))return false;
    return polygon(loop).is_simple() && boundaryArea(loop)!=0;
}
std::vector<BoundaryLoop> splitBoundaryCycles(const BoundaryLoop& walk) {
    std::vector<BoundaryLoop> cycles;
    if(walk.empty())return cycles;
    std::map<std::pair<double,double>,size_t> indices;
    BoundaryLoop path;
    for(size_t i=0;i<=walk.size();++i) {
        const auto p=walk[i%walk.size()];const auto key=std::make_pair(p.x,p.y);
        const auto found=indices.find(key);
        if(found==indices.end()){indices[key]=path.size();path.push_back(p);continue;}
        const auto first=found->second;
        BoundaryLoop cycle(path.begin()+first,path.end());
        if(simpleBoundary(cycle))cycles.push_back(std::move(cycle));
        for(size_t j=first+1;j<path.size();++j)indices.erase({path[j].x,path[j].y});
        path.resize(first+1);
    }
    return cycles;
}
BoundaryLoop simplifyBoundary(const BoundaryLoop& loop,double tolerance,
    BoundaryRecoveryDiagnostics& diagnostics) {
    for(int attempt=0;attempt<5;++attempt,tolerance*=.5) {
        auto candidate=simplifyAt(loop,tolerance);
        if(compatibleShape(loop,candidate) &&
           std::abs(boundaryArea(candidate)-boundaryArea(loop))<=std::abs(boundaryArea(loop))*.05)return candidate;
        if(attempt==0) {
            diagnostics.nonSimpleSimplifiedLoops+=!simpleBoundary(candidate);
            ++diagnostics.simplificationFallbacks;
        }
    }
    return loop;
}
BoundaryLoop snapBoundary(const BoundaryLoop& loop,const std::vector<BoundaryLine>& constraints,
    double spacing,double reach,BoundaryRecoveryDiagnostics& diagnostics,size_t& snapped) {
    std::vector<BoundaryLine> lines;
    bool constrained=false;
    for(size_t i=0;i<loop.size();++i) {
        const auto a=loop[i],b=loop[(i+1)%loop.size()];const double length=distance(a,b);
        if(length<=0)continue;
        BoundaryLine selected{(a.y-b.y)/length,(b.x-a.x)/length,0,-int(i)-1,a};
        selected.c=-selected.a*a.x-selected.b*a.y;double best=reach;
        for(const auto& line:constraints) {
            // Even short edges must agree with the proposed intersection. A
            // midpoint near a plane is insufficient evidence for rotating it.
            const double angle=std::abs(line.a*(b.x-a.x)+line.b*(b.y-a.y))/length;
            if(angle>.3)continue;
            const double d=std::max(std::abs(line.a*a.x+line.b*a.y+line.c),
                                    std::abs(line.a*b.x+line.b*b.y+line.c));
            const double score=d+angle*std::min(length,spacing)*.2;
            if(score<best){best=score;selected=line;selected.anchor=a;}
        }
        constrained|=selected.identity>0;
        if(lines.empty()||selected.identity!=lines.back().identity)lines.push_back(selected);
    }
    if(!constrained)return loop;
    if(lines.size()>1 && lines.front().identity==lines.back().identity) {
        // The first vertex may lie in the middle of a straight run. Its
        // predecessor's corner belongs at the start of the final run, not at
        // that arbitrary first sample.
        lines.front().anchor=lines.back().anchor;lines.pop_back();
    }
    if(lines.size()<3){++diagnostics.constraintFallbacks;return loop;}
    BoundaryLoop recovered;size_t changed=0;bool feasible=true;
    for(size_t i=0;i<lines.size();++i) {
        const auto& a=lines[(i+lines.size()-1)%lines.size()];const auto& b=lines[i];
        const double determinant=a.a*b.b-b.a*a.b;
        P p=b.anchor;
        if(std::abs(determinant)>1e-8) {
            const P intersection{(a.b*b.c-b.b*a.c)/determinant,(b.a*a.c-a.a*b.c)/determinant};
            // A different corner elsewhere on the loop must not authorize a
            // distant intersection for this particular boundary vertex.
            if(distance(intersection,b.anchor)>reach*2){feasible=false;break;}
            p=intersection;changed+=a.identity>0||b.identity>0;
        } else if(a.identity>0||b.identity>0) {feasible=false;break;}
        recovered.push_back(p);
    }
    if(!feasible||!compatibleShape(loop,recovered)) {
        diagnostics.nonSimpleRecoveredLoops+=feasible&&!simpleBoundary(recovered);
        ++diagnostics.snappingFallbacks;
        return loop;
    }
    snapped+=changed;
    return recovered;
}
bool sameBoundaryTopology(const std::vector<BoundaryLoop>& original,
                          const std::vector<BoundaryLoop>& candidate) {
    if(original.size()!=candidate.size())return false;
    for(size_t i=0;i<original.size();++i)for(size_t j=0;j<i;++j) {
        const auto before=contact(original[i],original[j]),after=contact(candidate[i],candidate[j]);
        if(after.invalid || before.touching!=after.touching)return false;
        for(auto pair:{std::make_pair(i,j),std::make_pair(j,i)}) {
            const auto oldRing=polygon(original[pair.second]),newRing=polygon(candidate[pair.second]);
            const auto p=original[pair.first].front(),q=candidate[pair.first].front();
            const auto oldSide=oldRing.bounded_side({p.x,p.y}),newSide=newRing.bounded_side({q.x,q.y});
            if(oldSide!=CGAL::ON_BOUNDARY && oldSide!=newSide)return false;
        }
    }
    return true;
}
}
