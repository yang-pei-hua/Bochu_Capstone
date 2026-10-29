#include "AssemblyInternal.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <map>
#include <set>
#include <numeric>
#include <stdexcept>

namespace reconstruction::detail::assembly {
namespace {
using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;
using Vertex=CGAL::Triangulation_vertex_base_with_info_2<size_t,Kernel>;
using DT=CGAL::Delaunay_triangulation_2<Kernel,CGAL::Triangulation_data_structure_2<Vertex>>;
using P=modeling::Point2D;
double distance(P a,P b){return std::hypot(a.x-b.x,a.y-b.y);}
double area(const std::vector<P>& p){double result=0;for(size_t i=0;i<p.size();++i){auto a=p[i],b=p[(i+1)%p.size()];result+=a.x*b.y-a.y*b.x;}return result*.5;}
std::vector<P> simplify(const std::vector<P>& polygon,double tolerance) {
    if(polygon.size()<4)return polygon;size_t pivot=1;for(size_t i=2;i<polygon.size();++i)if(distance(polygon[0],polygon[i])>distance(polygon[0],polygon[pivot]))pivot=i;
    auto chain=[&](size_t first,size_t last){std::vector<P> points;for(size_t i=first;i<=last;++i)points.push_back(polygon[i%polygon.size()]);
        std::vector<bool> keep(points.size());keep.front()=keep.back()=true;std::vector<std::pair<size_t,size_t>> pending{{0,points.size()-1}};
        while(!pending.empty()){const auto range=pending.back();pending.pop_back();const auto a=points[range.first],b=points[range.second];double maximum=tolerance;size_t selected=0;
            for(size_t i=range.first+1;i<range.second;++i){const auto p=points[i];const double d=std::abs((b.x-a.x)*(a.y-p.y)-(a.x-p.x)*(b.y-a.y))/std::max(1e-20,distance(a,b));if(d>maximum){maximum=d;selected=i;}}
            if(selected){keep[selected]=true;pending.emplace_back(range.first,selected);pending.emplace_back(selected,range.second);}}
        std::vector<P> result;for(size_t i=0;i+1<points.size();++i)if(keep[i])result.push_back(points[i]);return result;};
    auto result=chain(0,pivot);const auto rest=chain(pivot,polygon.size());result.insert(result.end(),rest.begin(),rest.end());return result.size()>=3?result:polygon;
}
std::vector<std::vector<P>> alphaLoops(const std::vector<P>& points,double radius,double simplifyTolerance) {
    DT triangulation;std::vector<std::pair<Kernel::Point_2,size_t>> input;for(size_t i=0;i<points.size();++i)input.emplace_back(Kernel::Point_2(points[i].x,points[i].y),i);
    triangulation.insert(input.begin(),input.end());
    struct Edge{size_t count=0,first=0,second=0;};std::map<std::pair<size_t,size_t>,Edge> edges;
    for(auto face=triangulation.finite_faces_begin();face!=triangulation.finite_faces_end();++face) {
        const size_t ids[]{face->vertex(0)->info(),face->vertex(1)->info(),face->vertex(2)->info()};const auto a=points[ids[0]],b=points[ids[1]],c=points[ids[2]];
        const double cross=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);if(std::abs(cross)<1e-20)continue;
        const double r2=std::pow(distance(a,b)*distance(b,c)*distance(c,a),2)/(4*cross*cross);if(r2>radius*radius)continue;
        for(int i=0;i<3;++i){const auto from=ids[i],to=ids[(i+1)%3];auto& edge=edges[std::minmax(from,to)];++edge.count;edge.first=from;edge.second=to;}
    }
    std::multimap<size_t,size_t> next;for(const auto& pair:edges)if(pair.second.count==1)next.emplace(pair.second.first,pair.second.second);
    std::set<std::pair<size_t,size_t>> visited;std::vector<std::vector<P>> loops;
    for(const auto& edge:next){if(visited.count(edge))continue;std::vector<P> loop;const auto first=edge.first;size_t current=first;
        for(size_t guard=0;guard<=next.size();++guard){loop.push_back(points[current]);const auto range=next.equal_range(current);auto selected=range.second;
            for(auto i=range.first;i!=range.second;++i)if(!visited.count(*i)){selected=i;break;}
            if(selected==range.second)break;visited.insert(*selected);current=selected->second;if(current==first)break;}
        if(current==first && loop.size()>=3){loop=simplify(loop,simplifyTolerance);if(std::abs(area(loop))>simplifyTolerance*simplifyTolerance)loops.push_back(std::move(loop));}}
    std::sort(loops.begin(),loops.end(),[](const auto& a,const auto& b){return std::abs(area(a))>std::abs(area(b));});return loops;
}
struct Line{double a=0,b=0,c=0;int identity=0;P anchor{};};
}
void recoverBoundaries(const WorkingData& data,const AssemblyOptions& options,AssemblyReconstructionResult& result,
    const std::function<bool()>& cancelled) {
    std::vector<std::vector<size_t>> neighbors(result.patches.size());
    for(const auto& c:result.connections)if(c.accepted){neighbors[c.first-1].push_back(c.second-1);neighbors[c.second-1].push_back(c.first-1);}
    for(size_t index=0;index<result.patches.size();++index) {
        if(cancelled&&cancelled())throw std::runtime_error("Cancelled");auto& patch=result.patches[index];if(!patch.planar)continue;
        const auto x=patch.xDirection,y=cross(patch.normal,x);std::vector<P> points;
        for(auto i:data.groups[index]){const auto d=subtract(data.points[i].position,patch.origin);points.push_back({dot(d,x),dot(d,y)});}
        auto loops=alphaLoops(points,std::max(data.tolerance*2,patch.spacing*options.alphaFactor),std::max(data.tolerance,patch.spacing*.45));
        std::vector<Line> constraints;
        for(auto j:neighbors[index]){const auto& other=result.patches[j];if(!other.planar)continue;Line line;
            line.a=dot(other.normal,x);line.b=dot(other.normal,y);line.c=dot(other.normal,subtract(patch.origin,other.origin));const double norm=std::hypot(line.a,line.b);
            if(norm<.15)continue;line.a/=norm;line.b/=norm;line.c/=norm;line.identity=int(j)+1;constraints.push_back(line);}
        for(auto& loop:loops) {
            std::vector<Line> lines;const double reach=std::max(data.tolerance*4,patch.spacing*options.connectivityFactor);
            for(size_t i=0;i<loop.size();++i){const auto a=loop[i],b=loop[(i+1)%loop.size()];const double length=distance(a,b);if(length<1e-15)continue;
                Line selected{(a.y-b.y)/length,(b.x-a.x)/length,0,-int(i)-1};selected.c=-selected.a*a.x-selected.b*a.y;double best=reach;
                for(const auto& line:constraints){const double angle=std::abs(line.a*(b.x-a.x)+line.b*(b.y-a.y))/length;
                    if(angle>.42 && length>patch.spacing*2)continue;const double d=std::abs(line.a*(a.x+b.x)*.5+line.b*(a.y+b.y)*.5+line.c);
                    const double score=d+angle*std::min(length,patch.spacing)*.2;if(score<best){best=score;selected=line;}}
                selected.anchor=a;if(lines.empty()||selected.identity!=lines.back().identity)lines.push_back(selected);
            }
            if(lines.size()>1 && lines.front().identity==lines.back().identity)lines.pop_back();
            if(lines.size()<3)continue;std::vector<Vec3> recovered;
            for(size_t i=0;i<lines.size();++i){const auto& a=lines[(i+lines.size()-1)%lines.size()];const auto& b=lines[i];const double determinant=a.a*b.b-b.a*a.b;
                P p=b.anchor;
                if(std::abs(determinant)>1e-8){const P intersection{(a.b*b.c-b.b*a.c)/determinant,(b.a*a.c-a.a*b.c)/determinant};
                    double nearest=INFINITY;for(auto q:loop)nearest=std::min(nearest,distance(intersection,q));
                    if(nearest<reach*2){p=intersection;if(a.identity>0 || b.identity>0)++patch.snappedVertices;}}
                recovered.push_back(add(patch.origin,add(scale(x,p.x),scale(y,p.y))));}
            // Boundary direction is inherited from the alpha triangles, keeping
            // inner loops opposite to the finite outer loop.
            if(recovered.size()>=3)patch.loops.push_back(std::move(recovered));
        }
    }
}
}
