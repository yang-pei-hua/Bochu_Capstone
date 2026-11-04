#include "AssemblyInternal.h"
#include "BoundaryGeometry.h"
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
}
std::vector<std::vector<P>> alphaLoops(const std::vector<P>& points,double radius,double simplifyTolerance,
    BoundaryRecoveryDiagnostics& diagnostics) {
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
    std::set<std::pair<size_t,size_t>> visited;std::vector<std::vector<P>> original,loops;
    for(const auto& edge:next){if(visited.count(edge))continue;std::vector<P> loop;const auto first=edge.first;size_t current=first;
        for(size_t guard=0;guard<=next.size();++guard){loop.push_back(points[current]);const auto range=next.equal_range(current);auto selected=range.second;
            for(auto i=range.first;i!=range.second;++i)if(!visited.count(*i)){selected=i;break;}
            if(selected==range.second)break;visited.insert(*selected);current=selected->second;if(current==first)break;}
        if(current==first && loop.size()>=3) {
            ++diagnostics.tracedLoops;diagnostics.nonSimpleAlphaLoops+=!simpleBoundary(loop);
            auto cycles=splitBoundaryCycles(loop);
            if(cycles.empty())++diagnostics.rejectedAlphaLoops;
            if(cycles.size()>1)diagnostics.splitAlphaLoops+=cycles.size()-1;
            for(const auto& cycle:cycles) {
                auto simplified=simplifyBoundary(cycle,simplifyTolerance,diagnostics);
                if(std::abs(boundaryArea(simplified))>simplifyTolerance*simplifyTolerance) {
                    original.push_back(cycle);loops.push_back(std::move(simplified));
                }
            }
        }}
    if(!sameBoundaryTopology(original,loops)){loops=original;++diagnostics.topologyFallbacks;}
    std::sort(loops.begin(),loops.end(),[](const auto& a,const auto& b){return std::abs(boundaryArea(a))>std::abs(boundaryArea(b));});return loops;
}
void recoverBoundaries(const WorkingData& data,const AssemblyOptions& options,AssemblyReconstructionResult& result,
    const std::function<bool()>& cancelled) {
    std::vector<std::vector<size_t>> neighbors(result.patches.size());
    for(const auto& c:result.connections)if(c.accepted){neighbors[c.first-1].push_back(c.second-1);neighbors[c.second-1].push_back(c.first-1);}
    for(size_t index=0;index<result.patches.size();++index) {
        if(cancelled&&cancelled())throw std::runtime_error("Cancelled");auto& patch=result.patches[index];if(!patch.planar)continue;
        const auto x=patch.xDirection,y=cross(patch.normal,x);std::vector<P> points;
        for(auto i:data.groups[index]){const auto d=subtract(data.points[i].position,patch.origin);points.push_back({dot(d,x),dot(d,y)});}
        auto loops=alphaLoops(points,std::max(data.tolerance*2,patch.spacing*options.alphaFactor),std::max(data.tolerance,patch.spacing*.45),patch.boundaryDiagnostics);
        std::vector<BoundaryLine> constraints;
        for(auto j:neighbors[index]){const auto& other=result.patches[j];if(!other.planar)continue;BoundaryLine line;
            line.a=dot(other.normal,x);line.b=dot(other.normal,y);line.c=dot(other.normal,subtract(patch.origin,other.origin));const double norm=std::hypot(line.a,line.b);
            if(norm<.15)continue;line.a/=norm;line.b/=norm;line.c/=norm;line.identity=int(j)+1;constraints.push_back(line);}
        std::vector<BoundaryLoop> recovered;size_t snapped=0;
        const double reach=std::max(data.tolerance*4,patch.spacing*options.connectivityFactor);
        for(const auto& loop:loops)recovered.push_back(snapBoundary(loop,constraints,patch.spacing,reach,patch.boundaryDiagnostics,snapped));
        if(!sameBoundaryTopology(loops,recovered)) {
            recovered=loops;snapped=0;++patch.boundaryDiagnostics.topologyFallbacks;
        }
        patch.snappedVertices=snapped;
        for(const auto& loop:recovered) {
            std::vector<Vec3> world;
            for(auto p:loop)world.push_back(add(patch.origin,add(scale(x,p.x),scale(y,p.y))));
            patch.loops.push_back(std::move(world));
        }
    }
}
}
