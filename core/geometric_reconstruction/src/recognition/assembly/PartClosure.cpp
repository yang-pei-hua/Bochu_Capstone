#include "AssemblyInternal.h"
#include <Eigen/QR>
#include <Eigen/LU>
#include <BRepClass3d_SolidClassifier.hxx>
#include <modeling/ModelingCore.h>
#include <map>
#include <numeric>
#include <stdexcept>

namespace reconstruction::detail::assembly {
namespace {
struct Vertex {Vec3 position{};std::vector<size_t> faces;};
double signedArea(const std::vector<Vec3>& loop,const FiniteSurfacePatch& face){double area=0;const auto x=face.xDirection,y=cross(face.normal,x);
    for(size_t i=0;i<loop.size();++i){const auto a=subtract(loop[i],face.origin),b=subtract(loop[(i+1)%loop.size()],face.origin);area+=dot(a,x)*dot(b,y)-dot(b,x)*dot(a,y);}return area*.5;}
bool convexBoundaries(const WorkingData& data,AssemblyReconstructionResult& output,AssemblyPart& part,
    std::vector<size_t>& active,double& confidence) {
    if(part.patchIds.size()<4 || part.patchIds.size()>32)return false;
    Vec3 center{};size_t centerCount=0;for(auto id:part.patchIds)for(auto i:data.groups[id-1]){center=add(center,data.points[i].position);++centerCount;}
    if(!centerCount)return false;center=scale(center,1.0/centerCount);const bool inferOrientation=!part.orientationKnown;
    struct Plane {size_t id=0;Vec3 normal{},origin{},x{};std::vector<size_t> patches,points;};std::vector<Plane> planes;
    for(auto id:part.patchIds){const auto& patch=output.patches[id-1];if(!patch.planar)return false;
        const auto normal=inferOrientation&&dot(patch.normal,subtract(patch.origin,center))<0?scale(patch.normal,-1):patch.normal;
        if(inferOrientation)for(auto i:data.groups[id-1]){const auto& p=data.points[i];if(p.normal&&p.normalOrientationKnown&&dot(*p.normal,normal)<.5)return false;}
        for(const auto& loop:patch.loops)if(signedArea(loop,patch)<-data.tolerance*data.tolerance)return false;
        size_t match=planes.size();for(size_t j=0;j<planes.size();++j){const auto& p=planes[j];if(dot(p.normal,normal)<.9998)continue;
            if(std::abs(dot(p.normal,subtract(patch.origin,p.origin)))>data.tolerance*2)continue;bool coherent=true;
            for(auto i:data.groups[id-1])if(std::abs(dot(p.normal,subtract(data.points[i].position,p.origin)))>data.tolerance*4){coherent=false;break;}
            if(coherent){match=j;break;}}
        if(match==planes.size())planes.push_back({id,normal,patch.origin,patch.xDirection,{},{}});
        auto& plane=planes[match];plane.patches.push_back(id);plane.points.insert(plane.points.end(),data.groups[id-1].begin(),data.groups[id-1].end());
    }
    if(planes.size()<4)return false;
    // A complete set of outward support planes must have no recession ray.
    // Missing caps therefore remain open, irrespective of the observed hull.
    auto recession=[&](Vec3 d){for(const auto& p:planes)if(dot(p.normal,d)>1e-8)return false;return true;};
    for(size_t i=0;i<planes.size();++i)for(size_t j=0;j<i;++j){const auto d=cross(planes[i].normal,planes[j].normal);
        if(length(d)>1e-8&&(recession(normalized(d))||recession(scale(normalized(d),-1))))return false;}
    std::vector<double> offsets;
    for(const auto& plane:planes)offsets.push_back(dot(plane.normal,subtract(plane.origin,center)));
    for(auto id:part.patchIds)for(auto i:data.groups[id-1])for(size_t j=0;j<planes.size();++j)
        if(dot(planes[j].normal,subtract(data.points[i].position,center))>offsets[j]+data.tolerance*4)return false;
    std::vector<Vec3> vertices;
    for(size_t a=0;a<planes.size();++a)for(size_t b=0;b<a;++b)for(size_t c=0;c<b;++c){Eigen::Matrix3d matrix;
        const auto na=planes[a].normal,nb=planes[b].normal,nc=planes[c].normal;
        matrix<<na.x,na.y,na.z,nb.x,nb.y,nb.z,nc.x,nc.y,nc.z;
        if(std::abs(matrix.determinant())<1e-7)continue;const Eigen::Vector3d solution=matrix.fullPivLu().solve(Eigen::Vector3d(offsets[a],offsets[b],offsets[c]));
        const Vec3 v{solution[0],solution[1],solution[2]};bool feasible=true;
        for(size_t j=0;j<planes.size();++j)if(dot(planes[j].normal,v)>offsets[j]+data.tolerance*4){feasible=false;break;}
        if(!feasible)continue;const auto p=add(center,v);
        if(std::none_of(vertices.begin(),vertices.end(),[&](Vec3 q){return length(subtract(q,p))<data.tolerance*4;}))vertices.push_back(p);
    }
    if(vertices.size()<4)return false;std::vector<std::vector<Vec3>> loops;confidence=1;
    for(size_t j=0;j<planes.size();++j){const auto& plane=planes[j];const auto y=cross(plane.normal,plane.x);std::vector<modeling::Point2D> projected;
        double spacing=0;for(auto id:plane.patches)spacing=std::max(spacing,output.patches[id-1].spacing);
        const auto& patch=output.patches[plane.id-1];const double reach=std::max(spacing,length(subtract(patch.upper,patch.lower))/std::sqrt(double(plane.points.size())))*6;
        for(auto vertex:vertices){if(std::abs(dot(plane.normal,subtract(vertex,center))-offsets[j])>data.tolerance*4)continue;
            double nearest=INFINITY;for(auto i:plane.points)nearest=std::min(nearest,length(subtract(vertex,data.points[i].position)));
            if(nearest>reach)return false;confidence=std::min(confidence,1/(1+nearest/std::max(reach,1e-12)));
            const auto d=subtract(vertex,plane.origin);projected.push_back({dot(d,plane.x),dot(d,y)});}
        std::sort(projected.begin(),projected.end(),[](auto a,auto b){return std::tie(a.x,a.y)<std::tie(b.x,b.y);});
        auto turn=[](auto a,auto b,auto c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);};
        std::vector<modeling::Point2D> hull;for(auto p:projected){while(hull.size()>=2&&turn(hull[hull.size()-2],hull.back(),p)<=1e-12)hull.pop_back();hull.push_back(p);}
        const size_t lower=hull.size();for(size_t k=projected.size();k-->0;){const auto p=projected[k];while(hull.size()>lower&&turn(hull[hull.size()-2],hull.back(),p)<=1e-12)hull.pop_back();hull.push_back(p);}
        if(!hull.empty())hull.pop_back();if(hull.size()<3)return false;std::vector<Vec3> loop;
        for(auto p:hull)loop.push_back(add(plane.origin,add(scale(plane.x,p.x),scale(y,p.y))));loops.push_back(std::move(loop));
    }
    active.clear();for(size_t j=0;j<planes.size();++j){const auto& plane=planes[j];active.push_back(plane.id);
        auto& patch=output.patches[plane.id-1];patch.normal=plane.normal;patch.loops={loops[j]};patch.snappedVertices=loops[j].size();
        for(auto id:plane.patches)if(id!=plane.id){output.patches[id-1].loops.clear();output.patches[id-1].snappedVertices=0;}}
    if(inferOrientation){part.normalOrientationInferred=true;part.orientationKnown=true;confidence*=.8;}
    return true;
}
}
void closeParts(const WorkingData& data,const AssemblyOptions& options,AssemblyReconstructionResult& output,
    const std::function<bool()>& cancelled) {
    size_t closedPoints=0;
    for(auto& part:output.parts) {
        if(cancelled&&cancelled())throw std::runtime_error("Cancelled");
        part.status="open";part.reason="Finite boundaries do not yet form a closed shell";
        if(part.patchIds.size()>128){part.status="ambiguous";part.reason="Surface component exceeds bounded closure budget";continue;}
        std::vector<size_t> faceIds=part.patchIds;double boundaryConfidence=1;
        convexBoundaries(data,output,part,faceIds,boundaryConfidence);
        const double weldTolerance=std::max(data.tolerance*4,1e-7);
        std::vector<Vertex> vertices;std::vector<std::vector<std::vector<size_t>>> faces;bool valid=true;
        auto vertex=[&](Vec3 p,size_t face){for(size_t i=0;i<vertices.size();++i)if(length(subtract(p,vertices[i].position))<=weldTolerance){vertices[i].faces.push_back(face);return i;}
            vertices.push_back({p,{face}});return vertices.size()-1;};
        for(auto id:faceIds) {
            const auto& patch=output.patches[id-1];if(!patch.planar||patch.loops.empty()){valid=false;continue;}
            if(patch.loops.size()>1 && std::any_of(patch.loops.begin()+1,patch.loops.end(),[&](const auto& loop){return signedArea(loop,patch)>0;})) {valid=false;part.reason="Disconnected boundary islands remain ambiguous";continue;}
            std::vector<std::vector<size_t>> loops;
            for(const auto& loop:patch.loops){std::vector<size_t> ids;for(auto p:loop){const auto index=vertex(p,id-1);if(ids.empty()||ids.back()!=index)ids.push_back(index);}
                if(ids.size()>1&&ids.front()==ids.back())ids.pop_back();if(ids.size()<3){valid=false;continue;}loops.push_back(std::move(ids));}
            faces.push_back(std::move(loops));
        }
        if(vertices.size()>4096 || part.patchIds.size()>128){part.status="ambiguous";part.reason="Surface component exceeds bounded closure budget";continue;}
        // Intersections of the incident fitted planes define shared vertices.
        // No original edge or model is used to supply those connections.
        for(auto& v:vertices){std::sort(v.faces.begin(),v.faces.end());v.faces.erase(std::unique(v.faces.begin(),v.faces.end()),v.faces.end());
            if(v.faces.size()<3)continue;Eigen::MatrixXd a(v.faces.size(),3);Eigen::VectorXd b(v.faces.size());
            for(size_t i=0;i<v.faces.size();++i){const auto& face=output.patches[v.faces[i]];a.row(i)<<face.normal.x,face.normal.y,face.normal.z;b[i]=dot(face.normal,subtract(face.origin,v.position));}
            const auto qr=a.colPivHouseholderQr();if(qr.rank()<3)continue;const Eigen::Vector3d shift=qr.solve(b);const Vec3 d{shift[0],shift[1],shift[2]};if(length(d)<=weldTolerance*2)v.position=add(v.position,d);}
        // Split a long boundary edge wherever another face has an observed
        // vertex on it, so T-junctions are explicit rather than silently sewn.
        for(auto& face:faces)for(auto& loop:face){std::vector<size_t> split;
            for(size_t i=0;i<loop.size();++i){const auto first=loop[i],last=loop[(i+1)%loop.size()];const auto a=vertices[first].position,d=subtract(vertices[last].position,a);const double length2=dot(d,d);if(length2<1e-20)continue;
                std::vector<std::pair<double,size_t>> onEdge{{0,first}};
                for(size_t j=0;j<vertices.size();++j){if(j==first||j==last)continue;const auto v=subtract(vertices[j].position,a);const double t=dot(v,d)/length2;
                    if(t>1e-6&&t<1-1e-6&&length(subtract(v,scale(d,t)))<=weldTolerance)onEdge.emplace_back(t,j);}
                std::sort(onEdge.begin(),onEdge.end());for(auto p:onEdge)split.push_back(p.second);}
            loop=std::move(split);}
        struct Edge {size_t uses=0;int direction=0,id=0;};std::map<std::pair<size_t,size_t>,Edge> edges;
        for(const auto& face:faces)for(const auto& loop:face)for(size_t i=0;i<loop.size();++i){const auto a=loop[i],b=loop[(i+1)%loop.size()];if(a==b)continue;
            auto& edge=edges[std::minmax(a,b)];++edge.uses;edge.direction+=a<b?1:-1;}
        for(auto& pair:edges){auto& e=pair.second;
            if(e.uses==1)++part.openEdges;else if(e.uses!=2||e.direction!=0)++part.nonmanifoldEdges;}
        int edgeId=0;for(auto& pair:edges)pair.second.id=++edgeId;
        if(!valid || faces.size()!=faceIds.size()) {part.reason="Unresolved curved/degenerate patches or disconnected boundary islands";continue;}
        if(part.openEdges || part.nonmanifoldEdges){part.reason="Boundary graph has "+std::to_string(part.openEdges)+" unmatched and "+std::to_string(part.nonmanifoldEdges)+" inconsistent edges";continue;}
        if(!part.orientationKnown){part.status="orientation_unknown";part.reason="Material normal directions are not established";continue;}
        if(!options.buildSolids){part.status="closed_boundary";part.reason="Solid construction disabled";continue;}
        if(vertices.empty())continue;Vec3 low=vertices.front().position,high=low;
        for(const auto& v:vertices){low={std::min(low.x,v.position.x),std::min(low.y,v.position.y),std::min(low.z,v.position.z)};high={std::max(high.x,v.position.x),std::max(high.y,v.position.y),std::max(high.z,v.position.z)};}
        if(std::min({high.x-low.x,high.y-low.y,high.z-low.z})<=data.tolerance){part.reason="Unresolved three-dimensional extent";continue;}
        modeling::ProfileBodyParams body;body.pose.origin={(low.x+high.x)*.5,(low.y+high.y)*.5,low.z};body.referenceSizeX=body.sizeX=high.x-low.x;
        body.referenceSizeY=body.sizeY=high.y-low.y;body.depth=body.sizeZ=high.z-low.z;body.boundary.emplace();
        body.boundary->tolerance=std::min(weldTolerance,std::min({body.sizeX,body.sizeY,body.sizeZ})*.05);
        for(size_t i=0;i<faces.size();++i){const auto& patch=output.patches[faceIds[i]-1];modeling::BoundaryFace face;
            face.surface.kind=modeling::BoundarySurface::Kind::Plane;face.surface.pose={subtract(patch.origin,body.pose.origin),patch.xDirection,patch.normal};
            const auto x=patch.xDirection,y=cross(patch.normal,x);
            for(const auto& loop:faces[i]){std::vector<modeling::BoundaryEdgeUse> wire;
                for(size_t j=0;j<loop.size();++j){const auto a=loop[j],b=loop[(j+1)%loop.size()];const auto key=std::minmax(a,b);modeling::BoundaryEdgeUse edge;
                    edge.edgeId=edges.at(key).id;edge.reversed=a>b;edge.curve.degree=1;edge.curve.knots={0,1};edge.curve.multiplicities={2,2};edge.curve.weights={1,1};
                    for(auto index:{key.first,key.second}){const auto d=subtract(vertices[index].position,patch.origin);edge.curve.poles.push_back({dot(d,x),dot(d,y),0});}wire.push_back(std::move(edge));}
                face.wires.push_back(std::move(wire));}body.boundary->faces.push_back(std::move(face));}
        TopoDS_Shape built;std::string failure;
        if(!modeling::buildProfileBody(body,built,failure)){part.status="invalid_shell";part.reason=failure;continue;}
        size_t conflicts=0;BRepClass3d_SolidClassifier classifier(built);
        for(const auto& p:data.points){const auto q=p.position;if(q.x<=low.x+weldTolerance||q.x>=high.x-weldTolerance||q.y<=low.y+weldTolerance||q.y>=high.y-weldTolerance||q.z<=low.z+weldTolerance||q.z>=high.z-weldTolerance)continue;
            double boundary=INFINITY;for(auto id:part.patchIds){const auto& face=output.patches[id-1];boundary=std::min(boundary,std::abs(dot(subtract(q,face.origin),face.normal)));}
            if(boundary<=weldTolerance)continue;classifier.Perform(gp_Pnt(q.x,q.y,q.z),weldTolerance);if(classifier.State()==TopAbs_IN)++conflicts;}
        if(conflicts>std::max<size_t>(2,part.pointIds.size()/200)){part.status="material_conflict";part.reason="Observed interior surfaces contradict a filled solid";continue;}
        part.closed=true;part.status="closed_solid";part.reason.clear();part.lower=low;part.upper=high;
        size_t snapped=0,total=0;for(auto id:part.patchIds){const auto& patch=output.patches[id-1];snapped+=patch.snappedVertices;for(const auto& loop:patch.loops)total+=loop.size();}
        part.confidence=std::min(boundaryConfidence,double(snapped)/std::max<size_t>(1,total));closedPoints+=part.pointIds.size();output.candidate.profileBodies.push_back(std::move(body));
    }
    // Closed analytic curved primitives still require actual caps and angular
    // support. Reuse that general verifier locally after separation.
    if(options.buildSolids) {
        std::vector<size_t> order(output.parts.size());std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](size_t a,size_t b){return output.parts[a].pointIds.size()>output.parts[b].pointIds.size();});
        size_t attempts=0;
        for(auto index:order){auto& part=output.parts[index];if(part.closed||!part.orientationKnown)continue;
            if(cancelled&&cancelled())throw std::runtime_error("Cancelled");
            const auto span=subtract(part.upper,part.lower);if(std::min({span.x,span.y,span.z})<=data.tolerance*5)continue;
            if(part.patchIds.size()==1 && output.patches[part.patchIds.front()-1].planar)continue;
            if(attempts>=options.maximumPrimitiveAttempts)continue;std::vector<PointSample> samples;
            for(auto id:part.patchIds)for(auto i:data.groups[id-1])samples.push_back(data.points[i]);
            if(samples.size()<80 || samples.size()>20000)continue;++attempts;
            CurvedSolidRecognitionOptions settings;settings.distanceTolerance=data.tolerance;settings.minimumSurfaceCoverage=.95;settings.minimumBoundaryCoverage=.95;
            CurvedSolidCandidate candidate;std::string failure;
            if(!reconstructCurvedSolid(PointStore(std::move(samples)),settings,candidate,failure))continue;
            modeling::ModelingCore core;modeling::ModelPatch patch;
            std::visit([&](const auto& p){patch.commands.push_back(modeling::AddFeatureCommand{p,true});},candidate.primitive);
            if(!core.apply(patch).success)continue;
            size_t conflicts=0;BRepClass3d_SolidClassifier classifier(core.bodyShape());
            for(const auto& point:data.points){const auto p=point.position;
                if(p.x<=part.lower.x+data.tolerance*4||p.x>=part.upper.x-data.tolerance*4||p.y<=part.lower.y+data.tolerance*4||p.y>=part.upper.y-data.tolerance*4||p.z<=part.lower.z+data.tolerance*4||p.z>=part.upper.z-data.tolerance*4)continue;
                if(matchesCurvedBoundary(candidate.primitive,point,data.tolerance*4))continue;
                classifier.Perform(gp_Pnt(p.x,p.y,p.z),data.tolerance*4);if(classifier.State()==TopAbs_IN)++conflicts;}
            if(conflicts>std::max<size_t>(2,part.pointIds.size()/200)){part.status="material_conflict";part.reason="Interior observations contradict the closed curved primitive";continue;}
            part.closed=true;part.status="closed_primitive";part.reason="Analytic boundary and cap coverage verified from local observations";
            part.confidence=std::min(candidate.surfaceCoverage,candidate.boundaryCoverage);part.openEdges=0;part.nonmanifoldEdges=0;
            closedPoints+=part.pointIds.size();output.candidate.curvedSolids.push_back(std::move(candidate));
        }
    }
    output.candidate.surfaceCoverage=double(closedPoints)/std::max<size_t>(1,output.inputPoints);output.candidate.confidence=output.candidate.surfaceCoverage;
}
}
