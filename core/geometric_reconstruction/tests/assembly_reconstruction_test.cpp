#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <reconstruction/modeling/ModelingAdapter.h>
#include <BRepGProp.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <GProp_GProps.hxx>
#include <algorithm>
#include <random>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace {
using modeling::Vec3;using reconstruction::PointSample;using reconstruction::PointStore;
void require(bool ok,const std::string& error){if(!ok)throw std::runtime_error(error);}
Vec3 rotate(Vec3 p,double angle){return {std::cos(angle)*p.x-std::sin(angle)*p.y,std::sin(angle)*p.x+std::cos(angle)*p.y,p.z};}
void box(std::vector<PointSample>& cloud,Vec3 center,Vec3 dimensions,double angle=0,int missing=-1,double noise=0) {
    std::mt19937 rng(71);std::normal_distribution<double> gaussian(0,noise);const double dimensionsArray[]{dimensions.x,dimensions.y,dimensions.z};
    for(int axis=0;axis<3;++axis)for(int side=0;side<2;++side){if(axis*2+side==missing)continue;
        for(int i=0;i<16;++i)for(int j=0;j<16;++j){double values[3]{},normal[3]{};values[axis]=(side?.5:-.5)*dimensionsArray[axis];normal[axis]=side?1:-1;
            values[(axis+1)%3]=((i+.5)/16-.5)*dimensionsArray[(axis+1)%3];values[(axis+2)%3]=((j+.5)/16-.5)*dimensionsArray[(axis+2)%3];
            auto p=rotate({values[0],values[1],values[2]},angle);p={p.x+center.x+gaussian(rng),p.y+center.y+gaussian(rng),p.z+center.z+gaussian(rng)};
            PointSample sample;sample.id=10+cloud.size()*7;sample.position=p;sample.normal=rotate({normal[0],normal[1],normal[2]},angle);cloud.push_back(sample);}}
}
reconstruction::AssemblyReconstructionResult run(const std::vector<PointSample>& cloud,double tolerance=.02) {
    reconstruction::AssemblyOptions options;options.distanceTolerance=tolerance;reconstruction::AssemblyReconstructionResult result;std::string error;
    require(reconstruction::reconstructAssembly(PointStore(cloud),options,result,error),error);return result;
}
double volume(const reconstruction::CompoundCandidate& candidate) {
    modeling::ModelingCore core;const auto committed=reconstruction::commitCompoundCandidate(candidate,core);require(committed.success,committed.error);
    require(!core.bodyShape().IsNull()&&BRepCheck_Analyzer(core.bodyShape()).IsValid(),"assembly creates valid CAD");GProp_GProps props;BRepGProp::VolumeProperties(core.bodyShape(),props,1e-9);return props.Mass();
}
size_t closed(const reconstruction::AssemblyReconstructionResult& result){return std::count_if(result.parts.begin(),result.parts.end(),[](const auto& p){return p.closed;});}
void separatedAndRotated() {
    std::vector<PointSample> points;box(points,{-12,0,0},{6,8,5});box(points,{12,4,2},{3,5,9},.37);
    const auto result=run(points);std::cout<<"separated: patches="<<result.patches.size()<<" parts="<<result.parts.size()<<" closed="<<closed(result)<<'\n';
    for(const auto& p:result.parts)std::cout<<p.id<<": "<<p.status<<" "<<p.reason<<'\n';
    require(result.parts.size()==2 && closed(result)==2,"two separated/rotated parts form two independent closed solids");
    require(result.patches.size()==12 && result.unassignedPointIds.empty(),"all observed faces and points retained");
    require(std::abs(volume(result.candidate)-375)<1e-5,"plane intersections recover dimensions rather than shrinking to sampled cap hulls");
    auto shuffled=points;std::mt19937 random(42);std::shuffle(shuffled.begin(),shuffled.end(),random);for(size_t i=0;i<shuffled.size();++i)shuffled[i].id=100+i*13;
    const auto replay=run(shuffled);require(replay.parts.size()==2&&closed(replay)==2&&std::abs(volume(replay.candidate)-volume(result.candidate))<1e-6,"order and point IDs cannot act as part labels");
    for(auto& point:shuffled)point.position={100+point.position.x*7,-30+point.position.y*7,20+point.position.z*7};
    const auto scaled=run(shuffled,.14);require(closed(scaled)==2&&std::abs(volume(scaled.candidate)-375*343)<.01,"global scale and translation do not encode a particular example");
    auto spatialRotate=[](Vec3 p){const Vec3 q{std::cos(.61)*p.x+std::sin(.61)*p.z,p.y,-std::sin(.61)*p.x+std::cos(.61)*p.z};return Vec3{q.x,std::cos(.29)*q.y-std::sin(.29)*q.z,std::sin(.29)*q.y+std::cos(.29)*q.z};};
    for(auto& point:shuffled){point.position=spatialRotate(point.position);point.normal=spatialRotate(*point.normal);}
    const auto tilted=run(shuffled,.14);require(closed(tilted)==2&&std::abs(volume(tilted.candidate)-375*343)<.01,"closure is invariant to arbitrary three-dimensional orientation");
}
void missingFaceAndPartial() {
    std::vector<PointSample> points;box(points,{0,0,0},{6,8,5},.23,5);auto result=run(points);
    require(closed(result)==0 && !result.parts.empty(),"missing cap remains open rather than inventing a closing face");
    require(std::any_of(result.parts.begin(),result.parts.end(),[](const auto& p){return p.openEdges>0;}),"open boundary edges are reported");
    box(points,{20,0,0},{4,6,8},-.31);result=run(points);require(closed(result)==1 && result.parts.size()==2,"recoverable part is exported independently of an open neighbor");
    require(std::abs(volume(result.candidate)-192)<1e-5 && result.candidate.surfaceCoverage>0 && result.candidate.surfaceCoverage<1,"partial coverage is explicit and excludes open geometry");
}
void finiteConcavityAndHole() {
    std::vector<PointSample> points;
    for(int i=0;i<48;++i)for(int j=0;j<48;++j){const double x=-6+(i+.5)/4,y=-6+(j+.5)/4;if(x*x+y*y<4)continue;
        PointSample p;p.id=points.size()+1;p.position={x,y,0};p.normal=Vec3{0,0,1};points.push_back(p);}
    auto result=run(points);require(result.patches.size()==1 && result.patches[0].loops.size()>=2,"finite planar boundary preserves a sampled inner hole");require(closed(result)==0,"a bounded single face is not a volume");
    points.clear();for(int i=0;i<32;++i)for(int j=0;j<32;++j){const double x=(i+.5)/4,y=(j+.5)/4;if(x>4&&y>4)continue;
        PointSample p;p.id=points.size()+1;p.position={x,y,0};p.normal=Vec3{0,0,1};points.push_back(p);}
    result=run(points);require(result.patches.size()==1 && !result.patches[0].loops.empty(),"concave face has a finite boundary");
    const auto& loop=result.patches[0].loops[0];double area=0;for(size_t i=0;i<loop.size();++i){const auto p=loop[i],q=loop[(i+1)%loop.size()];area+=p.x*q.y-q.x*p.y;}
    require(std::abs(area)*.5<55 && std::abs(area)*.5>35,"concavity is retained rather than replaced by a bounding rectangle");
}
void noiseAndGaps() {
    std::vector<PointSample> points;box(points,{0,0,0},{6,8,5},.51,-1,.003);const auto noisy=run(points,.03);
    require(closed(noisy)==1 && std::abs(volume(noisy.candidate)-240)<5,"small scan noise admits bounded closure");
    points.clear();box(points,{0,0,0},{6,6,6});box(points,{6.3,0,0},{6,6,6});const auto gaps=run(points);
    std::cout<<"gaps: patches="<<gaps.patches.size()<<" parts="<<gaps.parts.size()<<" closed="<<closed(gaps)<<'\n';
    for(const auto& p:gaps.parts)std::cout<<p.id<<": "<<p.status<<" patches="<<p.patchIds.size()<<" "<<p.reason<<'\n';
    require(gaps.parts.size()==2&&closed(gaps)==2,"a resolvable physical gap is not bridged by neighborhood connectivity");
    points.clear();box(points,{0,0,0},{6,8,5},.31);for(auto& p:points){p.normal.reset();p.normalOrientationKnown=false;}
    const auto estimated=run(points,.03);
    require(!estimated.parts.empty(),"normal-free observations still admit spatial separation and finite patch recovery");
    require(closed(estimated)>=1,"a complete convex scan can infer outward orientation without an external model");
}
void invalidAndCancel() {
    std::vector<PointSample> points;box(points,{0,0,0},{6,8,5});reconstruction::AssemblyReconstructionResult result;std::string error;reconstruction::AssemblyOptions options;
    require(!reconstruction::reconstructAssembly(PointStore(points),options,result,error,[]{return true;})&&result.parts.empty(),"cancel leaves no partial publishable result");
    points[1].id=points[0].id;bool rejected=false;
    try{rejected=!reconstruction::reconstructAssembly(PointStore(points),options,result,error);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"duplicate point identity is rejected");
}
void curvedParts() {
    std::vector<PointSample> points;const double pi=std::acos(-1.0);
    auto append=[&](Vec3 p,Vec3 n){PointSample sample;sample.id=points.size()+1;sample.position=p;sample.normal=n;points.push_back(sample);};
    for(int i=0;i<72;++i)for(int j=0;j<16;++j){const double a=(i+.5)*2*pi/72,z=(j+.5)*8/16;append({3*std::cos(a),3*std::sin(a),z},{std::cos(a),std::sin(a),0});}
    for(int cap=0;cap<2;++cap)for(int r=0;r<12;++r)for(int i=0;i<72;++i){const double a=(i+.5)*2*pi/72,radius=(r+.5)*3/12;
        append({radius*std::cos(a),radius*std::sin(a),cap*8.0},{0,0,cap?1.0:-1.0});}
    box(points,{20,0,0},{5,7,9},.28);const auto result=run(points);
    std::cout<<"curved: parts="<<result.parts.size()<<" closed="<<closed(result)<<'\n';for(const auto& p:result.parts)std::cout<<p.id<<" "<<p.status<<" "<<p.reason<<'\n';
    require(closed(result)==2,"separated curved and planar parts close independently");
    require(std::abs(volume(result.candidate)-(72*pi+315))<.1,"cylinder caps and finite angular support are verified from points");
}
void nonorthogonalFaces() {
    const std::vector<Vec3> vertices{{-4,-2,-2},{4,-2,-2},{1,4,-2},{-4,-2,3},{4,-2,3},{1,4,3}};
    const std::vector<std::vector<int>> faces{{0,2,1},{3,4,5},{0,1,4,3},{1,2,5,4},{2,0,3,5}};
    std::vector<PointSample> points;
    for(const auto& face:faces)for(size_t t=1;t+1<face.size();++t){const auto a=vertices[face[0]],b=vertices[face[t]],c=vertices[face[t+1]];
        const Vec3 ab{b.x-a.x,b.y-a.y,b.z-a.z},ac{c.x-a.x,c.y-a.y,c.z-a.z};Vec3 n{ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x};
        const double length=std::hypot(n.x,n.y,n.z);n={n.x/length,n.y/length,n.z/length};
        for(int i=0;i<20;++i)for(int j=0;j<20;++j){const double r=std::sqrt((i+.5)/20),s=(j+.5)/20;PointSample p;p.id=points.size()+1;
            p.position={a.x*(1-r)+b.x*r*(1-s)+c.x*r*s,a.y*(1-r)+b.y*r*(1-s)+c.y*r*s,a.z*(1-r)+b.z*r*(1-s)+c.z*r*s};p.normal=n;points.push_back(p);}}
    const auto result=run(points);std::cout<<"triangular: patches="<<result.patches.size()<<" parts="<<result.parts.size()<<" closed="<<closed(result)<<'\n';
    for(const auto& p:result.parts)std::cout<<p.id<<" "<<p.status<<" "<<p.reason<<'\n';
    for(const auto& p:result.patches){std::cout<<"patch "<<p.id<<" loops="<<p.loops.size()<<" spacing="<<p.spacing<<" snapped="<<p.snappedVertices<<'\n';for(const auto& l:p.loops){for(auto v:l)std::cout<<" ("<<v.x<<","<<v.y<<","<<v.z<<")";std::cout<<'\n';}}
    require(closed(result)==1&&result.patches.size()==5,"nonorthogonal finite faces connect into a triangular prism");
    require(std::abs(volume(result.candidate)-120)<1e-5,"closure is not limited to six orthogonal box planes");
}
}
int main(){try{separatedAndRotated();missingFaceAndPartial();finiteConcavityAndHole();noiseAndGaps();curvedParts();nonorthogonalFaces();invalidAndCancel();std::cout<<"Assembly reconstruction tests passed\n";return 0;}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
