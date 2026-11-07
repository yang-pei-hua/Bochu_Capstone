#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <reconstruction/modeling/ModelingAdapter.h>
#include <BRepGProp.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <GProp_GProps.hxx>
#include <algorithm>
#include <array>
#include <random>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include "recognition/assembly/BoundaryGeometry.h"
#include "recognition/assembly/AssemblyInternal.h"

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
void stagedAssembly() {
    using namespace reconstruction;
    using namespace detail::assembly;
    std::vector<PointSample> points;
    box(points, {-12, 0, 0}, {6, 8, 5});
    const auto split = points.size();
    box(points, {12, 4, 2}, {3, 5, 9}, .37);
    const PointStore source(points);
    AssemblyOptions options;
    options.distanceTolerance = .02;
    options.maximumWorkingPoints = points.size() * 3 / 4;
    auto extraction = extractPatches(source, options, {});
    auto& patches = extraction.patches;
    require(patches.size() == 12 && extraction.work.points.size() == options.maximumWorkingPoints,
        "extraction returns the twelve working patches within budget");
    auto grouping = groupPatches(extraction.work, patches, options);
    require(grouping.parts.size() == 2, "working patches group into two parts");
    for (const auto& part : grouping.parts) {
        require(!part.workingPointIds.empty() && part.pointIds.empty(), "grouping reports working IDs first");
        const auto belongsToFirst = [&](PointId id) { return id < points[split].id; };
        require(std::all_of(part.workingPointIds.begin(), part.workingPointIds.end(),
            [&](PointId id) { return belongsToFirst(id) == belongsToFirst(part.workingPointIds.front()); }),
            "working IDs agree with the synthetic instance labels");
    }

    recoverBoundaries(extraction.work, options, grouping.connections, patches, {});
    const auto observed = patches.front().loops;
    require(!observed.empty(), "boundary recovery can run before full-resolution assignment");
    const auto unassigned = assignSourceObservations(source, extraction.work, options, patches, grouping, {});
    const auto assigned = grouping.parts[0].pointIds.size() + grouping.parts[1].pointIds.size();
    require(assigned + unassigned.size() == source.size() && assigned > extraction.work.points.size(),
        "source assignment accounts for observations beyond the working sample");
    const auto closed = closeParts(extraction.work, options, patches, grouping.parts, source.size(), {});
    require(closed.profileBodies.size() == 2 && std::abs(volume(closed)-375) < 1e-5,
        "closure independently recovers the two known solid volumes");
    const auto& original = patches.front().boundaryBeforeClosure;
    require(original && original->revisionSource == BoundaryRevisionSource::ConvexSupportPlanes &&
        original->loops.size() == observed.size(), "closure retains the observed loops with revision provenance");
    require(original->loops.front().size() == observed.front().size() &&
        reconstruction::math3::length(reconstruction::math3::subtract(original->loops.front().front(), observed.front().front())) == 0,
        "closure leaves the recorded observed vertices intact");
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
void drilledProfiles() {
    using namespace reconstruction::math3;
    constexpr double height = 4;
    constexpr double pi = 3.14159265358979323846;
    const std::vector<Vec3> corners{{-60,-24,0},{60,-24,0},{60,14,0},{46,24,0},{-60,24,0}};
    const std::array<Vec3,2> holes{{{-20,-4,6},{22,5,4.5}}};
    auto fixture = [&](bool incomplete, bool partialArc = false, bool blind = false, int endGap = 0, bool notched = false) {
        std::vector<PointSample> points;
        auto outline = corners;
        if (notched) outline.insert(outline.end()-1,{{10,24,0},{10,12,0},{-10,12,0},{-10,24,0}});
        auto emit = [&](Vec3 p, Vec3 n) {
            auto tilt = [](Vec3 v) {
                return rotate({v.x,std::cos(.44)*v.y-std::sin(.44)*v.z,std::sin(.44)*v.y+std::cos(.44)*v.z}, .37);
            };
            points.push_back({points.size()+1, add(tilt(p), {130,-217,49}), tilt(n), 1, std::nullopt, true});
        };
        for (int cap : {0,1}) for (int u=0; u<80; ++u) for (int v=0; v<36; ++v) {
            const double x = -60 + (u+.5)*120/80, y = -24 + (v+.5)*48/36;
            if ((x-46)*10+(y-24)*14 > 0 || (notched && x>-10 && x<10 && y>12) ||
                std::any_of(holes.begin(),holes.end(),[&](Vec3 h) {
                    return (!blind || cap == (h.x > 0)) && std::hypot(x-h.x,y-h.y) < h.z;
                })) continue;
            emit({x,y,cap*height}, {0,0,cap?1.0:-1.0});
        }
        for (std::size_t k=0; k<outline.size(); ++k) {
            const auto start = outline[k], edge = subtract(outline[(k+1)%outline.size()],start);
            const auto n = normalized(Vec3{edge.y,-edge.x,0});
            for (int u=0; u<36; ++u) for (int v=0; v<12; ++v) {
                if (k == outline.size()-1 && u < endGap) continue;
                const auto p = add(start,scale(edge,(u+.5)/36));
                emit({p.x,p.y,(v+.5)*height/12},n);
            }
        }
        for (const auto& h : holes) for (int u=0; u<(partialArc?32:64); ++u) {
            const double a = 2*pi*(u+.5)/64, x = std::cos(a), y = std::sin(a);
            const auto depthToZ = [&](double d) { return h.x > 0 ? height-d : d; };
            for (int v=0; v<(incomplete && !blind && &h==&holes.back()?5:12); ++v) {
                const double depth = (v+.5)*(blind ? 1.8 : height)/12;
                emit({h.x+h.z*x,h.y+h.z*y,depthToZ(depth)}, {-x,-y,0});
            }
            if (!blind || (incomplete && &h == &holes.back())) continue;
            for (int v=0; v<8; ++v) {
                const double t = std::sqrt((v+.5)/8), radius = h.z*t;
                emit({h.x+radius*x,h.y+radius*y,depthToZ(2.9-1.1*t)},
                     normalized(Vec3{-x,-y,(h.x > 0 ? 1 : -1)*h.z/1.1}));
            }
        }
        return points;
    };
    const auto points = fixture(false);
    const auto drilled = run(points,.006);
    require(closed(drilled)==1 && drilled.candidate.profileBodies.size()==1 &&
            drilled.candidate.profileBodies.front().cavities.size()==2,
            "opposed cap and bore walls reconstruct two independent through holes in a chamfered outline");
    const double expected = height*(120*48 - 14*10/2 - pi*(6*6 + 4.5*4.5));
    require(std::abs(volume(drilled.candidate)-expected) < .03 && drilled.candidate.surfaceCoverage > .999,
            "the reconstructed polygon and both bores agree with the analytic reference volume");
    require(drilled.parts.front().pointIds.size()+drilled.unassignedPointIds.size()==points.size() &&
            drilled.parts.front().workingPointIds.size()+drilled.unassignedWorkingPointIds.size()==drilled.workingPoints,
            "fitted and unassigned observations account for both source and working sets");
    auto assemblyPoints = points;
    for (auto sample : points) {
        sample.id += points.size();
        sample.position.x += 180;
        assemblyPoints.push_back(sample);
    }
    reconstruction::AssemblyOptions assemblyOptions;
    assemblyOptions.distanceTolerance = .006;
    assemblyOptions.maximumWorkingPoints = points.size() * 3 / 2;
    assemblyOptions.maximumLocalProfileAttempts = 0;
    const PointStore assemblySource(std::move(assemblyPoints));
    reconstruction::AssemblyReconstructionResult assembly;
    std::string assemblyError;
    require(reconstruction::reconstructAssembly(assemblySource,assemblyOptions,assembly,assemblyError),assemblyError);
    require(closed(assembly)<2,"the sampled assembly leaves drilled profiles open before local recovery");
    assemblyOptions.maximumLocalProfileAttempts = 4;
    require(reconstruction::reconstructAssembly(assemblySource,assemblyOptions,assembly,assemblyError),assemblyError);
    require(closed(assembly)==2 && assembly.candidate.profileBodies.size()==2 &&
            assembly.candidate.profileBodies[0].cavities.size()==2 && assembly.candidate.profileBodies[1].cavities.size()==2 &&
            std::abs(volume(assembly.candidate)-2*expected)<.06 && assembly.candidate.surfaceCoverage>.99,
            "local full-resolution recovery closes both drilled profiles and preserves the bores");
    const auto incomplete = run(fixture(true),.006);
    require(closed(incomplete)==0, "holes that do not reach both caps cannot be inferred as through cuts");
    const auto partialArc = run(fixture(false,true),.006);
    require(closed(partialArc)==1 && std::abs(volume(partialArc.candidate)-expected) < .03,
            "paired cap openings complete a bore when an inward wall spans the thickness with partial angular sampling");
    const auto endGap = run(fixture(false,false,false,7),.006);
    require(closed(endGap)==1 && std::abs(volume(endGap.candidate)-expected) < .03,
            "the two caps and adjoining wall corroborate the endpoint of an observed side");
    require(closed(run(fixture(false,false,false,12),.006))==0,
            "the corner observations cannot complete a side with a large unsampled span");
    const auto blind = run(fixture(false,false,true),.006);
    const double blindVolume = height*(120*48 - 14*10/2) - pi*(6*6+4.5*4.5)*(1.8+1.1/3);
    require(closed(blind)==1 && blind.candidate.profileBodies.size()==1 &&
            blind.candidate.profileBodies.front().cavities.size()==4 &&
            std::abs(volume(blind.candidate)-blindVolume) < .03 && blind.candidate.surfaceCoverage > .999,
            "inward cylindrical walls and observed conical tips close blind bores from either face");
    require(closed(run(fixture(true,false,true),.006))==0,
            "a second blind bore without a visible tip must not be completed by inference");
    const auto notched = run(fixture(false,false,false,0,true),.006);
    require(closed(notched)==1 && notched.candidate.profileBodies.front().outline.size()==9 &&
            notched.candidate.profileBodies.front().cavities.size()==2 &&
            std::abs(volume(notched.candidate)-(expected-height*20*12)) < .03,
            "finite walls close a rotated reentrant outline and preserve both through holes");
    require(closed(run(fixture(false,false,false,36,true),.006))==0,
            "a missing outer wall cannot be recovered by extending its neighbors");
}
void profileCutouts() {
    using namespace reconstruction::math3;
    constexpr double depth = 2.5, radius = 4;
    const double pi = std::acos(-1.0), halfChord = std::sqrt(12.0);
    const std::vector<Vec3> perimeter{{-22,-13,0},{22,-13,0},{22,13,0},{-22,13,0}};
    const std::vector<Vec3> window{{3,-4,0},{3,4,0},{9,4,0},{9,-4,0}};
    std::vector<PointSample> points;
    const auto emit = [&](Vec3 p, Vec3 n) {
        points.push_back({points.size()+1,p,n,1,std::nullopt,true});
    };
    for (int i=0; i<96; ++i) for (int j=0; j<56; ++j) {
        const double x = -22+(i+.5)*44/96, y = -13+(j+.5)*26/56;
        if ((x>=-10 && std::hypot(x+8,y)<radius) || (x>3 && x<9 && std::abs(y)<4)) continue;
        emit({x,y,0},{0,0,-1});
        emit({x,y,depth},{0,0,1});
    }
    const auto emitWall = [&](Vec3 a, Vec3 b) {
        const auto edge = subtract(b,a);
        const auto n = normalized(Vec3{edge.y,-edge.x,0});
        for (int i=0; i<64; ++i) for (int j=0; j<12; ++j) {
            auto p = add(a,scale(edge,(i+.5)/64));
            p.z = depth*(j+.5)/12;
            emit(p,n);
        }
    };
    for (const auto* loop : {&perimeter,&window}) {
        for (std::size_t i=0; i<loop->size(); ++i) emitWall((*loop)[i],(*loop)[(i+1)%loop->size()]);
    }
    for (int i=0; i<120; ++i) for (int j=0; j<12; ++j) {
        const double angle = ((i+.5)/120-.5)*4*pi/3;
        const double x = std::cos(angle), y = std::sin(angle);
        emit({-8+radius*x,radius*y,depth*(j+.5)/12},{-x,-y,0});
    }
    require(closed(run(points,.004))==0,"an incomplete curved opening needs its closing wall");
    emitWall({-10,-halfChord,0},{-10,halfChord,0});
    const auto result = run(points,.004);
    const double openingArea = radius*radius*(pi-std::acos(.5))+2*halfChord;
    require(closed(result)==1 && result.candidate.profileBodies.front().profileVoids.size()==2 &&
            std::abs(volume(result.candidate)-depth*(44*26-6*8-openingArea))<.05,
            "signed finite walls preserve a clipped circular opening and a separate rectangular opening");
}

void countersunkProfile() {
    constexpr double inner = 6, mouth = 8, height = 8, chamfer = 2;
    const double pi = std::acos(-1.0);
    std::vector<PointSample> points;
    box(points,{0,0,0},{60,36,height});
    points.erase(std::remove_if(points.begin(),points.end(),[mouth](const auto& p) {
        return std::abs(p.normal->z) > .99 && std::hypot(p.position.x,p.position.y) < mouth;
    }),points.end());
    for (int i=0; i<64; ++i) {
        const double angle = 2*pi*(i+.5)/64, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<16; ++j) {
            const double z = (height-2*chamfer)*((j+.5)/16-.5);
            points.push_back({points.size()+1000000,{inner*x,inner*y,z},Vec3{-x,-y,0},1,std::nullopt,true});
        }
    }
    require(closed(run(points,.004))==0,"an unobserved opening transition cannot complete a countersunk bore");
    for (double sign : {-1,1}) for (int i=0; i<64; ++i) {
        const double angle = 2*pi*(i+.5)/64, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<8; ++j) {
            const double t = (j+.5)/8, r = inner+t*(mouth-inner), z = sign*(height/2-chamfer+chamfer*t);
            const auto n = reconstruction::math3::normalized(Vec3{-x,-y,sign});
            points.push_back({points.size()+1000000,{r*x,r*y,z},n,1,std::nullopt,true});
        }
    }
    const auto result = run(points,.004);
    const double expected = 60*36*height-pi*inner*inner*(height-2*chamfer)-
        2*pi*chamfer*(mouth*mouth+mouth*inner+inner*inner)/3;
    require(closed(result)==1 && result.candidate.profileBodies.size()==1 &&
            result.candidate.profileBodies.front().cavities.size()==3 &&
            std::abs(volume(result.candidate)-expected) < .02,
            "observed conical mouths join the through bore without losing the chamfers");
    points.erase(std::remove_if(points.begin(),points.end(),[](const auto& sample) {
        if (sample.normal->z<.6 || sample.normal->z>.8) return false;
        const auto& p = sample.position;
        return std::abs(std::atan2(p.y,p.x))<1.25;
    }),points.end());
    const auto partial = run(points,.004);
    require(closed(partial)==1 && partial.candidate.profileBodies.front().cavities.size()==3 &&
            std::abs(volume(partial.candidate)-expected)<.02,
            "a cap opening and cylindrical wall corroborate a partially observed countersink");
}

void steppedProfile() {
    constexpr double inner = 5, counterbore = 8, mouth = 7, height = 8;
    const double pi = std::acos(-1.0);
    std::vector<PointSample> points;
    box(points,{0,0,0},{60,36,height});
    points.erase(std::remove_if(points.begin(),points.end(),[mouth,counterbore](const auto& p) {
        return std::abs(p.normal->z) > .99 && std::hypot(p.position.x,p.position.y) < (p.position.z > 0 ? mouth : counterbore);
    }),points.end());
    auto emit = [&](Vec3 p, Vec3 n) { points.push_back({points.size()+1000000,p,n,1,std::nullopt,true}); };
    for (int i=0; i<64; ++i) {
        const double angle = 2*pi*(i+.5)/64, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<12; ++j) {
            const double t = (j+.5)/12, r = inner+t*(mouth-inner);
            emit({counterbore*x,counterbore*y,-4+4*t},{-x,-y,0});
            emit({inner*x,inner*y,2*t},{-x,-y,0});
            emit({r*x,r*y,2+2*t},reconstruction::math3::normalized(Vec3{-x,-y,1}));
        }
    }
    require(closed(run(points,.004))==0,"two bore radii need an observed connecting shoulder");
    for (int i=12; i<64; ++i) {
        const double angle = 2*pi*(i+.5)/64, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<8; ++j) {
            const double r = std::sqrt(inner*inner+(counterbore*counterbore-inner*inner)*(j+.5)/8);
            emit({r*x,r*y,0},{0,0,-1});
        }
    }
    const auto platePoints = points.size();
    for (int i=0; i<64; ++i) {
        const double angle = 2*pi*(i+.5)/64, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<8; ++j) emit({6*x,6*y,-5-(j+.5)/2},{x,y,0});
    }
    const auto result = run(points,.004);
    const double expected = 60*36*height-pi*(4*counterbore*counterbore+2*inner*inner+
        2*(mouth*mouth+mouth*inner+inner*inner)/3);
    require(closed(result)==1 && result.candidate.profileBodies.size()==1 &&
            result.candidate.profileBodies.front().cavities.size()==3 &&
            std::abs(volume(result.candidate)-expected)<.02 && result.unassignedPointIds.size()==points.size()-platePoints,
            "a supported counterbore and chamfer close the plate while exterior observations stay unassigned");
}

void annularTube() {
    using namespace reconstruction::math3;
    constexpr double inner = 7, outer = 9, depth = 18;
    const double pi = std::acos(-1.0);
    const auto axis = normalized(Vec3{.3,.4,1});
    const auto x = normalized(cross(axis,{1,0,0})), y = cross(axis,x);
    std::vector<PointSample> points;
    auto emit = [&](Vec3 radial, double radius, double z, Vec3 normal) {
        const auto position = add({171,-219,32},add(scale(radial,radius),scale(axis,z)));
        points.push_back({points.size()+1,position,normal,1,std::nullopt,true});
    };
    for (int i=0; i<80; ++i) {
        const double angle = 2*pi*(i+.5)/80;
        const auto radial = add(scale(x,std::cos(angle)),scale(y,std::sin(angle)));
        for (int j=0; j<20; ++j) {
            const double z = (j+.5)*depth/20;
            if (i>=5 || j>=12) emit(radial,inner,z,scale(radial,-1));
            emit(radial,outer,z,radial);
        }
        for (int j=0; j<6; ++j) {
            const double radius = std::sqrt(inner*inner+(outer*outer-inner*inner)*(j+.5)/6);
            emit(radial,radius,0,scale(axis,-1));
        }
    }
    require(closed(run(points,.004))==0,"a tube with an unobserved annular end remains open");
    for (int i=0; i<80; ++i) {
        const double angle = 2*pi*(i+.5)/80;
        const auto radial = add(scale(x,std::cos(angle)),scale(y,std::sin(angle)));
        for (int j=0; j<6; ++j) {
            const double radius = std::sqrt(inner*inner+(outer*outer-inner*inner)*(j+.5)/6);
            emit(radial,radius,depth,axis);
        }
    }
    const auto tubePoints = points.size();
    for (int i=0; i<32; ++i) for (int j=0; j<16; ++j) {
        const auto radial = add(scale(x,12+(i+.5)/8),scale(y,(j+.5)/8));
        emit(radial,1,depth,scale(axis,-1));
    }
    const auto result = run(points,.004);
    require(closed(result)==1 && result.parts.front().status=="closed_annular_tube" &&
            result.parts.front().pointIds.size()==tubePoints &&
            result.unassignedPointIds.size()==points.size()-tubePoints,
            "signed annular caps and walls isolate the tube from an unrelated coplanar surface");
    require(std::abs(volume(result.candidate)-pi*(outer*outer-inner*inner)*depth) < 1e-4,
            "closed annular tube preserves the bore and both fitted radii");
    for (int i=0; i<32; ++i) {
        const double angle = 2*pi*(i+.5)/32;
        const auto radial = add(scale(x,std::cos(angle)),scale(y,std::sin(angle)));
        emit(radial,(inner+outer)/2,depth/2,axis);
    }
    require(closed(run(points,.004))==0,"observations within the proposed annular material contradict closure");
}

void drilledCircularSlab() {
    constexpr double outer = 18, inner = 8, bore = 1.5, offset = 13, depth = 4;
    const double pi = std::acos(-1.0);
    std::vector<PointSample> points;
    const auto emit = [&](Vec3 p, Vec3 n) {
        points.push_back({points.size()+1,p,n,1,std::nullopt,true});
    };
    for (int i=0; i<82; ++i) for (int j=0; j<82; ++j) {
        const double x = outer*((i+.5)/41-1), y = outer*((j+.5)/41-1);
        const double radius = std::hypot(x,y);
        if (radius >= outer || radius <= inner || std::hypot(x-offset,y) <= bore) continue;
        emit({x,y,0},{0,0,-1});
        emit({x,y,depth},{0,0,1});
    }
    for (int i=0; i<96; ++i) for (int j=0; j<16; ++j) {
        const double angle = 2*pi*(i+.5)/96, x = std::cos(angle), y = std::sin(angle);
        const double z = depth*(j+.5)/16;
        emit({outer*x,outer*y,z},{x,y,0});
        emit({inner*x,inner*y,z},{-x,-y,0});
        emit({offset+bore*x,bore*y,z},{-x,-y,0});
    }
    const auto result = run(points,.004);
    const double expected = pi*depth*(outer*outer-inner*inner-bore*bore);
    require(closed(result)==1 && result.parts.front().status=="closed_analytic_profile" &&
            result.candidate.profileBodies.front().cavities.size()==2 &&
            std::abs(volume(result.candidate)-expected)<.01,
            "signed caps and three independent circular walls preserve both holes in a round slab");

    constexpr double grooveInner = 9, grooveOuter = 11, floor = 2.5;
    points.erase(std::remove_if(points.begin(),points.end(),[=](const auto& p) {
        const auto r = std::hypot(p.position.x,p.position.y);
        return p.position.z == depth && r > grooveInner && r < grooveOuter;
    }),points.end());
    auto nextId = points.back().id+1;
    const auto groovePoint = [&](Vec3 p, Vec3 n) {
        points.push_back({nextId++,p,n,1,std::nullopt,true});
    };
    for (int i=0; i<96; ++i) for (int j=0; j<12; ++j) {
        const double angle = 2*pi*(i+.5)/96, x = std::cos(angle), y = std::sin(angle);
        const double z = floor+(depth-floor)*(j+.5)/12;
        groovePoint({grooveInner*x,grooveInner*y,z},{x,y,0});
        groovePoint({grooveOuter*x,grooveOuter*y,z},{-x,-y,0});
    }
    require(closed(run(points,.004))==0,"opposed cylindrical walls need an observed annular floor");
    for (int i=0; i<96; ++i) for (int j=0; j<6; ++j) {
        const double angle = 2*pi*(i+.5)/96, r = grooveInner+(grooveOuter-grooveInner)*(j+.5)/6;
        groovePoint({r*std::cos(angle),r*std::sin(angle),floor},{0,0,1});
    }
    const auto grooved = run(points,.004);
    const double cutVolume = pi*(depth-floor)*(grooveOuter*grooveOuter-grooveInner*grooveInner);
    require(closed(grooved)==1 && grooved.candidate.profileBodies.front().cavities.size()==3 &&
            std::abs(volume(grooved.candidate)-(expected-cutVolume))<.01,
            "a verified annular floor closes a groove while preserving both drilled holes");
}

void circularSlabContact() {
    constexpr double outer = 15, limit = 12, depth = 3, tubeInner = 8, tubeOuter = 10, tubeDepth = 12;
    const double pi = std::acos(-1.0), chord = std::sqrt(outer*outer-limit*limit);
    std::vector<PointSample> points;
    auto emit = [&](Vec3 p, Vec3 n) {
        auto turn = [](Vec3 v) { return rotate({v.x,.8*v.y-.6*v.z,.6*v.y+.8*v.z},.4); };
        const auto q = turn(p);
        points.push_back({points.size()+1,{q.x+231,q.y-17,q.z+19},turn(n),1,std::nullopt,true});
    };
    for (int i=0; i<68; ++i) for (int j=0; j<68; ++j) {
        const double x = outer*((i+.5)/34-1), y = outer*((j+.5)/34-1), r = std::hypot(x,y);
        if (r>=outer || x>=limit || y>=limit) continue;
        emit({x,y,depth},{0,0,1});
        if (r>tubeOuter) emit({x,y,0},{0,0,-1});
    }
    for (int i=0; i<160; ++i) for (int j=0; j<10; ++j) {
        const double angle = 2*pi*(i+.5)/160, x = std::cos(angle), y = std::sin(angle), z = depth*(j+.5)/10;
        if (outer*x<limit && outer*y<limit) emit({outer*x,outer*y,z},{x,y,0});
    }
    for (int i=0; i<40; ++i) for (int j=0; j<10; ++j) {
        const double s = chord*((i+.5)/20-1), z = depth*(j+.5)/10;
        emit({limit,s,z},{1,0,0});
        emit({s,limit,z},{0,1,0});
    }
    require(closed(run(points,.004))==0,"a large gap in the cap needs independently verified contact evidence");
    const auto partialSize = points.size();
    for (int i=0; i<68; ++i) for (int j=0; j<68; ++j) {
        const double x = outer*((i+.5)/34-1), y = outer*((j+.5)/34-1);
        if (std::hypot(x,y)<=tubeOuter) emit({x,y,0},{0,0,-1});
    }
    const auto complete = run(points,.004);
    const double segment = outer*outer*std::acos(limit/outer)-limit*chord;
    const double slabVolume = depth*(pi*outer*outer-2*segment);
    require(closed(complete)==1 && complete.parts.front().status=="closed_circular_slab" &&
            std::abs(volume(complete.candidate)-slabVolume)<.03,
            "two observed caps close an independently observed circular slab with two chords");
    points.resize(partialSize);
    for (int i=0; i<96; ++i) {
        const double angle = 2*pi*(i+.5)/96, x = std::cos(angle), y = std::sin(angle);
        for (int j=0; j<20; ++j) {
            const double z = -tubeDepth*(j+.5)/20;
            emit({tubeInner*x,tubeInner*y,z},{-x,-y,0});
            emit({tubeOuter*x,tubeOuter*y,z},{x,y,0});
        }
        for (int j=0; j<6; ++j) {
            const double r = tubeInner+(tubeOuter-tubeInner)*(j+.5)/6;
            emit({r*x,r*y,0},{0,0,1});
            emit({r*x,r*y,-tubeDepth},{0,0,-1});
        }
    }
    const auto result = run(points,.004);
    const double expected = slabVolume+pi*(tubeOuter*tubeOuter-tubeInner*tubeInner)*tubeDepth;
    require(closed(result)==2 && result.parts.back().status=="closed_circular_slab_inferred_contact" &&
            result.unassignedPointIds.empty() && std::abs(volume(result.candidate)-expected)<.03,
            "multiple observed chords and the independent tube recover both solids at the shared interface");
}

void roundedProfile() {
    constexpr double w = 42, d = 25, h = 5, r = 7;
    const double pi = std::acos(-1.0);
    std::vector<PointSample> points;
    auto emit = [&](Vec3 p, Vec3 n) {
        auto q = rotate(p,.43);
        q.x += 191; q.y -= 140; q.z += 25;
        points.push_back({points.size()+1,q,rotate(n,.43),1,std::nullopt,true});
    };
    for (int cap : {0,1}) for (int i=0;i<80;++i) for (int j=0;j<48;++j) {
        const double x = ((i+.5)/40-1)*w, y = ((j+.5)/24-1)*d;
        if (std::hypot(std::max(0.0,std::abs(x)-w+r),std::max(0.0,std::abs(y)-d+r)) <= r)
            emit({x,y,cap*h},{0,0,cap?1.0:-1.0});
    }
    for (int i=0;i<60;++i) for (int j=0;j<12;++j) {
        const double s=(i+.5)/30-1, z=(j+.5)*h/12;
        for (double sign : {-1,1}) {
            emit({s*(w-r),sign*d,z},{0,sign,0});
            emit({sign*w,s*(d-r),z},{sign,0,0});
        }
    }
    for (int q=0;q<4;++q) for (int i=0;i<40;++i) for (int j=0;j<12;++j) {
        const double a = (q+(i+.5)/40)*pi/2;
        const double cx = (q==0 || q==3 ? 1 : -1)*(w-r), cy=(q<2 ? 1 : -1)*(d-r);
        emit({cx+r*std::cos(a),cy+r*std::sin(a),(j+.5)*h/12},{std::cos(a),std::sin(a),0});
    }
    const auto result = run(points,.006);
    require(closed(result)==1 && result.candidate.profileBodies.size()==1,"outward circular walls close a rounded outline");
    const auto& outline=result.candidate.profileBodies.front().outline;
    require(std::count_if(outline.begin(),outline.end(),[](const auto& c){return c.circular;})==4,
            "all four observed corners rebuild as circular arcs");
    require(std::abs(volume(result.candidate)-h*(4*w*d-(4-pi)*r*r)) < .03,
            "rounded profile agrees with reference volume");
}
void safeBoundaryRecovery() {
    using namespace reconstruction::detail::assembly;
    const BoundaryLoop touching{{0,0},{1,0},{1,1},{0,1},{0,0},{-1,0},{-1,-1},{0,-1}};
    const auto cycles=splitBoundaryCycles(touching);
    require(cycles.size()==2,"touching alpha islands become two simple cycles, not a figure-eight");
    double cycleArea=0;
    for(const auto& loop:cycles){require(simpleBoundary(loop),"split alpha cycles are simple");cycleArea+=boundaryArea(loop);}
    require(std::abs(cycleArea-2)<1e-12,"splitting does not fill gaps or discard observed area");

    reconstruction::BoundaryRecoveryDiagnostics diagnostics;size_t snapped=0;
    const BoundaryLoop thin{{0,0},{10,0},{10,.2},{0,.2}};
    const auto retained=snapBoundary(thin,{{0,1,-.1,1}},5,1,diagnostics,snapped);
    require(simpleBoundary(retained)&&std::abs(boundaryArea(retained)-2)<1e-12,
        "opposite thin-wall edges cannot collapse onto the same intersection line");
    require(snapped==0&&diagnostics.snappingFallbacks>0,"rejected snapping does not claim repaired vertices");

    const BoundaryLoop inset{{.1,.1},{9.9,.1},{9.9,9.9},{.1,9.9}};
    const auto aligned=snapBoundary(inset,{{1,0,0,1},{1,0,-10,2},{0,1,0,3},{0,1,-10,4}},1,.5,diagnostics,snapped);
    require(simpleBoundary(aligned)&&std::abs(boundaryArea(aligned)-100)<1e-10&&snapped==4,
        "supported plane intersections can still recover shared box corners");
    const BoundaryLoop middleStart{{5,.1},{9.9,.1},{9.9,9.9},{.1,9.9},{.1,.1}};
    size_t cyclicSnaps=0;
    const auto cyclic=snapBoundary(middleStart,{{1,0,0,1},{1,0,-10,2},{0,1,0,3},{0,1,-10,4}},1,.5,diagnostics,cyclicSnaps);
    require(simpleBoundary(cyclic)&&std::abs(boundaryArea(cyclic)-100)<1e-10&&cyclicSnaps==4,
        "a boundary starting mid-edge recovers the same shared corners as a corner-starting loop");

    const BoundaryLoop concave{{0,0},{10,0},{10,1},{1,1},{1,9},{10,9},{10,10},{0,10}};
    const auto simplified=simplifyBoundary(concave,100,diagnostics);
    require(simpleBoundary(simplified)&&std::abs(boundaryArea(simplified)-boundaryArea(concave))<1e-12,
        "excessive simplification cannot publish a degenerate or reversed concave boundary");
    const BoundaryLoop outer{{0,0},{10,0},{10,10},{0,10}},hole{{2,2},{2,3},{3,3},{3,2}};
    auto moved=hole;for(auto& p:moved)p.x+=20;
    require(!sameBoundaryTopology({outer,hole},{outer,moved}),"a snapped hole cannot escape its observed outer loop");
    require(sameBoundaryTopology({outer,hole},{outer,hole}),"unchanged inner holes retain their topology");
}
}
int main(){try{safeBoundaryRecovery();separatedAndRotated();stagedAssembly();missingFaceAndPartial();finiteConcavityAndHole();noiseAndGaps();curvedParts();nonorthogonalFaces();drilledProfiles();profileCutouts();countersunkProfile();steppedProfile();annularTube();drilledCircularSlab();circularSlabContact();roundedProfile();invalidAndCancel();std::cout<<"Assembly reconstruction tests passed\n";return 0;}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
