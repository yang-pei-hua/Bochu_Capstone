#include "reconstruction/analysis/SurfaceAnalysis.h"
#include "reconstruction/recognition/BoxRecognizer.h"
#include "reconstruction/recognition/CompoundRecognizer.h"
#include "reconstruction/recognition/MixedFeatureRecognizer.h"
#include "reconstruction/modeling/ModelingAdapter.h"
#include "reconstruction/testing/SyntheticPointCloud.h"
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <cmath>
#include <array>
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace reconstruction;
namespace {
void require(bool condition,const std::string& message) {if(!condition)throw std::runtime_error(message);}
void snapshotsAndPatches() {
    std::vector<PointSample> samples;
    PlaneEvidence plane;plane.plane={{0,0,1},0};plane.confidence=1;
    for(int block=0;block<2;++block) for(int x=0;x<10;++x) for(int y=0;y<10;++y) {
        PointSample p;p.id=samples.size()+1;p.position={double(block*100+x),double(y),0};p.normal=modeling::Vec3{0,0,1};
        samples.push_back(p);plane.supportPointIds.push_back(p.id);
    }
    PointStore points(samples);SurfaceAnalysisOptions options;options.connectivityDistance=2;
    SurfaceAnalysisResult analysis;std::string error;
    require(indexSurfaceEvidence(points,{plane},options,analysis,error),error);
    require(analysis.surfaces.size()==1 && analysis.patches.size()==2,"one equation retains two disconnected patches");
    require(analysis.unassignedPointIds.empty(),"all observations accounted for");
    auto detectedOptions=options;
    detectedOptions.detection.detectCylinders=detectedOptions.detection.detectSpheres=
        detectedOptions.detection.detectCones=detectedOptions.detection.detectTori=false;
    detectedOptions.detection.plane.minimumSupportPoints=20;
    detectedOptions.detection.plane.maximumPlanes=1;
    detectedOptions.detection.plane.clusterEpsilon=2;
    SurfaceAnalysisResult detected;
    require(analyzeSurfaces(points,detectedOptions,detected,error),error);
    require(detected.surfaces.size()==1 && detected.patches.size()==2 && detected.unassignedPointIds.empty(),
            "detector deduplication preserves disconnected coplanar observations even at equation quota");
    bool coplanar=false,adjacent=false;
    for(const auto& r:analysis.relations) {coplanar|=r.kind==SurfaceRelationKind::Coplanar;adjacent|=r.kind==SurfaceRelationKind::Adjacent;}
    require(coplanar && !adjacent,"distant coplanarity must not imply contact");
    SurfaceRelationHint hint;hint.firstPoints=analysis.patches[0].supportPointIds;
    hint.secondPoints=analysis.patches[1].supportPointIds;hint.kind=SurfaceRelationKind::Coplanar;
    require(applySurfaceRelationHints(points,{hint},analysis,error),error);
    require(std::any_of(analysis.relations.begin(),analysis.relations.end(),[](const auto& r){return !r.automatic;}),
            "validated manual relations enter shared analysis");
    hint.kind=SurfaceRelationKind::Adjacent;
    require(!applySurfaceRelationHints(points,{hint},analysis,error),"unsupported manual contact cannot invent geometry");
    require(analysisMatches(points,analysis),"valid snapshot reusable");
    samples[0].position.x+=0.01;
    require(!analysisMatches(PointStore(samples),analysis),"changed geometry invalidates analysis");
    require(!analysisMatches(PointStore(points.points(),LengthUnit::Meter),analysis),"changed unit invalidates analysis");
    auto changed=options;changed.detection.cylinder.minimumRadius=3;
    require(!analysisSettingsMatch(options,changed),"changed detection settings invalidate cache");
    changed=options;changed.relations.distanceTolerance*=2;
    require(!analysisSettingsMatch(options,changed),"changed relation settings invalidate cache");
    plane.supportPointIds.push_back(99999);
    require(!indexSurfaceEvidence(points,{plane},options,analysis,error),"foreign observation IDs rejected");
}
void sharedBox() {
    SyntheticBoxRequest request;request.box.sizeX=20;request.box.sizeY=30;request.box.sizeZ=10;request.samplesPerEdge=15;
    const auto points=makeSyntheticBoxSurface(request);
    PlaneDetectionOptions plane;plane.distanceThreshold=0.02;plane.minimumSupportPoints=40;
    auto options=recognitionAnalysisOptions(points,plane,{});
    options.detection.detectCylinders=options.detection.detectSpheres=options.detection.detectCones=options.detection.detectTori=false;
    SurfaceAnalysisResult analysis;std::string error;
    require(analyzeSurfaces(points,options,analysis,error),error);
    require(analysis.detectionPasses==1 && analysis.surfaces.size()==6,"one shared plane pass");
    BoxCandidate box;
    require(recognizeBoxFromAnalysis(points,analysis,{},box,error),error);
    require(std::abs(box.primitive.sizeX*box.primitive.sizeY*box.primitive.sizeZ-6000)<1e-6,"shared plane box geometry");
    CompoundCandidate solid;
    require(recognizeParametricSolidFromAnalysis(points,analysis,solid,error),error);
    require(solid.boxes.size()==1 && analysis.detectionPasses==1,"same analysis feeds multiple recognizers");
    auto changed=points.points();changed.front().normalOrientationKnown=false;
    require(!recognizeParametricSolidFromAnalysis(PointStore(changed),analysis,solid,error),"recognizer rejects stale evidence");
    SurfaceAnalysisResult cancelled;
    require(!analyzeSurfaces(points,options,cancelled,error,{},[]{return true;}) && error=="Cancelled","cancel before detector");
}
PointStore mixedCloud(bool incomplete=false) {
    std::vector<PointSample> points;
    auto emit=[&](modeling::Vec3 p,modeling::Vec3 n) {
        PointSample s;s.id=points.size()+1;s.position=p;s.normal=n;points.push_back(s);
    };
    const double pi=std::acos(-1.0);
    // Analytic source union: 120x80x5 plate; radius-8,height-22 boss at
    // (30,30); radius-7 through-hole at (85,45). No internal cap observations.
    for(int axis=0;axis<3;++axis) for(int side=0;side<2;++side) {
        const std::array<double,3> lengths{120,80,5};
        const int u=(axis+1)%3,v=(axis+2)%3;
        for(int i=0;i<int(lengths[u]);++i) for(int j=0;j<int(lengths[v]);++j) {
            std::array<double,3> q{};q[axis]=side*lengths[axis];q[u]=i+0.5;q[v]=j+0.5;
            if(std::hypot(q[0]-85,q[1]-45)<7) continue;
            if(axis==2 && side==1 && std::hypot(q[0]-30,q[1]-30)<8) continue;
            modeling::Vec3 n{};if(axis==0)n.x=side*2-1;if(axis==1)n.y=side*2-1;if(axis==2)n.z=side*2-1;
            emit({q[0],q[1],q[2]},n);
        }
    }
    for(int which=0;which<2;++which) {
        double r=which?7:8,cx=which?85:30,cy=which?45:30,height=which?5:22,z0=which?0:5;
        for(int z=0;z<int(height*2);++z) for(int a=0;a<96;++a) {
            double angle=2*pi*(a+0.5)/96,c=std::cos(angle),s=std::sin(angle),sign=which?-1:1;
            emit({cx+r*c,cy+r*s,z0+(z+0.5)/2},{sign*c,sign*s,0});
        }
        if(!which && !incomplete) for(int x=-8;x<8;++x) for(int y=-8;y<8;++y)
            if(std::hypot(x+0.5,y+0.5)<8) emit({30+x+0.5,30+y+0.5,27},{0,0,1});
    }
    return PointStore(std::move(points));
}
void attachedBossAndHole() {
    const auto points=mixedCloud();PlaneDetectionOptions planes;planes.distanceThreshold=0.05;planes.minimumSupportPoints=30;
    CylinderDetectionOptions cylinders;cylinders.distanceThreshold=0.05;cylinders.minimumSupportPoints=30;
    auto options=recognitionAnalysisOptions(points,planes,cylinders);
    options.detection.detectSpheres=options.detection.detectCones=options.detection.detectTori=false;
    SurfaceAnalysisResult analysis;std::string error;
    require(analyzeSurfaces(points,options,analysis,error),error);
    CompoundCandidate candidate;
    require(recognizeMixedFeatures(points,analysis,candidate,error),error);
    require(candidate.boxes.size()==1 && candidate.curvedSolids.size()==1 && candidate.throughHoles.size()==1,"host, attached boss and cavity all retained");
    modeling::ModelingCore core;
    auto result=commitCompoundCandidate(candidate,core);
    require(result.success,result.error);
    int solids=0;for(TopExp_Explorer e(core.bodyShape(),TopAbs_SOLID);e.More();e.Next())++solids;
    require(solids==1,"attached curved primitive must fuse into host");
    GProp_GProps mass;BRepGProp::VolumeProperties(core.bodyShape(),mass);
    const double expected=120*80*5+std::acos(-1.0)*8*8*22-std::acos(-1.0)*7*7*5;
    require(std::abs(mass.Mass()-expected)<expected*0.005,"mixed boolean volume");
    const auto missing=mixedCloud(true);
    require(analyzeSurfaces(missing,options,analysis,error),error);
    require(!recognizeMixedFeatures(missing,analysis,candidate,error),"unobserved boss top must not be invented");
    require(!recognizeParametricSolidFromAnalysis(missing,analysis,candidate,error),"fallback must not silently discard an unsupported attached feature");
}
void adaptiveRegions() {
    std::vector<PointSample> samples;std::vector<PointId> ids;
    for(int region=0;region<2;++region)for(int x=0;x<10;++x)for(int y=0;y<10;++y) {
        PointSample p;p.id=samples.size()+1;p.position={double(region*100+x*(region?4:1)),double(y*(region?4:1)),0};
        ids.push_back(p.id);samples.push_back(p);
    }
    PointStore points(samples);
    const auto groups=connectedPointComponents(points,ids,1.5,12);
    require(groups.size()==2 && groups[0].size()==100 && groups[1].size()==100,"density-adaptive regions keep sparse features connected and remote features separate");
}
void localRecoveryAndConstraints() {
    std::string error;
    const auto source=mixedCloud();
    std::vector<PointSample> reduced;
    std::size_t capCount=0;
    for(const auto& p:source.points()) {
        if(p.position.z==27 && p.normal && p.normal->z>0.9 && capCount++%25!=0)continue;
        reduced.push_back(p);
    }
    PointStore working(std::move(reduced));
    PlaneDetectionOptions plane;plane.distanceThreshold=0.05;plane.minimumSupportPoints=30;
    CylinderDetectionOptions cylinder;cylinder.distanceThreshold=0.05;cylinder.minimumSupportPoints=30;
    auto options=recognitionAnalysisOptions(working,plane,cylinder);
    options.detection.detectSpheres=options.detection.detectCones=options.detection.detectTori=false;
    SurfaceAnalysisResult analysis;
    require(analyzeSurfaces(working,options,analysis,error),error);
    const auto before=pointStoreFingerprint(working);
    CompoundCandidate result;std::vector<LocalAnalysisFailure> failures;
    require(recognizeMixedFeatures(working,analysis,result,error,&failures,&source),error);
    const auto repairedFailure=std::find_if(failures.begin(),failures.end(),[](const auto& f){return f.repaired;});
    require(repairedFailure!=failures.end(),"sparse cap repaired by bounded source retrieval");
    require(!repairedFailure->recoveredPointIds.empty() && !repairedFailure->candidates.empty() && !repairedFailure->repairedCandidates.empty(),"failure keeps observations and alternative candidates");
    require(before==pointStoreFingerprint(working),"recovery never mutates working observations");
    require(result.boxes.size()==1 && result.throughHoles.size()==1 && result.curvedSolids.size()==1,"recovery preserves host and hole");
    const auto missing=mixedCloud(true);
    require(analyzeSurfaces(missing,options,analysis,error),error);failures.clear();
    require(!recognizeMixedFeatures(missing,analysis,result,error,&failures,&missing),"recovery cannot invent unobserved cap");
    require(!failures.empty() && !failures.back().repaired && failures.back().reason==LocalFailureReason::MissingEndFace,"missing-end diagnostic survives failure");
    LocalAnalysisFailure bad;bad.pointIds={999999999};PointStore local({});SurfaceAnalysisResult repaired;
    require(!recoverLocalSurfaces(missing,missing,analysis,bad,local,repaired,error),"foreign recovery ID rejected");
    bad.pointIds={missing.points()[0].id};
    require(!recoverLocalSurfaces(missing,PointStore(missing.points(),LengthUnit::Meter),analysis,bad,local,repaired,error),"source units checked");

    std::vector<PointSample> samples;std::vector<PrimitiveEvidence> evidence;
    // Two slightly misfit parallel planes and an unrelated exact plane.
    for(int face=0;face<3;++face) {
        PlaneEvidence e;e.confidence=1;
        const double tilt=face==1?0.01:0;
        e.plane={{tilt,0,std::sqrt(1-tilt*tilt)},-double(face*5)};
        for(int x=0;x<10;++x)for(int y=0;y<10;++y) {
            PointSample p;p.id=samples.size()+1;p.position={double(x),double(y),double(face*5)};p.normal=modeling::Vec3{0,0,1};
            samples.push_back(p);e.supportPointIds.push_back(p.id);
        }
        evidence.push_back(e);
    }
    const PointStore planes(samples);SurfaceAnalysisOptions settings;settings.connectivityDistance=2;
    require(indexSurfaceEvidence(planes,evidence,settings,analysis,error),error);
    RelationOptimizationReport report;
    SurfaceRelation parallel{1,2,SurfaceRelationKind::Parallel,0,0.95,true};
    SurfaceRelation conflict{1,2,SurfaceRelationKind::Perpendicular,0,0.5,true};
    require(optimizeSurfaceRelations(planes,analysis,{parallel,conflict},report,error),error);
    require(report.accepted.size()==1 && report.rejected.size()==1,"feasible relation subset rejects conflict");
    const auto& p0=std::get<PlaneEvidence>(analysis.surfaces[0]).plane;
    const auto& p1=std::get<PlaneEvidence>(analysis.surfaces[1]).plane;
    require(std::abs(p0.normal.x-p1.normal.x)<0.002,"joint optimization aligns observed planes");
    require(report.afterError<report.beforeError,"optimization reevaluates and improves observed residuals");
    const auto& stable=std::get<PlaneEvidence>(analysis.surfaces[2]).plane;
    require(stable.normal.x==0 && stable.normal.z==1 && stable.offset==-10,"unrelated surface parameters stay fixed");
    require(analysisMatches(planes,analysis),"optimized analysis remains snapshot scoped");
    std::vector<PointSample> cornerPoints;std::vector<PrimitiveEvidence> cornerEvidence;
    for(int face=0;face<2;++face) {
        PlaneEvidence e;e.confidence=1;e.plane.normal=face?modeling::Vec3{std::sqrt(1-0.0001),0,0.01}:modeling::Vec3{0,0,1};
        for(int u=0;u<10;++u)for(int v=0;v<10;++v) {
            PointSample p;p.id=cornerPoints.size()+1;p.position=face?modeling::Vec3{0,double(u),double(v)}:modeling::Vec3{double(u),double(v),0};
            p.normal=face?modeling::Vec3{1,0,0}:modeling::Vec3{0,0,1};cornerPoints.push_back(p);e.supportPointIds.push_back(p.id);
        }
        cornerEvidence.push_back(e);
    }
    PointStore corner(cornerPoints);
    require(indexSurfaceEvidence(corner,cornerEvidence,settings,analysis,error),error);
    require(optimizeSurfaceRelations(corner,analysis,{{1,2,SurfaceRelationKind::Perpendicular,0,1,true}},report,error),error);
    require(report.accepted.size()==1 && report.afterError<report.beforeError,"perpendicular fit improves observed corner planes");
    std::vector<PointSample> rounds;std::vector<PrimitiveEvidence> cylinders;
    for(int part=0;part<2;++part) {
        CylinderEvidence c;c.confidence=1;c.cylinder={{part?0.03:0,0,0},{0,0,1},double(3+part)};
        for(int z=0;z<8;++z)for(int a=0;a<72;++a) {
            const double t=2*std::acos(-1.0)*a/72;PointSample p;p.id=rounds.size()+1;
            p.position={c.cylinder.radius*std::cos(t),c.cylinder.radius*std::sin(t),double(z+part*20)};
            p.normal=modeling::Vec3{std::cos(t),std::sin(t),0};rounds.push_back(p);c.supportPointIds.push_back(p.id);
        }
        cylinders.push_back(c);
    }
    PointStore coaxial(rounds);settings.connectivityDistance=2;
    require(indexSurfaceEvidence(coaxial,cylinders,settings,analysis,error),error);
    require(optimizeSurfaceRelations(coaxial,analysis,{{1,2,SurfaceRelationKind::Coaxial,0,1,true}},report,error),error);
    require(report.accepted.size()==1 && report.afterError<report.beforeError,"joint coaxial fit improves real cylinder support");
    const auto& c0=std::get<CylinderEvidence>(analysis.surfaces[0]);
    const auto& c1=std::get<CylinderEvidence>(analysis.surfaces[1]);
    require(std::abs(c0.cylinder.axisPoint.x-c1.cylinder.axisPoint.x)<0.01,"coaxial axes converge");
    require(std::abs(c0.cylinder.radius-3)<0.01 && std::abs(c1.cylinder.radius-4)<0.01,"coaxial fit preserves independent radii");

}

}
int main() {
    try {snapshotsAndPatches();sharedBox();attachedBossAndHole();adaptiveRegions();localRecoveryAndConstraints();std::cout<<"Shared surface analysis tests passed\n";return 0;}
    catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
