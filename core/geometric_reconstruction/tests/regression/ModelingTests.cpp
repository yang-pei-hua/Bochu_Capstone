#include "TestSupport.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>

namespace regression {
namespace {
reconstruction::PointStore perforatedBlock(double floor, bool missingBase) {
    using namespace reconstruction;
    std::vector<PointSample> points;
    auto add = [&](double x,double y,double z,modeling::Vec3 normal) {
        points.push_back({points.size()+1,{x,y,z},normal});
    };
    constexpr double height=8, radius=1.5;
    for (int i=0;i<=24;++i) for (int j=0;j<=24;++j) {
        const double x=-4+8.*i/24, y=-3+6.*i/24, z=height*j/24;
        add(x,-3,z,{0,-1,0}); add(x,3,z,{0,1,0});
        add(-4,y,z,{-1,0,0}); add(4,y,z,{1,0,0});
        const double v=-3+6.*j/24;
        const bool inHole=std::hypot(x,v)<radius;
        if (!missingBase && (floor>0 || !inHole)) add(x,v,0,{0,0,-1});
        if (!inHole) add(x,v,height,{0,0,1});
        else if (floor>0) add(x,v,floor,{0,0,1});
    }
    for (int i=0;i<80;++i) for (int j=0;j<=24;++j) {
        const double angle=2*std::acos(-1.)*i/80;
        const double x=std::cos(angle), y=std::sin(angle);
        add(radius*x,radius*y,floor+(height-floor)*j/24,{-x,-y,0});
    }
    return PointStore(std::move(points));
}
}

void testProjectionExtrusion() {
    using namespace reconstruction;
    ProjectionExtrusionOptions options;
    options.distanceTolerance=.04;
    for (const double floor : {0.,4.}) {
        ProjectionExtrusionResult fit;
        const bool accepted=fitProjectionExtrusion(perforatedBlock(floor,false),options,fit);
        require(accepted, "Projection should recover a supported hole: "+fit.reason);
        require(fit.operations.size()==(floor==0?1:2),"Through hole should use an inner loop, blind hole should use Cut");
        require(fit.quality.p95Distance<options.distanceTolerance,"Projection must fit the actual boundary");
        modeling::ModelingCore core;
        std::string error;
        require(buildProjectionProgram(fit.operations,core,error),error);
        const double expected=384-std::acos(-1.)*1.5*1.5*(8-floor);
        requireNear(volumeOf(core.bodyShape()),expected,.3,"Projection hole volume");
        const auto block=BRepPrimAPI_MakeBox(gp_Pnt(-4,-3,0),8,6,8).Shape();
        const auto tool=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,floor),gp_Dir(0,0,1)),1.5,9-floor).Shape();
        BRepAlgoAPI_Cut reference(block,tool);
        require(reference.IsDone(),"Build independent analytical benchmark solid");
        BRepAlgoAPI_Common intersection(reference.Shape(),core.bodyShape());
        require(intersection.IsDone(),"Measure projection benchmark intersection");
        const double overlap=volumeOf(intersection.Shape());
        const double iou=overlap/(volumeOf(reference.Shape())+volumeOf(core.bodyShape())-overlap);
        require(iou>.999,"Projection hole must match the analytical target volume and location");
        if (floor>0) {
            const auto feature=core.features().back();
            auto cut=std::get<modeling::ExtrudeFeatureParams>(feature.params);
            cut.depth-=1;
            modeling::ModelPatch patch;
            patch.commands.push_back(modeling::EditFeatureCommand{feature.id,cut,true,{}});
            require(core.apply(patch).success,"Recovered cut depth must be editable");
            requireNear(volumeOf(core.bodyShape()),expected+std::acos(-1.)*2.25,.3,"Edited blind hole volume");
        }
    }
    ProjectionExtrusionResult rejected;
    require(!fitProjectionExtrusion(perforatedBlock(0,true),options,rejected),
            "Do not publish a part with a significant unobserved face");
    require(rejected.operations.empty(),"Rejected programs must remain diagnostic only");
    require(!fitProjectionExtrusion(perforatedBlock(0,false),options,rejected,{},[]{return true;}) &&
            rejected.reason=="Cancelled","Cancelled fit must publish no operations");
}

void testProjectionBooleanIsScoped() {
    using namespace modeling;
    using namespace reconstruction;
    ProjectionOperation base;
    base.sketch.entities={{1,Rectangle2D{0,0,4,4}}};
    base.depth=4;
    auto neighbor=base;
    neighbor.sketch.entities={{1,Rectangle2D{4,0,2,4}}};
    auto cut=base;
    cut.sketch.entities={{1,Rectangle2D{3,1,2,2}}};
    cut.operation=ExtrudeOperation::Cut;
    CompoundCandidate candidate;
    candidate.projectionPrograms={{neighbor},{base,cut}};
    ModelingCore core;
    const auto committed=commitCompoundCandidate(candidate,core);
    require(committed.success,committed.error);
    requireNear(volumeOf(core.bodyShape()),88,1e-6,"Scoped cut cannot remove adjacent body material");
    ModelingCore reopened;
    require(reopened.restoreFeatures(core.features()),reopened.lastError());
    requireNear(volumeOf(reopened.bodyShape()),88,1e-6,"Scoped boolean must survive replay");
}

void testCompoundCommitFusesSolidsAtomically() {
    reconstruction::CompoundCandidate candidate;
    reconstruction::BoxCandidate base,upright;
    base.primitive.sizeX=80;base.primitive.sizeY=60;base.primitive.sizeZ=8;
    upright.primitive.pose.origin={0,0,8};
    upright.primitive.sizeX=80;upright.primitive.sizeY=8;upright.primitive.sizeZ=60;
    candidate.boxes={base,upright};
    reconstruction::PrismCandidate rib;
    rib.plane={{36,8,8},{1,0,0},{0,1,0}};
    rib.polygon={{0,0},{40,0},{0,50}};rib.depth=8;
    candidate.prisms.push_back(rib);
    modeling::ModelingCore core;
    const auto committed=reconstruction::commitCompoundCandidate(candidate,core,0);
    require(committed.success,committed.error);
    require(core.features().size()==4,"L part requires two boxes, one sketch and one extrusion");
    requireNear(volumeOf(core.bodyShape()),84800,1e-6,"Fused L bracket volume");
    const auto stale=reconstruction::commitCompoundCandidate(candidate,core,0);
    require(!stale.success&&core.features().size()==4,"Stale patch must leave compound unchanged");
    for (const auto& feature : core.features())
        require(feature.reconstructed, "All generated features retain reconstruction provenance");
    const auto replaced=reconstruction::commitCompoundCandidate(candidate,core,core.revision(),true);
    require(replaced.success && core.features().size()==4,
            "Replacing generated compound does not append duplicate features");
    requireNear(volumeOf(core.bodyShape()),84800,1e-6,"Replacement preserves fused volume");
    modeling::SketchFeatureParams dependent;
    dependent.plane=modeling::featureFace(core.features().back().id,modeling::FaceRole::EndFace);
    const auto authored=core.execute(modeling::AddFeatureCommand{dependent});
    require(authored.success,"Append authored dependent for replacement guard");
    const auto revision=core.revision();
    const auto blocked=reconstruction::commitCompoundCandidate(candidate,core,revision,true);
    require(!blocked.success && core.revision()==revision && core.features().size()==5,
            "Authored dependents reject replacement atomically instead of being deleted");
    requireNear(volumeOf(core.bodyShape()),84800,1e-6,"Rejected replacement preserves native body");
    modeling::ModelingCore empty;
    candidate.prisms[0].depth=-1;
    const auto invalid=reconstruction::commitCompoundCandidate(candidate,empty,0);
    require(!invalid.success&&empty.features().empty(),"Invalid prism must roll back all preceding boxes");
}

void testCurvedFeaturesEditAndRollback() {
    const double pi=std::acos(-1.0);
    const std::array<modeling::CurvedPrimitiveParams,4> primitives{{
        modeling::SpherePrimitiveParams{{3,-2,5},4},
        modeling::CylinderPrimitiveParams{{3,-2,5},{1,2,3},4,10},
        modeling::ConePrimitiveParams{{3,-2,5},{1,2,3},4,0,10},
        modeling::TorusPrimitiveParams{{3,-2,5},{1,2,3},10,3}
    }};
    const std::array<double,4> volumes{{4*pi*64/3,pi*16*10,pi*16*10/3,2*pi*pi*10*9}};
    const std::array<modeling::FeatureType,4> types{{modeling::FeatureType::SpherePrimitive,
        modeling::FeatureType::CylinderPrimitive,modeling::FeatureType::ConePrimitive,modeling::FeatureType::TorusPrimitive}};
    for(std::size_t i=0;i<primitives.size();++i) {
        modeling::ModelingCore core;reconstruction::CompoundCandidate candidate;
        reconstruction::CurvedSolidCandidate solid;solid.primitive=primitives[i];candidate.curvedSolids.push_back(solid);
        auto result=reconstruction::commitCompoundCandidate(candidate,core,0);
        require(result.success,result.error);
        require(core.features().size()==1&&core.features()[0].type==types[i],"Curved primitive must retain its semantic feature type");
        requireNear(volumeOf(core.bodyShape()),volumes[i],1e-6,"curved primitive volume");
        auto edited=std::visit([](auto p)->modeling::FeatureParams {
            using P=std::decay_t<decltype(p)>;
            if constexpr(std::is_same_v<P,modeling::SpherePrimitiveParams>)p.radius*=1.25;
            else if constexpr(std::is_same_v<P,modeling::TorusPrimitiveParams>)p.minorRadius*=1.25;
            else p.height*=1.5;
            return p;
        },primitives[i]);
        modeling::ModelPatch edit;edit.expectedRevision=core.revision();
        edit.commands.push_back(modeling::EditFeatureCommand{core.features()[0].id,edited});
        require(core.apply(edit).success,"Curved dimensions must remain editable through ModelPatch");
        const double factor=i==0?std::pow(1.25,3):i==3?1.25*1.25:1.5;
        requireNear(volumeOf(core.bodyShape()),volumes[i]*factor,1e-6,"edited curved volume");
        const auto revision=core.revision();const double volume=volumeOf(core.bodyShape());
        modeling::ModelPatch invalid;invalid.expectedRevision=revision;
        invalid.commands.push_back(modeling::AddFeatureCommand{edited});
        invalid.commands.push_back(modeling::AddFeatureCommand{modeling::TorusPrimitiveParams{{},{0,0,1},2,3}});
        require(!core.apply(invalid).success,"Self-intersecting torus must reject the complete patch");
        require(core.revision()==revision&&core.features().size()==1,"Failed curved patch must restore feature graph and revision");
        requireNear(volumeOf(core.bodyShape()),volume,1e-9,"failed patch body rollback");
    }
    std::string error;
    require(!modeling::validateCurvedPrimitiveParams(modeling::SpherePrimitiveParams{{},std::numeric_limits<double>::quiet_NaN()},error),"NaN primitive must be rejected");
    require(!modeling::validateCurvedPrimitiveParams(modeling::CylinderPrimitiveParams{{},{0,0,0},3,10},error),"Zero axis must be rejected");
}
} // namespace regression
