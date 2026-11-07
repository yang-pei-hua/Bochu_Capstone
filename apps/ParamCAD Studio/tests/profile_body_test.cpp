#include <modeling/ModelingCore.h>
#include <modeling/PrimitiveEditing.h>
#include "io/FeatureCodec.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <QCoreApplication>
#include <iostream>
#include <stdexcept>

using namespace modeling;
namespace {
void require(bool ok,const std::string& text){if(!ok)throw std::runtime_error(text);}
double volume(const TopoDS_Shape& shape){GProp_GProps p;BRepGProp::VolumeProperties(shape,p,1e-10);return p.Mass();}
ProfileBodyParams ring() {
    ProfileBodyParams p;p.pose={{12,-18,25},{0,1,0},{1,0,0}};
    p.depth=p.sizeZ=8;p.referenceSizeX=p.referenceSizeY=p.sizeX=p.sizeY=20;
    p.envelope={{0,0,0,8,10,10}};p.cavities={{0,0,0,8,3,3},{0,0,6,8,8,8,6,6}};return p;
}
void editableAnnularBody() {
    const double pi=std::acos(-1.0);auto p=ring();ModelingCore core;
    std::vector<Feature> features{{1,"annular",FeatureType::ProfileBody,p},{2,"independent",FeatureType::BoxPrimitive,BoxPrimitiveParams{{{80,0,0}},5,5,5,false}}};
    require(core.restoreFeatures(features),core.lastError());require(std::abs(volume(core.bodyShape())-(672*pi+125))<1e-7,"annular groove retains its central material island");
    for(const auto& check:std::vector<std::pair<double,TopAbs_State>>{{4,TopAbs_IN},{7,TopAbs_OUT},{9,TopAbs_IN}}) {
        BRepClass3d_SolidClassifier classifier(core.bodyShape(),gp_Pnt(19,-18+check.first,25),1e-7);
        require(classifier.State()==check.second,"annular radial material membership");
    }
    std::vector<Feature> decoded;QString error;
    require(FeatureCodec::decode(FeatureCodec::encode(features),decoded,error),error.toStdString());
    ModelingCore restored;require(restored.restoreFeatures(decoded),restored.lastError());
    require(std::abs(volume(restored.bodyShape())-volume(core.bodyShape()))<1e-8,"analytic feature codec rebuilds the same material");
    p.cavities[0].startRadius=p.cavities[0].endRadius=4;
    ModelPatch edit;edit.expectedRevision=core.revision();edit.commands={EditFeatureCommand{1,p,true,ResizeAnchor::Center}};
    require(core.apply(edit).success,"edit cylindrical cavity");
    require(std::abs(volume(core.bodyShape())-(616*pi+125))<1e-7,"hole edit preserves the independent body");
    const auto revision=core.revision();const double before=volume(core.bodyShape());p.cavities[1].innerStartRadius=20;
    edit.expectedRevision=revision;edit.commands={EditFeatureCommand{1,p,true,ResizeAnchor::Center}};
    require(!core.apply(edit).success&&core.revision()==revision&&std::abs(volume(core.bodyShape())-before)<1e-9,"invalid annular edit is atomic");
}
void analyticOutlineAndResize() {
    ProfileBodyParams p;p.depth=p.sizeZ=5;p.referenceSizeX=p.sizeX=24;p.referenceSizeY=p.sizeY=18;
    p.outline={{false,0,0,20,0},{true,20,0,24,4,20,4,4,-std::acos(-1.0)/2,std::acos(-1.0)/2},
        {false,24,4,24,18},{false,24,18,0,18},{false,0,18,0,0}};
    TopoDS_Shape shape;std::string error;require(buildProfileBody(p,shape,error),error);
    const double expected=5*(416+4*std::acos(-1.0));require(std::abs(volume(shape)-expected)<1e-7,"mixed line/arc outline is analytic");
    FeatureParams adjusted;require(adjustedPrimitive(p,{28.8,14.4,10},ResizeAnchor::Center,adjusted,error),error);
    require(buildProfileBody(std::get<ProfileBodyParams>(adjusted),shape,error),error);
    require(std::abs(volume(shape)-expected*1.92)<1e-6,"dimension edit rebuilds nonuniformly scaled analytic body");
}
void noncircularCutout() {
    ProfileBodyParams p;p.depth=p.sizeZ=5;p.referenceSizeX=p.sizeX=40;p.referenceSizeY=p.sizeY=30;
    p.outline={{false,-20,-15,20,-15},{false,20,-15,20,15},{false,20,15,-20,15},{false,-20,15,-20,-15}};
    const double pi=std::acos(-1.0);
    p.profileVoids={{{false,-5,-3,5,-3},{true,5,-3,5,3,5,0,3,-pi/2,pi},
        {false,5,3,-5,3},{true,-5,3,-5,-3,-5,0,3,pi/2,pi}}};
    TopoDS_Shape shape;std::string error;require(buildProfileBody(p,shape,error),error);
    const double expected=(1200-60-9*pi)*5;
    require(std::abs(volume(shape)-expected)<1e-7,"obround through cut preserves its straight and circular boundaries");
    std::vector<Feature> decoded;QString failure;
    require(FeatureCodec::decode(FeatureCodec::encode({{1,"slotted plate",FeatureType::ProfileBody,p}}),decoded,failure),failure.toStdString());
    ModelingCore restored;require(restored.restoreFeatures(decoded),restored.lastError());
    require(std::abs(volume(restored.bodyShape())-expected)<1e-7,"noncircular cut survives feature serialization and independent rebuild");
    p.sizeX*=1.1;p.sizeY*=.8;
    require(refreshProfileBodyDimensions(p,error) && buildProfileBody(p,shape,error),error);
    require(std::abs(volume(shape)-expected*.88)<1e-6,"reference dimension refresh preserves scaled noncircular cut material: "+std::to_string(volume(shape))+" expected "+std::to_string(expected*.88));
    p.profileVoids.clear();p.sizeX=p.referenceSizeX;p.sizeY=p.referenceSizeY;
    AxialSection transverse{0,0,-25,25,1,1};transverse.centerZ=2.5;transverse.axis={1,0,0};p.cavities={transverse};
    require(buildProfileBody(p,shape,error),error);
    require(std::abs(volume(shape)-(6000-40*pi))<1e-7,"transverse hole cuts the actual side walls");
    require(FeatureCodec::decode(FeatureCodec::encode({{1,"side drilled plate",FeatureType::ProfileBody,p}}),decoded,failure),failure.toStdString());
    require(restored.restoreFeatures(decoded) && std::abs(volume(restored.bodyShape())-volume(shape))<1e-7,"oriented section axis survives feature serialization");
}

void roundedEndBevels() {
    ProfileBodyParams p;
    p.depth = p.sizeZ = 12;
    p.referenceSizeX = p.sizeX = 31;
    p.referenceSizeY = p.sizeY = 36;
    p.roundedEnd = RoundedEndProfile{{4,-2,3},13,18,{1,2},{2,1}};
    p.cavities = {{4,-2,-1,16,3,3}};
    TopoDS_Shape shape;
    std::string error;
    require(buildProfileBody(p,shape,error),error);
    const double pi = std::acos(-1.);
    const auto band = [&](double start, double end, double depth) {
        return depth*(13*(start+end)+pi*(start*start+start*end+end*end)/6);
    };
    const auto expected = band(16,18,1)+band(18,18,9)+band(18,17,2)-pi*9*12;
    require(std::abs(volume(shape)-expected)<1e-6,"rounded profile retains the straight and conical bevels");
    std::vector<Feature> decoded;
    QString failure;
    require(FeatureCodec::decode(FeatureCodec::encode({{1,"rounded plate",FeatureType::ProfileBody,p}}),decoded,failure),
            failure.toStdString());
    ModelingCore restored;
    require(restored.restoreFeatures(decoded),restored.lastError());
    require(std::abs(volume(restored.bodyShape())-expected)<1e-6,"round end parameters survive serialization");
    require(refreshProfileBodyDimensions(p,error) && buildProfileBody(p,shape,error),error);
    require(std::abs(volume(shape)-expected)<1e-6,"rebasing dimensions preserves the beveled body");
}

void selectiveRimRounding() {
    ProfileBodyParams p;
    p.depth = p.sizeZ = 11;
    p.referenceSizeX = p.sizeX = 30;
    p.referenceSizeY = p.sizeY = 24;
    const auto pi = std::acos(-1.);
    p.outline = {{false,0,0,30,0},{false,30,0,30,16},{true,30,16,22,24,22,16,8,0,pi/2},
                 {false,22,24,0,24},{false,0,24,0,0}};
    TopoDS_Shape plain, rounded;
    std::string error;
    require(buildProfileBody(p,plain,error),error);
    p.rimRounds = {{2,1.5,false},{2,2,true}};
    require(buildProfileBody(p,rounded,error),error);
    int tori = 0;
    for (TopExp_Explorer it(rounded,TopAbs_FACE); it.More(); it.Next())
        tori += BRepAdaptor_Surface(TopoDS::Face(it.Current())).GetType()==GeomAbs_Torus;
    require(tori==2 && volume(rounded)<volume(plain),"selective rounding builds both toroidal transitions");
    std::vector<Feature> decoded;
    QString failure;
    require(FeatureCodec::decode(FeatureCodec::encode({{1,"rounded rims",FeatureType::ProfileBody,p}}),decoded,failure),
            failure.toStdString());
    ModelingCore restored;
    require(restored.restoreFeatures(decoded),restored.lastError());
    require(std::abs(volume(restored.bodyShape())-volume(rounded))<1e-6,"rim rounds survive feature serialization");
}

void obliqueProfileTrim() {
    ProfileBodyParams p;
    p.depth = p.sizeZ = 10;
    p.referenceSizeX = p.sizeX = 20;
    p.referenceSizeY = p.sizeY = 10;
    p.outline = {{false,0,0,20,0},{false,20,0,20,10},{false,20,10,0,10},{false,0,10,0,0}};
    const auto normalLength = std::hypot(.2,1.);
    p.trimPlanes = {{{0,0,9},{.2/normalLength,0,1/normalLength}}};
    TopoDS_Shape shape;
    std::string error;
    require(buildProfileBody(p,shape,error),error);
    require(std::abs(volume(shape)-1400)<1e-6,"oblique end trim preserves the measured half-space");
    std::vector<Feature> decoded;
    QString failure;
    require(FeatureCodec::decode(FeatureCodec::encode({{1,"trimmed profile",FeatureType::ProfileBody,p}}),decoded,failure),
            failure.toStdString());
    ModelingCore restored;
    require(restored.restoreFeatures(decoded),restored.lastError());
    require(std::abs(volume(restored.bodyShape())-1400)<1e-6,"the trim plane survives feature serialization");
    require(refreshProfileBodyDimensions(p,error) && buildProfileBody(p,shape,error),error);
    require(std::abs(volume(shape)-1400)<1e-6,"the trim plane survives reference dimension refresh");
}
}
int main(int argc,char** argv){QCoreApplication app(argc,argv);try{editableAnnularBody();analyticOutlineAndResize();noncircularCutout();roundedEndBevels();selectiveRimRounding();obliqueProfileTrim();std::cout<<"Profile body tests passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
