#include "app/MainWindow.h"
#include "app/ModelingController.h"
#include "app/ProjectController.h"
#include "core/SketchFrame.h"
#include "io/FeatureCodec.h"
#include "ui/viewport/VTKViewer.h"
#include "ui/panels/SketchToolPalette.h"
#include <modeling/SemanticFaces.h>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <QAction>
#include <QApplication>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QToolButton>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNew.h>
#include <vtkTransform.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
bool near(double a, double b) { return std::abs(a-b) < 1e-5; }
double top(const TopoDS_Shape& shape) {
    Bnd_Box bounds; BRepBndLib::Add(shape,bounds);
    double x0,y0,z0,x1,y1,z1; bounds.Get(x0,y0,z0,x1,y1,z1); return z1;
}
FeatureId add(ModelingCore& core, const FeatureParams& params) {
    const auto result = core.execute(AddFeatureCommand{params});
    require(result.success,"add feature"); return result.featureId;
}
SketchFeatureParams square(SketchPlaneReference plane) {
    return {std::move(plane),{{1,Rectangle2D{1,1,2,2}}}};
}
void kernelReferences() {
    ModelingCore core;
    BoxPrimitiveParams box; box.sizeX=10; box.sizeY=20; box.sizeZ=30;
    const auto owner=add(core,box);
    const auto sketch=add(core,square(featureFace(owner,FaceRole::BoxZMax)));
    const auto extrude=add(core,ExtrudeFeatureParams{sketch,5,false,ExtrudeOperation::Join});
    require(core.rebuild() && near(top(core.bodyShape()),35),"joined extrusion starts on box top");
    box.sizeZ=40;
    require(core.apply({core.revision(),{EditFeatureCommand{owner,box}}}).success
        && near(top(core.bodyShape()),45),"box resize moves downstream B-Rep, not only its preview");
    Plane3d plane; std::string error;
    require(resolveSketchPlane(core.features(),featureFace(extrude,FaceRole::EndFace),plane,error)
        && near(plane.origin.z,45),"nested extrusion references follow primitive resize");
    const auto dependents=core.document().dependentsOf(owner,true);
    require(dependents.size()==2,"owner dependency includes face sketch and consuming extrusion");
    const auto body=core.bodyShape(); const auto revision=core.revision();
    require(!core.apply({revision,{RemoveFeatureCommand{owner,false}}}).success
        && core.bodyShape().IsSame(body) && core.revision()==revision,"safe deletion rolls back");
    require(!core.apply({revision,{SetFeatureSuppressedCommand{owner,true}}}).success
        && core.bodyShape().IsSame(body),"suppressing referenced owner cannot leave detached geometry");
    require(!core.apply({revision,{EditFeatureCommand{sketch,square(featureFace(owner,FaceRole::CylinderTop))}}}).success
        && core.revision()==revision,"wrong owner role fails atomically");
    require(core.apply({revision,{RemoveFeatureCommand{owner,true}}}).success
        && core.features().empty() && core.bodyShape().IsNull(),"cascade removes the dependent chain");

    ModelingCore cylinderCore;
    CylinderPrimitiveParams cylinder{{},{0,0,2},4,10};
    const auto cylinderId=add(cylinderCore,cylinder);
    const auto cylinderSketch=add(cylinderCore,square(featureFace(cylinderId,FaceRole::CylinderTop)));
    add(cylinderCore,ExtrudeFeatureParams{cylinderSketch,3,false,ExtrudeOperation::NewBody});
    require(cylinderCore.rebuild() && near(top(cylinderCore.bodyShape()),13),"cylinder top supports extrusion");
    cylinder.height=17;
    require(cylinderCore.apply({cylinderCore.revision(),{EditFeatureCommand{cylinderId,cylinder}}}).success
        && near(top(cylinderCore.bodyShape()),20),"non-unit cylinder axis preserves metric reference height");
    require(resolveSketchPlane(cylinderCore.features(),featureFace(cylinderId,FaceRole::CylinderBase),plane,error)
        && near(plane.origin.z,0) && near(plane.normal.z,-1),"base role remains fixed with outward normal");

    std::vector<Feature> decoded; QString codecError;
    require(FeatureCodec::decode(FeatureCodec::encode(cylinderCore.features()),decoded,codecError),"new face roles serialize");
    ModelingCore restored; require(restored.restoreFeatures(decoded) && near(top(restored.bodyShape()),20),"restored roles rebuild the same B-Rep");
}
void finiteMatching() {
    ModelingCore core;
    BoxPrimitiveParams a; a.sizeX=10; a.sizeY=20; a.sizeZ=30;
    const auto first=add(core,a); auto b=a; b.pose.origin.x=20; add(core,b);
    require(core.rebuild(),"separate coplanar boxes");
    std::string error;
    auto found=identifyPrimitiveFace(core.features(),{{},{-1,0,0},{0,1,0}},{0,5,5},error);
    require(found && found->ownerFeature==first && found->role==FaceRole::BoxXMin,"finite face match chooses correct owner");
    found=identifyPrimitiveFace(core.features(),{{0,0,30},{0,0,1},{1,0,0}},{5,5,30},error);
    require(found && found->ownerFeature==first,"coplanar disjoint owners distinguished by model-space pick");
    auto overlap=a; overlap.pose.origin.x=2; add(core,overlap); require(core.rebuild(),"overlapping owners");
    require(!identifyPrimitiveFace(core.features(),{{0,0,30},{0,0,1},{1,0,0}},{5,5,30},error)
        && error.find("Ambiguous")!=std::string::npos,"ambiguous owners cannot attach to arbitrary feature");
    require(!identifyPrimitiveFace(core.features(),{{0,0,30},{0,0,1},{1,0,0}},{50,5,30},error),"infinite plane alone is insufficient");
    BoxPrimitiveParams rotated{{{7,11,-3},{0,1,0},{0,0,1}},10,20,30,false};
    const Vec3 normals[6]{{0,-1,0},{0,1,0},{1,0,0},{-1,0,0},{0,0,-1},{0,0,1}};
    for (int i=0;i<6;++i) {
        Plane3d plane; require(primitiveFacePlane(rotated,static_cast<FaceRole>(i+3),plane,error),"six rotated semantic faces resolve");
        require(near(plane.normal.x,normals[i].x) && near(plane.normal.y,normals[i].y)
            && near(plane.normal.z,normals[i].z),"local axes produce expected outward normals");
    }
}
void nativePickAndSave() {
    MainWindow window; window.setAttribute(Qt::WA_DontShowOnScreen); window.show(); QApplication::processEvents();
    auto* model=window.findChild<ModelingController*>(); auto* viewer=window.findChild<VTKViewer*>();
    auto* project=window.findChild<ProjectController*>();
    reconstruction::BoxCandidate candidate; candidate.primitive.sizeX=10; candidate.primitive.sizeY=20; candidate.primitive.sizeZ=30;
    require(model->commitReconstructedBox(candidate),"create parametric model");
    const auto owner=model->features()[0].id;
    viewer->setDatumPlanesVisible(false);
    // A translated, rotated, nonuniform display placement must never be saved
    // as the CAD plane or alter drawing dimensions.
    viewer->setObjectTransform(25,-15,8,0,0,25,1.2,.9,1.1);
    vtkNew<vtkTransform> placement; placement->Translate(25,-15,8); placement->RotateZ(25); placement->Scale(1.2,.9,1.1);
    const double local[3]{5,10,30}; double world[3]; placement->TransformPoint(local,world);
    auto* renderer=viewer->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto* camera=renderer->GetActiveCamera(); camera->SetPosition(world[0],world[1],world[2]+100);
    camera->SetFocalPoint(world); camera->SetViewUp(0,1,0); camera->SetParallelProjection(1); camera->SetParallelScale(25);
    renderer->ResetCameraClippingRange(); viewer->renderNow();
    renderer->SetWorldPoint(world[0],world[1],world[2],1); renderer->WorldToDisplay();
    const auto* display=renderer->GetDisplayPoint(); auto* interactor=viewer->renderWindow()->GetInteractor();
    interactor->SetEventInformation(static_cast<int>(std::round(display[0])),static_cast<int>(std::round(display[1])));
    interactor->LeftButtonPressEvent(); interactor->LeftButtonReleaseEvent();
    QAction* newSketch=nullptr;
    for (auto* action:window.findChildren<QAction*>()) if (action->text()=="New Sketch") newSketch=action;
    require(newSketch && newSketch->isEnabled(),"native face pick enables sketch creation"); newSketch->trigger();
    const auto sketchId=model->features().back().id;
    const auto* params=model->sketchParams(sketchId);
    const auto* reference=std::get_if<FaceReference>(&params->plane);
    require(reference && reference->ownerFeature==owner && reference->role==FaceRole::BoxZMax,"real UI pick persists semantic owner");
    // Draw through the real inverse-transformed ray, rather than injecting a
    // model-space point which would bypass the display placement entirely.
    auto* palette=window.findChild<SketchToolPalette*>();
    palette->findChild<QToolButton*>("toolPoint")->click();
    const double modelVertex[3]{2,3,30}; placement->TransformPoint(modelVertex,world);
    renderer->SetWorldPoint(world[0],world[1],world[2],1); renderer->WorldToDisplay();
    display=renderer->GetDisplayPoint();
    interactor->SetEventInformation(static_cast<int>(std::round(display[0])),static_cast<int>(std::round(display[1])));
    interactor->LeftButtonPressEvent(); interactor->LeftButtonReleaseEvent();
    params=model->sketchParams(sketchId);
    require(params->entities.size()==1,"native drawing creates one model-space point");
    const auto& vertex=std::get<Point2D>(params->entities[0].geometry);
    require(std::abs(vertex.x-2)<.15 && std::abs(vertex.y-3)<.15,"nonuniform display scale does not change drawing units");
    palette->exitSketchRequested();
    require(model->editPrimitive(owner,{10,20,40},ResizeAnchor::ZMin,model->revision()),"edit primitive with dependent face sketch");
    sketchapp::PlaneFrame frame;
    require(sketchapp::resolveSketchFrame(model->features(),model->sketchParams(sketchId)->plane,frame)
        && near(frame.origin[2],40),"preview uses rebuilt semantic plane");
    QTemporaryDir temp; QString error; const auto path=temp.filePath("references.pcad");
    require(project->saveTo(path,error) && project->loadFrom(path,error),"semantic reference project saves and reopens");
    require(model->editPrimitive(owner,{10,20,45},ResizeAnchor::ZMin,model->revision())
        && sketchapp::resolveSketchFrame(model->features(),model->sketchParams(sketchId)->plane,frame)
        && near(frame.origin[2],45),"saved sketch continues following owner after reopening");
}
}
int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat()); QApplication app(argc,argv);
    try { kernelReferences(); finiteMatching(); nativePickAndSave(); std::cout<<"Semantic face reference tests passed.\n"; return 0; }
    catch (const std::exception& failure) { std::cerr<<failure.what()<<'\n'; return 1; }
}
