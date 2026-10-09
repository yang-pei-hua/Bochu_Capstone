#include "app/MainWindow.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/ProjectController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/CadReconstructionTask.h"
#include "io/FeatureCodec.h"
#include "io/StepModelLoader.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/FeatureEditorPanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/viewport/VTKViewer.h"
#include <modeling/PrimitiveEditing.h>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QTimer>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransform.h>
#include <vtkNew.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
bool near(double a,double b) { return std::abs(a-b)<1e-6; }
double volume(const TopoDS_Shape& shape) { GProp_GProps p; BRepGProp::VolumeProperties(shape,p); return p.Mass(); }
const Feature& feature(ModelingController* model,FeatureId id) {
    for (const auto& f:model->features()) if (f.id==id) return f;
    throw std::runtime_error("missing feature");
}
Vec3 boxPoint(const BoxPrimitiveParams& box,double x,double y,double z) {
    const auto axes=boxAxes(box); const double v[3]{x,y,z}; auto p=box.pose.origin;
    for (int i=0;i<3;++i) { p.x+=axes[i].x*v[i]; p.y+=axes[i].y*v[i]; p.z+=axes[i].z*v[i]; }
    return p;
}
void samePoint(Vec3 a,Vec3 b,const char* message) { require(near(a.x,b.x)&&near(a.y,b.y)&&near(a.z,b.z),message); }

void anchorGeometry() {
    BoxPrimitiveParams box{{{7,11,-3},{0,1,0},{0,0,1}},10,20,30,false};
    FeatureParams output; std::string error;
    require(adjustedPrimitive(box,{14,28,36},ResizeAnchor::Center,output,error),"center resize succeeds");
    samePoint(boxPoint(box,5,10,15),boxPoint(std::get<BoxPrimitiveParams>(output),7,14,18),"rotated box center stays fixed");
    for (int side=0;side<6;++side) {
        std::array<double,3> values{10,20,30}, point{2,3,4};
        point[side/2]=side%2 ? values[side/2] : 0;
        const auto before=boxPoint(box,point[0],point[1],point[2]);
        values[side/2]+=7;
        require(adjustedPrimitive(box,values,static_cast<ResizeAnchor>(side+1),output,error),"six face anchors accept normal resize");
        point[side/2]=side%2 ? values[side/2] : 0;
        samePoint(before,boxPoint(std::get<BoxPrimitiveParams>(output),point[0],point[1],point[2]),"fixed face point stays fixed in rotated coordinates");
    }
    require(!adjustedPrimitive(box,{11,22,30},ResizeAnchor::XMin,output,error),"fixed face rejects tangential changes");
    CylinderPrimitiveParams cylinder{{1,2,3},{0,2,0},4,10};
    require(adjustedPrimitive(cylinder,{5,20,1},ResizeAnchor::Top,output,error),"cylinder top resize");
    samePoint(std::get<CylinderPrimitiveParams>(output).baseCenter,{1,-8,3},"top anchor preserves cylinder endpoint");
    require(adjustedPrimitive(cylinder,{5,20,1},ResizeAnchor::Center,output,error),"cylinder center resize");
    samePoint(std::get<CylinderPrimitiveParams>(output).baseCenter,{1,-3,3},"center anchor preserves cylinder center");
}

void pick(VTKViewer* viewer,const double point[3]) {
    auto* renderer=viewer->renderWindow()->GetRenderers()->GetFirstRenderer();
    renderer->SetWorldPoint(point[0],point[1],point[2],1); renderer->WorldToDisplay();
    const auto* p=renderer->GetDisplayPoint(); auto* interactor=viewer->renderWindow()->GetInteractor();
    interactor->SetEventInformation(static_cast<int>(std::round(p[0])),static_cast<int>(std::round(p[1])));
    interactor->LeftButtonPressEvent(); interactor->LeftButtonReleaseEvent();
}

void referencePicking() {
    MainWindow window; window.setAttribute(Qt::WA_DontShowOnScreen); window.show(); QApplication::processEvents();
    auto* model=window.findChild<ModelingController*>(); auto* viewer=window.findChild<VTKViewer*>();
    auto* properties=window.findChild<PropertyPanel*>(); auto* list=window.findChild<ModelingPanel*>();
    reconstruction::BoxCandidate candidate; candidate.primitive={{{7,11,-3},{0,1,0},{0,0,1}},10,20.000000123,30.000000456,false};
    require(model->commitReconstructedBox(candidate),"create rotated box for reference selection");
    const auto id=model->features()[0].id; list->selectFeature(id);
    auto* editor=properties->featureDetailsPanel()->editor();
    viewer->setObjectTransform(25,-15,8,12,7,25,1.2,0.9,1.1);
    auto* renderer=viewer->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto* actors=renderer->GetActors(); actors->InitTraversal();
    vtkNew<vtkTransform> placement; placement->SetMatrix(actors->GetNextActor()->GetMatrix());
    const auto box=candidate.primitive; const auto axes=boxAxes(box); const auto center=boxPoint(box,5,10,15);
    const double centerInput[3]{center.x,center.y,center.z}, normalInput[3]{axes[0].x,axes[0].y,axes[0].z};
    double worldCenter[3],worldNormal[3]; placement->TransformPoint(centerInput,worldCenter); placement->TransformNormal(normalInput,worldNormal);
    properties->cameraPanel()->setOriginLock(false); CameraParameters camera;
    for (int i=0;i<3;++i) { camera.target[i]=worldCenter[i]; camera.position[i]=worldCenter[i]+100*worldNormal[i]; }
    const double upInput[3]{axes[2].x,axes[2].y,axes[2].z}; placement->TransformVector(upInput,camera.up);
    camera.parallelProjection=true; viewer->applyCamera(camera); viewer->setParallelScale(35);
    editor->findChild<QPushButton*>("chooseFeatureReferenceButton")->click();
    require(viewer->boxReferenceSelectionActive(),"reference button starts dedicated picking");
    const auto preview=qEnvironmentVariable("CAD_PARAMETER_PREVIEW");
    if (!preview.isEmpty()) require(window.grab().save(preview),"save parameter editor preview");
    const auto facePoint=boxPoint(box,10,6,9); const double input[3]{facePoint.x,facePoint.y,facePoint.z}; double worldPoint[3];
    placement->TransformPoint(input,worldPoint); pick(viewer,worldPoint);
    auto* anchor=editor->findChild<QComboBox*>("featureResizeAnchor");
    require(!viewer->boxReferenceSelectionActive() && anchor->currentData().toInt()==static_cast<int>(ResizeAnchor::XMax),
            "transformed viewport click resolves semantic box face");
    auto* x=editor->findChild<QDoubleSpinBox*>("featureDimension0");
    require(x->isEnabled() && !editor->findChild<QDoubleSpinBox*>("featureDimension1")->isEnabled(),"face editor permits only normal dimension");
    const auto fixed=boxPoint(box,10,6,9); x->setValue(15); editor->findChild<QPushButton*>("applyFeatureParametersButton")->click();
    const auto resized=std::get<BoxPrimitiveParams>(feature(model,id).params);
    require(resized.sizeY==box.sizeY && resized.sizeZ==box.sizeZ,"disabled dimensions retain full reconstructed precision");
    samePoint(fixed,boxPoint(resized,15,6,9),"UI applies resize without moving selected face");
    editor->findChild<QPushButton*>("chooseFeatureReferenceButton")->click();
    const auto newCenter=boxPoint(resized,7.5,10,15); const double newCenterInput[3]{newCenter.x,newCenter.y,newCenter.z};
    placement->TransformPoint(newCenterInput,worldPoint); pick(viewer,worldPoint);
    require(anchor->currentData().toInt()==0 && !viewer->boxReferenceSelectionActive(),"yellow center marker is selectable through faces");
    editor->findChild<QPushButton*>("chooseFeatureReferenceButton")->click();
    auto* interactor=viewer->renderWindow()->GetInteractor(); interactor->SetKeyEventInformation(0,0,0,0,"Escape"); interactor->KeyPressEvent();
    require(!viewer->boxReferenceSelectionActive(),"Esc exits reference mode");
}
}

int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat()); QApplication app(argc,argv);
    try {
        anchorGeometry(); referencePicking(); QTemporaryDir temp; require(temp.isValid(),"test directory");
        MainWindow window; window.setAttribute(Qt::WA_DontShowOnScreen); window.show(); app.processEvents();
        auto* model=window.findChild<ModelingController*>(); auto* viewer=window.findChild<VTKViewer*>();
        auto* properties=window.findChild<PropertyPanel*>(); auto* list=window.findChild<ModelingPanel*>();
        auto* editor=properties->featureDetailsPanel()->editor(); auto* project=window.findChild<ProjectController*>();
        auto* modelView=window.findChild<ModelViewController*>();
        reconstruction::BoxWithThroughHolesCandidate candidate;
        candidate.box.primitive.sizeX=10; candidate.box.primitive.sizeY=20; candidate.box.primitive.sizeZ=30;
        reconstruction::ThroughHoleCandidate hole; hole.primitive={{5,10,0},{0,0,1},2}; candidate.throughHoles.push_back(hole);
        require(model->commitReconstructedBoxWithThroughHoles(candidate),"reconstruction publishes editable box plus through hole");
        const auto boxId=model->features()[0].id, holeId=model->features()[1].id;
        list->selectFeature(holeId); auto* first=editor->findChild<QDoubleSpinBox*>("featureDimension0");
        auto* apply=editor->findChild<QPushButton*>("applyFeatureParametersButton");
        require(editor->isVisible() && near(first->value(),4),"selected hole opens diameter editor");
        const double originalVolume=volume(model->bodyShape()); const auto originalRevision=model->revision();
        first->setValue(8); require(model->revision()==originalRevision,"typing does not modify model"); apply->click();
        const double editedVolume=volume(model->bodyShape());
        require(near(std::get<ThroughHolePrimitiveParams>(feature(model,holeId).params).radius,4)
            && near(originalVolume-editedVolume,3.14159265358979323846*12*30) && model->hasProtectedReconstruction(),
            "UI diameter edit changes actual solid and protects hand edits");
        auto* undo=window.findChild<QAction*>("undoCadAction"); auto* redo=window.findChild<QAction*>("redoCadAction");
        require(undo->isEnabled(),"CAD undo action enabled"); undo->trigger();
        require(near(volume(model->bodyShape()),originalVolume) && !model->hasProtectedReconstruction() && redo->isEnabled(),"undo restores parameters body and manual marker");
        const auto afterUndo=model->revision();
        require(!model->editPrimitive(holeId,{-1,1,1},ResizeAnchor::Center,afterUndo) && model->canRedo(),"failed edit preserves redo history");
        redo->trigger(); require(near(volume(model->bodyShape()),editedVolume) && model->revision()>afterUndo,"redo restores edit with newer revision");
        first->setValue(9); editor->findChild<QPushButton*>("cancelFeatureParametersButton")->click();
        require(near(first->value(),8) && near(volume(model->bodyShape()),editedVolume),"cancel discards pending input");
        const auto beforeFailure=model->revision(); const auto bodyBefore=model->bodyShape();
        require(!model->editPrimitive(boxId,{10,1,30},ResizeAnchor::YMin,beforeFailure)
            && model->revision()==beforeFailure && model->bodyShape().IsSame(bodyBefore)
            && near(std::get<BoxPrimitiveParams>(feature(model,boxId).params).sizeY,20),"failed dependent hole rebuild rolls parameters and body back together");
        require(!model->editPrimitive(holeId,{6,1,1},ResizeAnchor::Center,originalRevision),"stale revision cannot commit");

        list->selectFeature(boxId); auto* anchor=editor->findChild<QComboBox*>("featureResizeAnchor");
        anchor->setCurrentIndex(anchor->findData(static_cast<int>(ResizeAnchor::YMin)));
        editor->findChild<QDoubleSpinBox*>("featureDimension1")->setValue(25); apply->click();
        require(feature(model,boxId).resizeAnchor==ResizeAnchor::YMin && near(std::get<BoxPrimitiveParams>(feature(model,boxId).params).pose.origin.y,0),"box anchor and dimensions committed together");
        const auto savedVolume=volume(model->bodyShape()); QString error; const auto path=temp.filePath("edited.pcad");
        require(project->saveTo(path,error) && project->loadFrom(path,error),"edited project saves and reopens");
        require(near(volume(model->bodyShape()),savedVolume) && feature(model,boxId).resizeAnchor==ResizeAnchor::YMin
            && model->hasProtectedReconstruction() && !model->canUndo() && !model->canRedo(),"project retains dimensions anchors and protection with fresh session history");
        list->selectFeature(holeId);
        require(apply->isEnabled(),"reopened parametric body remains editable");
        first->setValue(6); apply->click();
        require(near(std::get<ThroughHolePrimitiveParams>(feature(model,holeId).params).radius,3),"continue UI editing after project reopen");
        undo->trigger(); require(near(volume(model->bodyShape()),savedVolume),"reopened model has independent new undo history");
        const auto step=temp.filePath("edited.step"); require(modelView->exportStepTo(step,error),"export edited STEP");
        StepModelLoader loader; LoadedModel loaded; require(loader.load(step,loaded,error) && near(volume(loaded.shape),savedVolume),"STEP contains edited CAD geometry");
        reconstruction::CompoundCandidate replacement; replacement.boxes.push_back(candidate.box); replacement.throughHoles=candidate.throughHoles;
        const auto protectedRevision=model->revision();
        require(!model->commitReconstructedCompound(replacement) && model->revision()==protectedRevision,"default replacement cannot overwrite manual changes");
        require(model->commitReconstructedCompound(replacement,true) && !model->hasProtectedReconstruction(),"explicit replacement can refit manually edited features");
        require(model->undo() && model->hasProtectedReconstruction() && near(volume(model->bodyShape()),savedVolume),"undo replacement restores manual edits and protection");

        auto legacy=FeatureCodec::encode(model->features());
        for (qsizetype i=0;i<legacy.size();++i) { auto f=legacy[i].toObject(); f.remove("resizeAnchor"); f.remove("manuallyEdited"); legacy[i]=f; }
        std::vector<Feature> decoded; require(FeatureCodec::decode(legacy,decoded,error) && !decoded[0].manuallyEdited,"legacy feature data remains readable");
        const auto cloudPath=temp.filePath("cloud.ply"); QFile source(cloudPath); require(source.open(QIODevice::WriteOnly),"cloud fixture");
        source.write("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n"); source.close();
        auto* cloud=viewer->pointCloudDocument(); require(cloud->load(cloudPath,error),"load cloud for guided protection"); cloud->select({0,1,2});
        window.findChild<PointCloudRelationsPanel*>()->assignRequested("plane","manual test",0);
        require(!window.findChild<CadReconstructionTask*>()->isRunning() && near(volume(model->bodyShape()),savedVolume),"template edit cannot auto overwrite manual model");
        bool asked=false; QTimer::singleShot(0,&window,[&] {
            auto* dialog=window.findChild<QMessageBox*>(); if (dialog && dialog->windowTitle().contains("替换手动修改")) { asked=true; dialog->done(QMessageBox::No); }
        });
        window.findChild<PointCloudWorkspaceController*>()->reconstructCad(); require(asked && model->hasProtectedReconstruction(),"explicit reconstruction asks before replacing protected model");

        reconstruction::CompoundCandidate curved;
        reconstruction::CurvedSolidCandidate solid;
        solid.primitive=CylinderPrimitiveParams{{1,2,3},{0,1,0},4,10}; curved.curvedSolids.push_back(solid);
        require(model->commitReconstructedCompound(curved,true),"create editable cylinder"); const auto cylinderId=model->features()[0].id;
        list->selectFeature(cylinderId);
        anchor->setCurrentIndex(anchor->findData(static_cast<int>(ResizeAnchor::Top)));
        first->setValue(5); editor->findChild<QDoubleSpinBox*>("featureDimension1")->setValue(20); apply->click();
        require(near(volume(model->bodyShape()),3.14159265358979323846*25*20),"cylinder UI radius and height affect solid");
        samePoint(std::get<CylinderPrimitiveParams>(feature(model,cylinderId).params).baseCenter,{1,-8,3},"cylinder UI preserves selected top endpoint");
        std::cout << "Parameter editing tests passed.\n"; return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
