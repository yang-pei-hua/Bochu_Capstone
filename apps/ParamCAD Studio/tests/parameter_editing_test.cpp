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
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QDir>
#include <QFile>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QTimer>
#include <QThread>
#include <TopExp_Explorer.hxx>
#include <BRepCheck_Analyzer.hxx>
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

void cloudOnlyButtonWorkflow() {
    QTemporaryDir directory;require(directory.isValid(),"cloud-only fixture directory");
    const auto path=directory.filePath("input.ply");QFile file(path);require(file.open(QIODevice::WriteOnly),"write cloud-only fixture");
    QByteArray data("ply\nformat ascii 1.0\nelement vertex 600\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n");
    for(int axis=0;axis<3;++axis)for(int sign:{-1,1})for(int i=0;i<10;++i)for(int j=0;j<10;++j) {
        double p[3]{},n[3]{};p[axis]=3*sign;n[axis]=sign;p[(axis+1)%3]=-3+(i+.5)*.6;p[(axis+2)%3]=-3+(j+.5)*.6;
        data+=QByteArray::number(p[0])+' '+QByteArray::number(p[1])+' '+QByteArray::number(p[2])+' '+QByteArray::number(n[0])+' '+QByteArray::number(n[1])+' '+QByteArray::number(n[2])+'\n';
    }
    require(file.write(data)==data.size(),"save observed box points");file.close();
    auto write=[&](const QString& name,const QByteArray& bytes){QFile f(directory.filePath(name));require(f.open(QIODevice::WriteOnly)&&f.write(bytes)==bytes.size(),"write neighbor fixture");};
    write("input.json","{\"unit\":\"mm\"}");
    // These would previously activate the removed source-model branch. They
    // must have no effect, even when the source model is unavailable.
    write("summary.json","{\"schema\":\"paramcad.step-surface-validation\",\"status\":\"passed\",\"cloud\":\"input.ply\",\"source_step\":\"missing.step\"}");
    write("reference.json","invalid metadata: this file must never be read by reconstruction");
    MainWindow window;window.setAttribute(Qt::WA_DontShowOnScreen);window.show();
    auto* workspace=window.findChild<PointCloudWorkspaceController*>();auto* task=window.findChild<CadReconstructionTask*>();
    auto* model=window.findChild<ModelingController*>();workspace->loadPointCloudFromFile(path);
    auto* action=window.findChild<QAction*>("reconstructCadAction");require(action->isEnabled() && !action->text().contains("STEP"),"button exposes only point-cloud reconstruction");
    std::optional<CadReconstructionResult> result;QObject::connect(task,&CadReconstructionTask::finished,&window,[&](const auto& value){result=value;});
    action->trigger();QElapsedTimer timer;timer.start();while(task->isRunning()&&timer.elapsed()<30000){QApplication::processEvents();QThread::msleep(5);}
    require(result && result->outcome==CadReconstructionResult::Outcome::Success && !model->features().empty(),"button reconstructs using only observations despite poisoned source-model sidecars");
    require(near(volume(model->bodyShape()),216),"cloud-only result has observed dimensions");
    window.findChild<ModelingPanel*>()->selectFeature(model->features().front().id);auto* editor=window.findChild<FeatureEditorPanel*>();
    editor->findChild<QDoubleSpinBox*>("featureDimension0")->setValue(7);editor->findChild<QPushButton*>("applyFeatureParametersButton")->click();
    require(near(volume(model->bodyShape()),252)&&model->undo()&&near(volume(model->bodyShape()),216),"cloud-only CAD remains editable and undoable");
    QString error;auto* project=window.findChild<ProjectController*>();require(project->saveTo(directory.filePath("result.pcad"),error)&&project->loadFrom(directory.filePath("result.pcad"),error),"cloud-only project saves and reopens");
    require(window.findChild<ModelViewController*>()->exportStepTo(directory.filePath("result.step"),error),"STEP export remains available for reconstructed CAD");
}

void assemblyButtonWorkflow() {
    QTemporaryDir directory;require(directory.isValid(),"assembly GUI fixture directory");const auto path=directory.filePath("parts.ply");
    QByteArray rows;
    auto add=[&](double cx,double cy,double cz,double sx,double sy,double sz,int missing){const double size[]{sx,sy,sz};
        for(int axis=0;axis<3;++axis)for(int side=0;side<2;++side){if(axis*2+side==missing)continue;
            for(int i=0;i<10;++i)for(int j=0;j<10;++j){double p[3]{},n[3]{};p[axis]=(side?.5:-.5)*size[axis];n[axis]=side?1:-1;
                p[(axis+1)%3]=((i+.5)/10-.5)*size[(axis+1)%3];p[(axis+2)%3]=((j+.5)/10-.5)*size[(axis+2)%3];
                rows+=QByteArray::number(p[0]+cx)+' '+QByteArray::number(p[1]+cy)+' '+QByteArray::number(p[2]+cz)+' '+QByteArray::number(n[0])+' '+QByteArray::number(n[1])+' '+QByteArray::number(n[2])+'\n';}}};
    add(0,0,0,6,6,6,-1);add(20,0,0,4,6,8,5);
    QFile file(path);require(file.open(QIODevice::WriteOnly),"write multiple observed parts");
    file.write("ply\nformat ascii 1.0\nelement vertex 1100\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n");file.write(rows);file.close();
    QFile unit(directory.filePath("parts.json"));require(unit.open(QIODevice::WriteOnly),"assembly units");unit.write("{\"unit\":\"mm\"}");unit.close();
    MainWindow window;window.setAttribute(Qt::WA_DontShowOnScreen);window.show();auto* workspace=window.findChild<PointCloudWorkspaceController*>();
    auto* task=window.findChild<CadReconstructionTask*>();auto* model=window.findChild<ModelingController*>();auto* viewer=window.findChild<VTKViewer*>();
    workspace->loadPointCloudFromFile(path);auto* document=viewer->pointCloudDocument();document->select({0,3});document->deleteSelectedPoints();
    std::optional<CadReconstructionResult> result;QObject::connect(task,&CadReconstructionTask::finished,&window,[&](const auto& value){result=value;});
    window.findChild<QAction*>("reconstructCadAction")->trigger();QElapsedTimer elapsed;elapsed.start();while(task->isRunning()&&elapsed.elapsed()<30000){QApplication::processEvents();QThread::msleep(5);}
    require(result&&result->assembly&&result->outcome==CadReconstructionResult::Outcome::Success,"actual button separates mixed closed/open components");
    if(!near(volume(model->bodyShape()),216))std::cerr<<"mixed-component CAD volume="<<volume(model->bodyShape())<<" expected=216\n";
    require(near(volume(model->bodyShape()),216),"an incomplete neighbor is excluded from publishable CAD");
    require(document->selection().regions().size()>=2 && viewer->assemblyBoundarySegmentCount()>0,"part collections and finite boundary overlay are visible");
    bool open=false,closed=false;for(const auto& r:document->selection().regions()){if(r.geometry!="part")continue;
        if(r.parameters["closed"].toBool())closed=true;else open=true;
        for(auto id:document->selection().regionPoints(r.id))require(id!=0&&id!=3,"part support maps through noncontiguous original indices");}
    require(open&&closed,"part annotations preserve closed and unresolved states separately");
    const auto screenshot=qEnvironmentVariable("PCAD_ASSEMBLY_PREVIEW");if(!screenshot.isEmpty()){QApplication::processEvents();require(window.grab().save(screenshot),"save assembly separation preview");}
    QString error;auto* project=window.findChild<ProjectController*>();require(project->saveTo(directory.filePath("assembly.pcad"),error)&&project->loadFrom(directory.filePath("assembly.pcad"),error),"part annotations and editable CAD survive project round trip");
    require(std::count_if(document->selection().regions().begin(),document->selection().regions().end(),[](const auto& r){return r.geometry=="part";})>=2,"saved part candidates retain their semantic type");
}

// Optional end-to-end acceptance against any user-supplied PLY. No source CAD,
// expected part count or case-specific dimensions are supplied to reconstruction.
void observedAssemblyWorkflow() {
    const auto path=qEnvironmentVariable("PCAD_REAL_CLOUD");if(path.isEmpty())return;
    const auto output=qEnvironmentVariable("PCAD_REAL_GUI_OUTPUT");require(!output.isEmpty()&&QDir().mkpath(output),"real-cloud GUI output directory");
    MainWindow window;window.setAttribute(Qt::WA_DontShowOnScreen);window.show();
    auto* workspace=window.findChild<PointCloudWorkspaceController*>();auto* task=window.findChild<CadReconstructionTask*>();
    auto* model=window.findChild<ModelingController*>();auto* viewer=window.findChild<VTKViewer*>();
    workspace->loadPointCloudFromFile(path);require(viewer->pointCloudDocument()->hasCloud(),"load arbitrary real observations");
    std::optional<CadReconstructionResult> result;QObject::connect(task,&CadReconstructionTask::finished,&window,[&](const auto& value){result=value;});
    window.findChild<QAction*>("reconstructCadAction")->trigger();QElapsedTimer timer;timer.start();
    while(task->isRunning()&&timer.elapsed()<180000){QApplication::processEvents();QThread::msleep(5);}
    require(result&&result->assembly,"real button produces separation diagnostics");
    const auto closed=std::count_if(result->assembly->parts.begin(),result->assembly->parts.end(),[](const auto& part){return part.closed;});
    auto* document=viewer->pointCloudDocument();const auto regions=document->selection().regions().size();
    require(regions>=result->assembly->parts.size()&&viewer->assemblyBoundarySegmentCount()>0,"real part groups and finite boundaries reach the viewport");
    std::cerr<<"real-cloud candidates="<<result->assembly->parts.size()<<" closed="<<closed<<" boundary_segments="<<viewer->assemblyBoundarySegmentCount()<<'\n';
    QApplication::processEvents();require(window.grab().save(QDir(output).filePath("separation.png")),"save real separation preview");
    if(closed){
        require(result->outcome==CadReconstructionResult::Outcome::Success&&model->features().size()==size_t(closed),"only verified real parts are published");
        require(BRepCheck_Analyzer(model->bodyShape()).IsValid(),"real published CAD is valid");
        auto* list=window.findChild<ModelingPanel*>();list->selectFeature(model->features().front().id);QApplication::processEvents();
        auto* editor=window.findChild<FeatureEditorPanel*>();auto* dimension=editor->findChild<QDoubleSpinBox*>("featureDimension0");
        const double before=volume(model->bodyShape());dimension->setValue(dimension->value()*1.05);editor->findChild<QPushButton*>("applyFeatureParametersButton")->click();
        require(std::abs(volume(model->bodyShape())-before)>std::abs(before)*1e-8,"actual real CAD parameter edit changes material");
        require(model->undo()&&std::abs(volume(model->bodyShape())-before)<std::max(1e-6,std::abs(before)*1e-8),"real CAD edit can be undone");
        QString error;require(window.findChild<ModelViewController*>()->exportStepTo(QDir(output).filePath("observed.step"),error),"real GUI exports reconstructed STEP");
    } else require(result->outcome==CadReconstructionResult::Outcome::Unsupported&&model->features().empty(),"unresolved real observations produce diagnostics without a false solid");
    QString error;auto* project=window.findChild<ProjectController*>();const auto projectPath=QDir(output).filePath("observed.pcad");
    require(project->saveTo(projectPath,error)&&project->loadFrom(projectPath,error),"real observations, candidate membership and CAD save and reopen");
    require(document->selection().regions().size()==regions,"reopened real candidates preserve all memberships");
}

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
void profileBodyEditing() {
    MainWindow window;window.setAttribute(Qt::WA_DontShowOnScreen);window.show();QApplication::processEvents();
    auto* model=window.findChild<ModelingController*>();auto* list=window.findChild<ModelingPanel*>();
    auto* editor=window.findChild<PropertyPanel*>()->featureDetailsPanel()->editor();
    auto* project=window.findChild<ProjectController*>();
    ProfileBodyParams body;body.depth=body.sizeZ=8;body.referenceSizeX=body.referenceSizeY=body.sizeX=body.sizeY=20;
    body.envelope={{0,0,0,8,10,10}};body.cavities={{0,0,0,8,3,3},{0,0,6,8,8,8,6,6}};
    ModelingCore prepared;require(prepared.restoreFeatures({{1,"ring",FeatureType::ProfileBody,body,false,true,{},true},
        {2,"other",FeatureType::BoxPrimitive,BoxPrimitiveParams{{{40,0,0}},5,5,5,false}}}),"profile editor fixture");
    model->adoptProject(std::move(prepared));list->selectFeature(1);QApplication::processEvents();
    auto* target=editor->findChild<QComboBox*>("profileBodyEditTarget");auto* apply=editor->findChild<QPushButton*>("applyFeatureParametersButton");
    auto* first=editor->findChild<QDoubleSpinBox*>("featureDimension0");auto* second=editor->findChild<QDoubleSpinBox*>("featureDimension1");
    require(editor->isVisible()&&target->isVisible()&&target->findData(3)>=0,"profile editor exposes an annular inner boundary");
    const double original=volume(model->bodyShape());
    target->setCurrentIndex(target->findData(0));first->setValue(8);second->setValue(8);apply->click();
    require(near(original-volume(model->bodyShape()),56*std::acos(-1.0))&&std::get<ProfileBodyParams>(feature(model,1).params).cavities[0].startRadius==4,
        "UI hole edit changes real material");
    require(model->undo()&&near(volume(model->bodyShape()),original),"profile edit undo restores solid");
    require(model->redo(),"profile edit redo");
    target->setCurrentIndex(target->findData(3));first->setValue(13);second->setValue(13);apply->click();
    require(near(std::get<ProfileBodyParams>(feature(model,1).params).cavities[1].innerStartRadius,6.5),"UI edits annular inner diameter");
    QTemporaryDir temp;QString error;const auto path=temp.filePath("profile-edit.pcad");
    require(project->saveTo(path,error)&&project->loadFrom(path,error),"edited profile project reopens");
    list->selectFeature(1);target->setCurrentIndex(target->findData(-2));first->setValue(22);second->setValue(22);apply->click();
    require(near(std::get<ProfileBodyParams>(feature(model,1).params).envelope[0].startRadius,11),"reopened profile retains an editable envelope");
    target->setCurrentIndex(target->findData(-1));first->setValue(24);apply->click();
    require(near(std::get<ProfileBodyParams>(feature(model,1).params).sizeX,24),"overall profile dimension editor remains functional");
    TopoDS_Shape resized;std::string failure;require(buildProfileBody(std::get<ProfileBodyParams>(feature(model,1).params),resized,failure),"build edited profile for independent dimensions");
    Bnd_Box box;BRepBndLib::AddOptimal(resized,box,false,false);double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    require(near(x1-x0,24),"overall dimensions stay truthful after an envelope edit");
    require(project->saveTo(path,error)&&project->loadFrom(path,error),"nonuniform profile scaling survives project round trip");
}
}

int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat()); QApplication app(argc,argv);
    try {
        cloudOnlyButtonWorkflow();
        assemblyButtonWorkflow();
        observedAssemblyWorkflow();
        anchorGeometry(); referencePicking();profileBodyEditing(); QTemporaryDir temp; require(temp.isValid(),"test directory");
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
