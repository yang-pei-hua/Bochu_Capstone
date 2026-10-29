#include "PointCloudCadValidation.h"
#include "AssemblyDiagnostics.h"
#include "app/CadReconstructionTask.h"
#include "io/PointCloudAdapter.h"
#include "io/ProjectStore.h"
#include "io/FeatureCodec.h"
#include <reconstruction/modeling/ModelingAdapter.h>
#include <modeling/StepExporter.h>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <QEventLoop>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QSaveFile>
#include <filesystem>
#include <fstream>
#include <optional>
#include <cmath>
#include <stdexcept>

namespace {
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void save(const QString& path,const QJsonObject& report){QSaveFile file(path);require(file.open(QIODevice::WriteOnly),"Cannot save cloud-only report");
    const auto data=QJsonDocument(report).toJson();require(file.write(data)==data.size() && file.commit(),"Cannot commit cloud-only report");}
int solids(const TopoDS_Shape& shape){int count=0;for(TopExp_Explorer i(shape,TopAbs_SOLID);i.More();i.Next())++count;return count;}
}
void validatePointCloudCad(const QStringList& args,QJsonObject& report) {
    require(args.size()==5,"Usage: reconstruct-cloud cloud.ply report.json output.step");
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    require(unit!=reconstruction::LengthUnit::Arbitrary,"Point cloud needs mm/m unit metadata; declare its unit explicitly");
    report["input_kind"]="point_cloud_only";report["source_model_read"]=false;report["topology_correspondence_used"]=false;
    report["reference_labels_used"]=false;report["source_completeness_verified"]=false;report["cad_constructed"]=false;
    report["cloud"]=QFileInfo(args[2]).absoluteFilePath();report["unit"]=unit==reconstruction::LengthUnit::Millimeter?"mm":"m";
    auto stage=[&](const QString& name){report["stage"]=name;report["status"]="running";save(args[3],report);};
    stage("loading_cloud");PlyCloud cloud;QString error;require(PlyReader::read(args[2],cloud,error),error.toStdString());
    report["input_points"]=double(cloud.positions.size());
    CadReconstructionTask task;QEventLoop loop;std::optional<CadReconstructionResult> result;
    QObject::connect(&task,&CadReconstructionTask::stageChanged,&loop,[&](auto value){
        stage(value==CadReconstructionTask::Stage::Preprocessing?"preprocessing":value==CadReconstructionTask::Stage::SeparatingParts?"separating_and_closing_parts":"recognizing_from_points");});
    QObject::connect(&task,&CadReconstructionTask::finished,&loop,[&](const auto& value){result=value;loop.quit();});
    require(task.start(cloud,unit),"Cannot start point-cloud worker");
    if(!result)loop.exec();require(result.has_value(),"Point-cloud worker returned no result");
    if(result->preprocessingComplete)report["preprocessing"]=QJsonObject{{"input_points",double(result->preprocessing.inputPointCount)},
        {"output_points",double(result->preprocessing.outputPointCount)},{"removed_voxel_duplicates",double(result->preprocessing.removedByVoxel)},
        {"removed_outliers",double(result->preprocessing.removedAsOutliers)},{"estimated_normals",double(result->preprocessing.normalsEstimated)}};
    report["detected_surface_patches"]=double(result->analysis.patches.size());
    if(result->assembly){stage("writing_separation_diagnostics");report["assembly"]=writeAssemblyDiagnostics(args[3],cloud,*result->assembly,report["unit"].toString());}
    if(result->outcome!=CadReconstructionResult::Outcome::Success) {
        report["status"]=result->outcome==CadReconstructionResult::Outcome::Unsupported?"unsupported":"failed";
        report["error"]=result->error;return;
    }
    stage("building_parametric_cad");modeling::ModelingCore core;
    const auto committed=reconstruction::commitCompoundCandidate(result->candidate,core);
    require(committed.success,committed.error);require(!core.bodyShape().IsNull() && BRepCheck_Analyzer(core.bodyShape()).IsValid(),"Invalid point-cloud CAD");
    report["surface_coverage"]=result->candidate.surfaceCoverage;report["features"]=double(core.features().size());
    if(result->assembly){
        report["coverage_basis"]="fraction_of_input_observations_assigned_to_closed_candidates";
        report["assigned_observation_fraction"]=result->candidate.surfaceCoverage;
    } else report["coverage_basis"]="recognizer_surface_support";
    report["solids"]=solids(core.bodyShape());report["brep_valid"]=true;
    std::string failure;stage("exporting_cad");require(modeling::exportStep(core.bodyShape(),std::filesystem::path(args[4].toStdWString()),&failure),failure);
    STEPControl_Reader reader;std::ifstream input(std::filesystem::path(args[4].toStdWString()),std::ios::binary);
    require(input.good() && reader.ReadStream("exported.step",input)==IFSelect_RetDone && reader.TransferRoots()>0,"Cannot read exported CAD");
    require(solids(reader.OneShape())==solids(core.bodyShape()) && BRepCheck_Analyzer(reader.OneShape()).IsValid(),"Exported CAD round trip lost valid solids");
    report["step_round_trip_verified"]=true;
    const auto info=QFileInfo(args[4]);const auto projectPath=info.dir().filePath(info.completeBaseName()+".pcad");
    const auto featuresPath=info.dir().filePath(info.completeBaseName()+".features.json");
    QSaveFile definitions(featuresPath);require(definitions.open(QIODevice::WriteOnly),"Cannot save feature parameters");
    const auto data=QJsonDocument(FeatureCodec::encode(core.features())).toJson();require(definitions.write(data)==data.size() && definitions.commit(),"Cannot commit feature parameters");
    ProjectSnapshot snapshot;snapshot.features=core.features();snapshot.currentBody=core.bodyShape();snapshot.parametricBody=true;snapshot.view.originLock=false;
    Bnd_Box box;BRepBndLib::AddOptimal(core.bodyShape(),box,false,false);double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    const double span=std::hypot(x1-x0,y1-y0,z1-z0);
    for(int i=0;i<3;++i)snapshot.view.camera.target[i]=(i==0?x0+x1:i==1?y0+y1:z0+z1)*.5;
    snapshot.view.camera.position[0]=snapshot.view.camera.target[0]+span;snapshot.view.camera.position[1]=snapshot.view.camera.target[1]-span;
    snapshot.view.camera.position[2]=snapshot.view.camera.target[2]+span;
    stage("saving_project");require(ProjectStore::save(projectPath,snapshot,error),error.toStdString());
    const auto restored=ProjectStore::load(projectPath,error);require(restored && restored->model.features().size()==core.features().size(),error.toStdString());
    report["project_round_trip_verified"]=true;report["project"]=projectPath;report["features_file"]=featuresPath;
    report["step"]=QFileInfo(args[4]).absoluteFilePath();report["cad_constructed"]=true;report["status"]="cad_exported";
    if(result->assembly && (report["assembly"].toObject()["open_parts"].toInteger()>0 || !result->assembly->unassignedPointIds.empty())) {
        report["status"]="cad_exported_partial";report["partial_reconstruction"]=true;
    }
}
