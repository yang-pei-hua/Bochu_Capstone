#include "PointCloudCadValidation.h"
#include "AssemblyDiagnostics.h"
#include "app/CadReconstructionTask.h"
#include "io/PointCloudAdapter.h"
#include "io/ProjectStore.h"
#include "io/FeatureCodec.h"
#include <reconstruction/modeling/ModelingAdapter.h>
#include <reconstruction/recognition/ProjectionExtrusion.h>
#include <modeling/StepExporter.h>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <QEventLoop>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
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
void exportCad(const modeling::ModelingCore& core, const QString& path, QJsonObject& report);
}
void validatePointCloudCad(const QStringList& args,QJsonObject& report) {
    require(args.size()==5 || (args.size()==6 && args[5]=="--projection-fallback"),
            "Usage: reconstruct-cloud cloud.ply report.json output.step [--projection-fallback]");
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
    require(task.start(cloud,unit,{},0,{},{},args.size()==6),"Cannot start point-cloud worker");
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
    stage("exporting_cad");exportCad(core,args[4],report);
    if(result->assembly && (report["assembly"].toObject()["unreconstructed_parts"].toInteger()>0 ||
                           !result->assembly->unassignedPointIds.empty())) {
        report["status"]="cad_exported_partial";report["partial_reconstruction"]=true;
    }
}

namespace {
void exportCad(const modeling::ModelingCore& core, const QString& path, QJsonObject& report) {
    QString error;
    report["solids"]=solids(core.bodyShape());report["brep_valid"]=true;
    std::string failure;require(modeling::exportStep(core.bodyShape(),std::filesystem::path(path.toStdWString()),&failure),failure);
    STEPControl_Reader reader;std::ifstream input(std::filesystem::path(path.toStdWString()),std::ios::binary);
    require(input.good() && reader.ReadStream("exported.step",input)==IFSelect_RetDone && reader.TransferRoots()>0,"Cannot read exported CAD");
    require(solids(reader.OneShape())==solids(core.bodyShape()) && BRepCheck_Analyzer(reader.OneShape()).IsValid(),"Exported CAD round trip lost valid solids");
    report["step_round_trip_verified"]=true;
    const auto info=QFileInfo(path);const auto projectPath=info.dir().filePath(info.completeBaseName()+".pcad");
    const auto featuresPath=info.dir().filePath(info.completeBaseName()+".features.json");
    QSaveFile definitions(featuresPath);require(definitions.open(QIODevice::WriteOnly),"Cannot save feature parameters");
    const auto data=QJsonDocument(FeatureCodec::encode(core.features())).toJson();require(definitions.write(data)==data.size() && definitions.commit(),"Cannot commit feature parameters");
    ProjectSnapshot snapshot;snapshot.features=core.features();snapshot.currentBody=core.bodyShape();snapshot.parametricBody=true;snapshot.view.originLock=false;
    Bnd_Box box;BRepBndLib::AddOptimal(core.bodyShape(),box,false,false);double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    const double span=std::hypot(x1-x0,y1-y0,z1-z0);
    for(int i=0;i<3;++i)snapshot.view.camera.target[i]=(i==0?x0+x1:i==1?y0+y1:z0+z1)*.5;
    snapshot.view.camera.position[0]=snapshot.view.camera.target[0]+span;snapshot.view.camera.position[1]=snapshot.view.camera.target[1]-span;
    snapshot.view.camera.position[2]=snapshot.view.camera.target[2]+span;
    require(ProjectStore::save(projectPath,snapshot,error),error.toStdString());
    const auto restored=ProjectStore::load(projectPath,error);require(restored && restored->model.features().size()==core.features().size(),error.toStdString());
    report["project_round_trip_verified"]=true;report["project"]=projectPath;report["features_file"]=featuresPath;
    report["step"]=QFileInfo(path).absoluteFilePath();report["cad_constructed"]=true;report["status"]="cad_exported";
}

QJsonArray pointIds(const std::vector<reconstruction::PointId>& ids) {
    QJsonArray result;
    for (auto id : ids) result.append(qint64(id));
    return result;
}
}

void validateProjectionCad(const QStringList& args, QJsonObject& report) {
    require(args.size()>=5 && args.size()<=7,
        "Usage: fit-projection cloud.ply report.json output.step [tolerance-mm] [--area-directions]");
    reconstruction::ProjectionExtrusionOptions options;
    for (int index=5; index<args.size(); ++index) {
        if (args[index]=="--area-directions") {
            require(!options.includeAreaDirections, "Duplicate area-directions option");
            options.includeAreaDirections=true;
        } else {
            bool parsed=false;
            const double tolerance=args[index].toDouble(&parsed);
            require(parsed && std::isfinite(tolerance) && tolerance>0 && options.distanceTolerance==0,
                    "Expected a positive tolerance in millimetres");
            options.distanceTolerance=tolerance;
        }
    }
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    const auto scale=reconstruction::millimetersPerUnit(unit);
    require(scale.has_value(),"Point cloud needs mm/m unit metadata; declare its unit explicitly");
    PlyCloud cloud;
    QString readError;
    require(PlyReader::read(args[2],cloud,readError),readError.toStdString());
    auto points=PointCloudAdapter::toPointStore(cloud,unit).points();
    for (auto& point : points) {
        point.position.x *= *scale;
        point.position.y *= *scale;
        point.position.z *= *scale;
    }
    report["input_kind"]="point_cloud_only";
    report["source_model_read"]=false;
    report["reference_labels_used"]=false;
    report["point_id_basis"]="one_based_row_in_input_ply";
    report["cloud"]=QFileInfo(args[2]).absoluteFilePath();
    report["input_points"]=qint64(points.size());
    report["area_directions_enabled"]=options.includeAreaDirections;
    report["cad_constructed"]=false;
    reconstruction::ProjectionExtrusionResult fit;
    const bool accepted=reconstruction::fitProjectionExtrusion(
        reconstruction::PointStore(std::move(points)),options,fit);
    QJsonArray axes,operations;
    for (const auto& axis : fit.axes) {
        axes.append(QJsonObject{{"direction",QJsonArray{axis.axis.x,axis.axis.y,axis.axis.z}},
            {"source",QString::fromStdString(axis.source)},{"convex_projection_area_mm2",axis.convexProjectionArea},
            {"convex_envelope_volume_mm3",axis.convexEnvelopeVolume},{"status",QString::fromStdString(axis.status)}});
    }
    for (const auto& operation : fit.operations) {
        operations.append(QJsonObject{{"operation",int(operation.operation)},
            {"depth_mm",operation.depth},{"reverse",operation.reverse},
            {"curve_count",qint64(operation.sketch.entities.size())},{"support_point_ids",pointIds(operation.supportPointIds)}});
    }
    report["axes"]=axes;
    report["operations"]=operations;
    report["distance_tolerance_mm"]=fit.distanceTolerance;
    report["quality"]=QJsonObject{{"scored_points",qint64(fit.quality.scoredPoints)},
        {"median_distance_mm",fit.quality.medianDistance},
        {"p95_distance_mm",fit.quality.p95Distance},{"surface_inlier_fraction",fit.quality.surfaceInlierFraction},
        {"unsupported_face_area_fraction",fit.quality.unsupportedFaceAreaFraction},
        {"mean_normal_error_degrees",fit.quality.meanNormalErrorDegrees},
        {"normal_comparisons",qint64(fit.quality.normalComparisons)},{"score",fit.quality.score}};
    report["sampled_residual_point_ids"]=pointIds(fit.residualPointIds);
    if (!accepted) {
        report["status"]="unsupported";
        report["error"]=QString::fromStdString(fit.reason);
        return;
    }
    modeling::ModelingCore core;
    std::string error;
    require(reconstruction::buildProjectionProgram(fit.operations,core,error),error);
    report["features"]=qint64(core.features().size());
    exportCad(core,args[4],report);
}
