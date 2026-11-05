#include "SurfaceCoverageValidation.h"
#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"
#include "io/PointCloudAnalysis.h"
#include <reconstruction/analysis/SurfaceAnalysis.h>
#include <reconstruction/detection/MultiScaleDetector.h>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <stdexcept>
#include <unordered_set>
#include <iostream>
#include <numeric>
#include <QFileInfo>
#include <QDir>
#include <filesystem>
#include <fstream>
#include <iomanip>

void validateSurfaceCoverage(const QStringList& args,QJsonObject& report) {
    if(args.size()!=6)throw std::runtime_error("Usage: analyze cloud.ply report.json reference.json tolerance_mm");
    bool ok=false;const double tolerance=args[5].toDouble(&ok);
    if(!ok || tolerance<=0)throw std::runtime_error("Invalid tolerance");
    if(PointCloudAdapter::lengthUnitFor(args[2])!=reconstruction::LengthUnit::Millimeter)
        throw std::runtime_error("Surface validation requires a millimeter cloud sidecar");
    PlyCloud ply;QString error;
    if(!PlyReader::read(args[2],ply,error))throw std::runtime_error(error.toStdString());
    const auto points=PointCloudAdapter::toPointStore(ply,reconstruction::LengthUnit::Millimeter);
    std::vector<reconstruction::PrimitiveEvidence> surfaces;
    std::array<std::unordered_set<reconstruction::PointId>,5> clientAssigned;
    std::vector<int> labels(ply.positions.size()),labelKinds(ply.positions.size(),-1);
    std::vector<double> scores(ply.positions.size(),INFINITY);
    QJsonArray displayRegions;
    const bool client=args[1]=="analyze-client";
    if(client) {
        std::vector<vtkIdType> ids(ply.positions.size());std::iota(ids.begin(),ids.end(),1);
        const auto result=analysePointCloud(ply,ids,tolerance,{},reconstruction::LengthUnit::Millimeter);
        if(!result.error.isEmpty())throw std::runtime_error(result.error.toStdString());
        surfaces=result.analysis.surfaces;report["processed_points"]=double(result.processedPoints);
        report["assigned_points"]=double(result.assignedPoints);
        report["detection_passes"]=double(result.analysis.detectionPasses);
        const char* kinds[]={"plane","cylinder","sphere","cone","ring"};
        int region=0;
        for(const auto& s:result.surfaces) {
            ++region;
            int kind=0;while(kind<5 && s.surface.geometry!=kinds[kind])++kind;
            if(kind<5)for(auto id:s.points) {
                clientAssigned[kind].insert(reconstruction::PointId(id));
                labels.at(std::size_t(id-1))=region;labelKinds.at(std::size_t(id-1))=kind;
            }
            displayRegions.append(QJsonObject{{"id",region},{"kind",s.surface.geometry},
                {"parameters",s.surface.parameters},{"points",double(s.points.size())},
                {"mean_residual_mm",s.surface.residual}});
        }
    } else {
        reconstruction::PrimitiveDetectionOptions o;
        o.detectPlanes=o.detectCylinders=o.detectCones=true;
        o.detectSpheres=o.detectTori=false;
        o.plane.distanceThreshold=o.cylinder.distanceThreshold=o.cone.distanceThreshold=tolerance;
        o.plane.minimumNormalAlignment=o.cylinder.minimumNormalAlignment=o.cone.minimumNormalAlignment=0.999;
        o.plane.minimumSupportPoints=o.cylinder.minimumSupportPoints=o.cone.minimumSupportPoints=80;
        o.plane.maximumPlanes=o.cylinder.maximumCylinders=o.cone.maximumCones=2048;
        o.plane.probability=o.cylinder.probability=o.cone.probability=0.001;
        o.plane.duplicatePlaneDistanceMultiplier=1;
        bool seedOk=true;const auto seedText=qEnvironmentVariable("PCAD_RANSAC_SEED");
        if(!seedText.isEmpty())o.randomSeed=seedText.toUInt(&seedOk);
        if(!seedOk)throw std::runtime_error("Invalid PCAD_RANSAC_SEED");
        report["ransac_seed"]=double(o.randomSeed);
        reconstruction::PrimitiveDetectionResult result;std::string text;
        reconstruction::MultiScaleDetectionReport detail;
        std::cout<<"Multiscale detection on "<<points.size()<<" points"<<std::endl;
        if(reconstruction::detectMultiScalePrimitiveEvidence(points,o,result,detail,text))
            surfaces.insert(surfaces.end(),result.evidence.begin(),result.evidence.end());
        report["detection_passes"]=double(detail.detectionPasses);report["local_regions"]=double(detail.localRegions);
        report["recovered_surfaces"]=double(detail.recoveredSurfaces);
        std::cout<<"Surfaces total "<<surfaces.size()<<std::endl;
    }
    // Load the independent oracle only AFTER detection. Never pass it to fitting.
    QFile file(args[4]);if(!file.open(QIODevice::ReadOnly))throw std::runtime_error("Cannot read reference inventory");
    const auto reference=QJsonDocument::fromJson(file.readAll()).object();
    if(reference["points"].toInt()!=int(points.size()))throw std::runtime_error("Reference point count mismatch");
    if(reference["faces"].toArray().isEmpty() || reference["solids"].toArray().isEmpty())
        throw std::runtime_error("Empty reference inventory");
    QJsonArray predictions;std::array<std::unordered_set<reconstruction::PointId>,5> assigned;
    const char* names[]={"plane","cylinder","sphere","cone","torus"};
    int surfaceId=0;
    for(const auto& surface:surfaces)std::visit([&](const auto& s) {
        ++surfaceId;
        const int kind=int(reconstruction::primitiveKind(surface));QJsonArray ids;
        for(auto id:s.supportPointIds){assigned[kind].insert(id);ids.append(double(id));
            if(!client) {
                const auto index=std::size_t(id-1);const auto& p=ply.positions.at(index);const auto& n=ply.normals.at(index);
                const modeling::Vec3 point{p[0],p[1],p[2]},normal{n[0],n[1],n[2]};
                const double score=reconstruction::surfaceResidual(surface,point)+tolerance*(1-reconstruction::surfaceNormalAlignment(surface,point,normal));
                if(score<scores[index]){scores[index]=score;labels[index]=surfaceId;labelKinds[index]=kind;}
            }
        }
        predictions.append(QJsonObject{{"id",surfaceId},{"kind",names[kind]},{"support_point_ids",ids},
            {"mean_residual_mm",s.meanAbsoluteResidual},{"max_residual_mm",s.maxAbsoluteResidual}});
    },surface);
    if(client)assigned=std::move(clientAssigned);
    QJsonArray faces,solids;int completeFaces=0;std::size_t totalAssigned=0;
    std::array<std::unordered_set<reconstruction::PointId>,5> displayed;
    for(std::size_t i=0;i<labelKinds.size();++i)if(labelKinds[i]>=0)displayed[labelKinds[i]].insert(i+1);
    for(auto value:reference["faces"].toArray()) {
        auto f=value.toObject();int kind=0;while(kind<5 && f["kind"]!=names[kind])++kind;
        const auto start=f["first_point_id"].toInt(),count=f["point_count"].toInt();int supported=0,labeled=0;
        if(start<1 || count<1 || start+count-1>int(points.size()))throw std::runtime_error("Invalid reference face point range");
        if(kind<5)for(int i=0;i<count;++i)supported+=assigned[kind].count(start+i)>0;
        if(kind<5)for(int i=0;i<count;++i)labeled+=displayed[kind].count(start+i)>0;
        const double coverage=double(supported)/count,displayCoverage=double(labeled)/count;
        const bool complete=coverage>=0.98 && displayCoverage>=0.98;
        f["coverage"]=coverage;f["display_coverage"]=displayCoverage;f["recognized"]=complete;faces.append(f);completeFaces+=complete;totalAssigned+=supported;
    }
    int completeSolids=0;
    for(auto value:reference["solids"].toArray()) {
        auto s=value.toObject();int count=0,recognized=0;
        for(auto f:faces)if(f.toObject()["solid_id"]==s["id"]){++count;recognized+=f.toObject()["recognized"].toBool();}
        s["recognized_faces"]=recognized;s["recognized"]=recognized==count;solids.append(s);completeSolids+=recognized==count;
    }
    report["status"]=completeFaces==faces.size()?"all_surfaces_recognized":"incomplete_surfaces";
    report["complete_cad_fit"]=false;report["tolerance_mm"]=tolerance;
    report["complete_surface_recognition"]=completeFaces==faces.size();
    report["reference_labels_used_by_recognition"]=false;
    report["classification_path"]=client?"studio_preprocessing_and_display_assignment":"multiscale_analytic_evidence";
    report["recognized_faces"]=completeFaces;report["total_faces"]=faces.size();
    report["recognized_solids"]=completeSolids;report["total_solids"]=solids.size();
    report["point_coverage"]=double(totalAssigned)/points.size();
    report["faces"]=faces;report["solids"]=solids;report["predictions"]=predictions;
    report["display_regions"]=displayRegions;
    const auto path=QFileInfo(args[3]);const auto colored=path.dir().filePath(path.completeBaseName()+".segmented.ply");
    std::ofstream out(std::filesystem::path(colored.toStdWString()));
    if(!out.good())throw std::runtime_error("Cannot create segmented PLY");
    out<<"ply\nformat ascii 1.0\ncomment units mm; colors from fitted regions\nelement vertex "<<ply.positions.size()
        <<"\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty int surface_id\nproperty int surface_kind\nend_header\n"<<std::setprecision(9);
    for(std::size_t i=0;i<ply.positions.size();++i) {
        const auto& p=ply.positions[i];const auto n=ply.normals.size()==ply.positions.size()?ply.normals[i]:std::array<float,3>{0,0,0};
        const auto color=std::uint32_t(labels[i])*2654435761U;
        const int r=labels[i]?60+int(color&255)*3/4:255,g=labels[i]?60+int((color>>8)&255)*3/4:0,b=labels[i]?60+int((color>>16)&255)*3/4:0;
        out<<p[0]<<' '<<p[1]<<' '<<p[2]<<' '<<n[0]<<' '<<n[1]<<' '<<n[2]<<' '<<r<<' '<<g<<' '<<b<<' '<<labels[i]<<' '<<labelKinds[i]<<'\n';
    }
    out.close();if(!out.good())throw std::runtime_error("Cannot finish segmented PLY");
    QFile sidecar(path.dir().filePath(path.completeBaseName()+".segmented.json"));
    if(!sidecar.open(QIODevice::WriteOnly))throw std::runtime_error("Cannot save segmented units");
    sidecar.write(QJsonDocument(QJsonObject{{"unit","mm"},{"source","surface_analysis"}}).toJson());
    report["segmented_cloud"]=QFileInfo(colored).absoluteFilePath();
}
