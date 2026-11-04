#include "AssemblyDiagnostics.h"
#include "io/PointCloudAdapter.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QDataStream>
#include <QFileInfo>
#include <QDir>
#include <QSaveFile>
#include <stdexcept>
#include <cmath>

namespace {
QJsonArray vector(modeling::Vec3 p){return {p.x,p.y,p.z};}
void save(const QString& path,const QJsonObject& value){QSaveFile file(path);if(!file.open(QIODevice::WriteOnly))throw std::runtime_error("Cannot save component diagnostics");
    const auto bytes=QJsonDocument(value).toJson();if(file.write(bytes)!=bytes.size()||!file.commit())throw std::runtime_error("Cannot commit component diagnostics");}
}
QJsonObject writeAssemblyDiagnostics(const QString& reportPath,const PlyCloud& cloud,
    const reconstruction::AssemblyReconstructionResult& result,const QString& unit) {
    const auto directory=QFileInfo(reportPath).dir();const auto jsonPath=directory.filePath("parts.json"),plyPath=directory.filePath("segmented.ply");
    QJsonArray patches,parts,connections;size_t loops=0,closed=0,openEdges=0;
    for(const auto& patch:result.patches){QJsonArray boundaries;for(const auto& loop:patch.loops){QJsonArray values;for(auto p:loop)values.append(vector(p));boundaries.append(values);++loops;}
        patches.append(QJsonObject{{"id",double(patch.id)},{"planar",patch.planar},{"normal",vector(patch.normal)},{"origin_mm",vector(patch.origin)},
            {"x_direction",vector(patch.xDirection)},{"spacing_mm",patch.spacing},{"rms_residual_mm",patch.residual},{"orientation_known",patch.orientationKnown},
            {"point_count",double(patch.pointIds.size())},{"boundary_loops_mm",boundaries},{"snapped_vertices",double(patch.snappedVertices)},
            {"boundary_recovery",QJsonObject{{"traced_loops",double(patch.boundaryDiagnostics.tracedLoops)},
                {"non_simple_alpha_loops",double(patch.boundaryDiagnostics.nonSimpleAlphaLoops)},
                {"non_simple_simplified_loops",double(patch.boundaryDiagnostics.nonSimpleSimplifiedLoops)},
                {"non_simple_recovered_loops",double(patch.boundaryDiagnostics.nonSimpleRecoveredLoops)},
                {"discarded_constraint_loops",double(patch.boundaryDiagnostics.discardedConstraintLoops)},
                {"split_alpha_loops",double(patch.boundaryDiagnostics.splitAlphaLoops)},
                {"rejected_alpha_loops",double(patch.boundaryDiagnostics.rejectedAlphaLoops)},
                {"simplification_fallbacks",double(patch.boundaryDiagnostics.simplificationFallbacks)},
                {"constraint_fallbacks",double(patch.boundaryDiagnostics.constraintFallbacks)},
                {"snapping_fallbacks",double(patch.boundaryDiagnostics.snappingFallbacks)},
                {"topology_fallbacks",double(patch.boundaryDiagnostics.topologyFallbacks)}}}});}
    std::vector<quint32> labels(cloud.positions.size());
    std::size_t projected = 0, attempted = 0;
    for(const auto& part:result.parts){QJsonArray ids,patchIds;for(auto id:part.pointIds){ids.append(double(id));if(id>0&&id<=labels.size())labels[id-1]=quint32(part.id);}
        for(auto id:part.patchIds)patchIds.append(double(id));if(part.closed)++closed;openEdges+=part.openEdges;
        QJsonObject item{{"id",double(part.id)},{"patch_ids",patchIds},{"point_ids",ids},{"status",QString::fromStdString(part.status)},
            {"reason",QString::fromStdString(part.reason)},{"closed",part.closed},{"open_edges",double(part.openEdges)},{"nonmanifold_edges",double(part.nonmanifoldEdges)},
            {"lower_mm",vector(part.lower)},{"upper_mm",vector(part.upper)},{"confidence",part.confidence},{"orientation_known",part.orientationKnown},
            {"normal_orientation_inferred",part.normalOrientationInferred}};
        if (part.projection) {
            ++attempted;
            const auto& fit = *part.projection;
            const bool accepted = !fit.operations.empty();
            if (accepted) ++projected;
            item["projection_fit"] = QJsonObject{{"accepted",accepted},
                {"reason",QString::fromStdString(fit.reason)},{"operations",qint64(fit.operations.size())},
                {"p95_distance_mm",fit.quality.p95Distance},{"inlier_fraction",fit.quality.surfaceInlierFraction},
                {"unsupported_face_area_fraction",fit.quality.unsupportedFaceAreaFraction}};
        }
        parts.append(item);
    }
    for(const auto& connection:result.connections)connections.append(QJsonObject{{"first",double(connection.first)},{"second",double(connection.second)},
        {"convex",connection.convex},{"connected",connection.accepted},{"orientation_known",connection.orientationKnown},{"observation_pairs",double(connection.observations)},{"distance_mm",connection.distance}});
    QJsonObject summary{{"candidate_parts",double(result.parts.size())},{"closed_parts",double(closed)},{"open_parts",double(result.parts.size()-closed)},
        {"projection_attempted_parts",qint64(attempted)},{"projection_reconstructed_parts",qint64(projected)},
        {"unreconstructed_parts",qint64(result.parts.size()-closed-projected)},
        {"finite_patches",double(result.patches.size())},{"boundary_loops",double(loops)},{"connections",double(result.connections.size())},{"open_edges",double(openEdges)},
        {"unassigned_points",double(result.unassignedPointIds.size())},{"working_points",double(result.workingPoints)},{"distance_tolerance_mm",result.distanceTolerance},
        {"part_report",jsonPath},{"segmented_cloud",plyPath}};
    QJsonObject report{{"schema","paramcad.observation-assembly"},{"version",1},{"unit","mm"},{"source_model_read",false},
        {"original_instance_identity_verified",false},{"summary",summary},{"patches",patches},{"parts",parts},{"connections",connections}};
    save(jsonPath,report);
    QSaveFile file(plyPath);if(!file.open(QIODevice::WriteOnly))throw std::runtime_error("Cannot write segmented cloud");const bool normals=cloud.normals.size()==cloud.positions.size();
    QByteArray header="ply\nformat binary_little_endian 1.0\ncomment inferred component candidates; not reference labels\nelement vertex "+QByteArray::number(cloud.positions.size())+
        "\nproperty float x\nproperty float y\nproperty float z\n";
    if(normals)header+="property float nx\nproperty float ny\nproperty float nz\n";
    header+="property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uint part_id\nend_header\n";
    if(file.write(header)!=header.size())throw std::runtime_error("Cannot write segmentation header");QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for(size_t i=0;i<cloud.positions.size();++i){for(float value:cloud.positions[i])stream<<value;if(normals)for(float value:cloud.normals[i])stream<<value;
        const auto hash=labels[i]*2654435761U;stream<<quint8(labels[i]?64+(hash&127):120)<<quint8(labels[i]?64+((hash>>8)&127):120)<<quint8(labels[i]?64+((hash>>16)&127):120)<<labels[i];}
    if(stream.status()!=QDataStream::Ok||!file.commit())throw std::runtime_error("Cannot commit segmented cloud");
    save(directory.filePath("segmented.json"),QJsonObject{{"unit",unit},{"labels","inferred_part_candidates"}});return summary;
}

void probeAssemblyBoundaries(const QStringList& args,QJsonObject& report) {
    if(args.size()!=5)throw std::runtime_error("Usage: probe-assembly cloud.ply report.json options.json");
    QFile configuration(args[4]);
    if(!configuration.open(QIODevice::ReadOnly))throw std::runtime_error("Cannot read probe options");
    QJsonParseError parseError;
    const auto document=QJsonDocument::fromJson(configuration.readAll(),&parseError);
    if(parseError.error!=QJsonParseError::NoError || !document.isObject())throw std::runtime_error("Expected probe options object");
    const auto values=document.object();reconstruction::AssemblyOptions options;
    for(auto it=values.begin();it!=values.end();++it) {
        if(!it.value().isDouble() || !std::isfinite(it.value().toDouble()))throw std::runtime_error("Probe options must be finite numbers");
        const auto value=it.value().toDouble();
        if(it.key()=="maximum_working_points") {
            if(value<24 || value>2000000 || value!=std::floor(value))throw std::runtime_error("Probe point budget must be an integer in [24, 2000000]");
            options.maximumWorkingPoints=size_t(value);
        } else if(it.key()=="alpha_factor")options.alphaFactor=value;
        else if(it.key()=="connectivity_factor")options.connectivityFactor=value;
        else if(it.key()=="distance_tolerance_mm")options.distanceTolerance=value;
        else if(it.key()=="normal_angle_degrees")options.normalAngleRadians=value*std::acos(-1.0)/180;
        else throw std::runtime_error("Unknown assembly probe option");
    }
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    if(unit==reconstruction::LengthUnit::Arbitrary)throw std::runtime_error("Probe input requires declared mm/m units");
    PlyCloud cloud;QString failure;
    if(!PlyReader::read(args[2],cloud,failure))throw std::runtime_error(failure.toStdString());
    const auto input=PointCloudAdapter::toPointStore(cloud,unit);
    auto samples=input.points();
    if(unit==reconstruction::LengthUnit::Meter)for(auto& p:samples){p.position.x*=1000;p.position.y*=1000;p.position.z*=1000;}
    reconstruction::AssemblyReconstructionResult result;std::string error;
    if(!reconstruction::reconstructAssembly(reconstruction::PointStore(std::move(samples),reconstruction::LengthUnit::Millimeter),options,result,error))
        throw std::runtime_error(error);
    report["input_points"]=double(cloud.positions.size());
    report["source_model_read"]=false;report["reference_labels_used"]=false;
    report["probe_options"]=QJsonObject{{"maximum_working_points",double(options.maximumWorkingPoints)},
        {"alpha_factor",options.alphaFactor},{"connectivity_factor",options.connectivityFactor},
        {"normal_angle_degrees",options.normalAngleRadians*180/std::acos(-1.0)},
        {"distance_tolerance_mm",options.distanceTolerance}};
    report["assembly"]=writeAssemblyDiagnostics(args[3],cloud,result,unit==reconstruction::LengthUnit::Meter?"m":"mm");
    report["status"]="assembly_probe_completed";
    // No CAD export or source-completeness claim is made by this diagnostic mode.
}
