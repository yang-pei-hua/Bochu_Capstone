#include "ReconstructionContractData.h"

#include "io/FeatureCodec.h"
#include "io/ProjectStore.h"
#include "occt/Builders.h"
#include <modeling/SemanticFaces.h>
#include <modeling/SketchValidation.h>
#include <modeling/StepExporter.h>
#include <reconstruction/modeling/ModelingAdapter.h>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <QDir>
#include <QFileInfo>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace contract {
namespace {
using namespace modeling;
using namespace reconstruction;

std::vector<Feature> readFeatures(const QJsonValue& value) {
    std::vector<Feature> features;
    QString error;
    if (!FeatureCodec::decode(value.toArray(), features, error))
        throw std::runtime_error(error.toStdString());
    require(!features.empty(), "Expected feature history");
    return features;
}

double volume(const TopoDS_Shape& body) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(body, props);
    return props.Mass();
}

void measureBody(const TopoDS_Shape& body, QJsonObject& metrics) {
    require(!body.IsNull() && BRepCheck_Analyzer(body).IsValid(), "Invalid B-rep");
    int solids = 0;
    for (TopExp_Explorer solid(body, TopAbs_SOLID); solid.More(); solid.Next()) ++solids;
    require(solids > 0, "B-rep contains no solids");
    metrics["solid_count"] = solids;
    metrics["volume_mm3"] = volume(body);
}

QString writeBody(const TopoDS_Shape& body) {
    std::ostringstream stream;
    BRepTools::Write(body, stream);
    require(stream.good(), "Cannot encode B-rep artifact");
    return QString::fromStdString(stream.str());
}

gp_Pnt point(Vec3 p) { return {p.x, p.y, p.z}; }
gp_Dir direction(Vec3 v) { return {v.x, v.y, v.z}; }

TopoDS_Shape readBody(const QJsonObject& input) {
    if (input["brep"].isString()) {
        TopoDS_Shape body;
        BRep_Builder builder;
        std::istringstream stream(input["brep"].toString().toStdString());
        BRepTools::Read(body, stream, builder);
        require(!body.IsNull(), "Cannot decode B-rep artifact");
        return body;
    }
    // A reference B-rep is built from analytic solids, independently of the
    // feature replay and STEP translator being tested.
    const auto reference = input["reference_body"].toObject();
    require(reference["kind"] == "box_with_blind_hole", "Expected B-rep artifact or analytic body");
    const auto dimensions = vector(reference["size"]);
    const auto center = vector(reference["hole_floor_center"]);
    const auto box = BRepPrimAPI_MakeBox(point(vector(reference["origin"])), dimensions.x, dimensions.y, dimensions.z).Shape();
    const auto tool = BRepPrimAPI_MakeCylinder(gp_Ax2(point(center), gp_Dir(0, 0, 1)),
        reference["hole_radius"].toDouble(), reference["tool_height"].toDouble()).Shape();
    BRepAlgoAPI_Cut cut(box, tool);
    require(cut.IsDone(), "Cannot build analytic reference B-rep");
    return cut.Shape();
}

ModelPatch readPlan(const QJsonObject& input) {
    require(input["expected_revision"].isString(), "Expected a revision-bound modeling plan");
    bool parsed = false;
    ModelPatch patch;
    patch.expectedRevision = input["expected_revision"].toString().toULongLong(&parsed);
    require(parsed && patch.expectedRevision != kAnyModelRevision, "Invalid plan revision");
    for (const auto& feature : readFeatures(input["plan_features"]))
        patch.commands.emplace_back(AddFeatureCommand{feature.params, feature.reconstructed});
    return patch;
}

std::vector<Feature> planFeatures(const ModelPatch& patch) {
    ModelingCore staging;
    for (const auto& command : patch.commands) {
        require(std::holds_alternative<AddFeatureCommand>(command), "Expected an additive plan for an empty model");
        const auto result = staging.execute(command);
        if (!result.success) throw std::runtime_error(result.error);
    }
    return staging.features();
}

void verifyDependencies(const std::vector<Feature>& features, QJsonObject& metrics) {
    std::unordered_set<FeatureId> sketches, bodies;
    int cuts = 0;
    for (const auto& feature : features) {
        if (std::holds_alternative<SketchFeatureParams>(feature.params)) {
            sketches.insert(feature.id);
        } else if (const auto* extrude = std::get_if<ExtrudeFeatureParams>(&feature.params)) {
            require(sketches.count(extrude->sketchId), "Plan extrusion does not reference an earlier sketch");
            if (extrude->operation == ExtrudeOperation::Cut) {
                require(bodies.count(extrude->targetBody), "Planned cut has no upstream target body");
                ++cuts;
            }
            bodies.insert(feature.id);
        }
    }
    metrics["command_count"] = qint64(features.size());
    metrics["scoped_cuts"] = cuts;
}

double validateProfiles(const std::vector<Feature>& features, QJsonObject& metrics) {
    int count = 0;
    double area = 0;
    std::string error;
    for (const auto& feature : features) {
        const auto* sketch = std::get_if<SketchFeatureParams>(&feature.params);
        if (!sketch) continue;
        if (!validateSketch(*sketch, error)) throw std::runtime_error(error);
        Plane3d plane;
        if (!resolveSketchPlane(features, sketch->plane, plane, error)) throw std::runtime_error(error);
        const auto normal = direction(plane.normal);
        const auto x = direction(plane.xDirection);
        const occt::PlaneFrame frame{point(plane.origin), x, normal.Crossed(x), normal};
        TopoDS_Face face;
        if (!occt::buildSketchFace(*sketch, frame, face, error)) throw std::runtime_error(error);
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties);
        area += properties.Mass();
        ++count;
    }
    require(count > 0, "Plan contains no sketch profiles");
    metrics["face_count"] = count;
    return area;
}
} // namespace

QJsonObject modelingStage(int module, QJsonObject input, QJsonObject& metrics, const QString& artifactPath) {
    if (module == 28) {
        CompoundCandidate candidate;
        candidate.projectionPrograms.push_back(readProjectionProgram(input["selected_operations"]));
        const ModelingCore snapshot;
        std::string error;
        const auto patch = planCompoundCandidate(candidate, snapshot, error);
        if (!patch) throw std::runtime_error(error);
        const auto features = planFeatures(*patch);
        verifyDependencies(features, metrics);
        metrics["source_revision"] = qint64(snapshot.revision());
        input["plan_features"] = FeatureCodec::encode(features);
        input["expected_revision"] = QString::number(patch->expectedRevision);
    } else if (module == 29) {
        const auto features = readFeatures(input["plan_features"]);
        metrics["area_mm2"] = validateProfiles(features, metrics);
    } else if (module == 30) {
        ModelingCore model;
        if (!model.restoreFeatures(readFeatures(input["plan_features"]))) throw std::runtime_error(model.lastError());
        measureBody(model.bodyShape(), metrics);
        metrics["feature_count"] = qint64(model.features().size());
        input["features"] = FeatureCodec::encode(model.features());
        input["brep"] = writeBody(model.bodyShape());
    } else if (module == 31) {
        const auto patch = readPlan(input);
        ModelingCore model;
        auto invalid = patch;
        invalid.commands.emplace_back(AddFeatureCommand{ExtrudeFeatureParams{999999, 2}});
        const auto bad = model.apply(invalid);
        metrics["rollback_verified"] = !bad.success && model.features().empty() && model.revision() == patch.expectedRevision;
        const auto applied = model.apply(patch);
        if (!applied.success) throw std::runtime_error(applied.error);
        const auto features = FeatureCodec::encode(model.features());
        const auto stale = model.apply(patch);
        metrics["stale_plan_rejected"] = !stale.success && model.revision() == applied.revision &&
            FeatureCodec::encode(model.features()) == features;
        metrics["revision"] = qint64(model.revision());
        metrics["affected_count"] = qint64(applied.affectedFeatures.size());
        measureBody(model.bodyShape(), metrics);
        input["features"] = features;
        input["brep"] = writeBody(model.bodyShape());
    } else if (module == 32) {
        ProjectSnapshot snapshot;
        snapshot.features = readFeatures(input["features"]);
        snapshot.currentBody = readBody(input);
        snapshot.parametricBody = true;
        const auto path = QFileInfo(artifactPath).dir().filePath("roundtrip.pcad");
        QString error;
        if (!ProjectStore::save(path, snapshot, error)) throw std::runtime_error(error.toStdString());
        const auto restored = ProjectStore::load(path, error);
        if (!restored) throw std::runtime_error(error.toStdString());
        const auto features = FeatureCodec::encode(restored->model.features());
        metrics["features_preserved"] = features == FeatureCodec::encode(snapshot.features);
        metrics["feature_count"] = qint64(restored->model.features().size());
        measureBody(restored->currentBody, metrics);
        input["project"] = path;
        input["features"] = features;
        input["brep"] = writeBody(restored->currentBody);
    } else {
        const auto body = readBody(input);
        QJsonObject sourceMetrics;
        measureBody(body, sourceMetrics);
        const auto path = QFileInfo(artifactPath).dir().filePath("roundtrip.step");
        const auto file = std::filesystem::path(path.toStdWString());
        std::string error;
        if (!exportStep(body, file, &error)) throw std::runtime_error(error);
        std::ifstream stream(file, std::ios::binary);
        STEPControl_Reader reader;
        require(stream.good() && reader.ReadStream("roundtrip.step", stream) == IFSelect_RetDone &&
            reader.TransferRoots() > 0, "Cannot read exported STEP");
        const auto restored = reader.OneShape();
        measureBody(restored, metrics);
        metrics["solid_count_preserved"] = metrics["solid_count"] == sourceMetrics["solid_count"];
        BRepAlgoAPI_Common common(body, restored);
        require(common.IsDone(), "Cannot compare exported STEP material");
        const auto overlap = volume(common.Shape());
        metrics["material_iou"] = overlap / (volume(body) + volume(restored) - overlap);
        input["step"] = path;
    }
    return input;
}
} // namespace contract
