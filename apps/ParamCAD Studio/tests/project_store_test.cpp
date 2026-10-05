#include "io/ProjectStore.h"
#include "io/PlyWriter.h"
#include "io/FeatureCodec.h"
#include "io/PointCloudAdapter.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <QCoreApplication>
#include <QDataStream>
#include <QFile>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <limits>

namespace {
void require(bool ok, const char* message)
{
    if (!ok)
        throw std::runtime_error(message);
}
double volume(const TopoDS_Shape& shape)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(shape, p);
    return p.Mass();
}
QByteArray read(const QString& path)
{
    QFile f(path);
    require(f.open(QIODevice::ReadOnly), "read file");
    return f.readAll();
}
void write(const QString& path, const QByteArray& bytes)
{
    QFile f(path);
    require(f.open(QIODevice::WriteOnly), "write file");
    require(f.write(bytes) == bytes.size(), "write all bytes");
}
std::vector<modeling::Feature> history()
{
    using namespace modeling;
    SketchFeatureParams base;
    base.entities.push_back({101, Rectangle2D{0, 0, 10, 8}});
    SketchFeatureParams hole;
    hole.plane = featureFace(20, FaceRole::EndFace);
    hole.entities.push_back({201, Circle2D{5, 4, 1}});
    auto& face = std::get<FaceReference>(hole.plane);
    face.normal = {0, 0, 1};
    face.centroid = {5, 4, 6};
    face.area = 80;
    face.transientFaceIndexHint = 5;
    SketchFeatureParams suppressed;
    suppressed.plane = offsetDatumPlane(DatumPlane::YZ, 2);
    suppressed.entities = {{301, Point2D{2, 3}}, {302, Line2D{0, 0, 1, 1}}};
    SketchFeatureParams explicitSketch;
    explicitSketch.plane = explicitPlane({3, 4, 5}, {0, 1, 0}, {1, 0, 0});
    explicitSketch.entities = {{401, Rectangle2D{1, 2, 3, 4}}};
    return {{10, "底面", FeatureType::Sketch, base},
            {20, "拉伸", FeatureType::Extrude, ExtrudeFeatureParams{10, 6, false}},
            {30, "孔草图", FeatureType::Sketch, hole},
            {40, "通孔", FeatureType::Cut, CutFeatureParams{30, 6, true, false}},
            {50, "辅助草图", FeatureType::Sketch, suppressed, true},
            {60, "独立平面", FeatureType::Sketch, explicitSketch, true}};
}
QByteArray withManifest(const QByteArray& archive, const QJsonObject& manifest)
{
    QByteArray old = archive;
    QDataStream reader(&old, QIODevice::ReadOnly);
    reader.skipRawData(8);
    quint32 oldLength = 0;
    reader >> oldLength;
    const auto json = QJsonDocument(manifest).toJson(QJsonDocument::Compact);
    QByteArray result;
    QDataStream out(&result, QIODevice::WriteOnly);
    out.writeRawData("PCADPRJ1", 8);
    out << static_cast<quint32>(json.size());
    out.writeRawData(json.constData(), static_cast<int>(json.size()));
    result += archive.mid(12 + oldLength);
    return result;
}
QJsonObject manifest(const QByteArray& archive)
{
    QByteArray bytes = archive;
    QDataStream stream(&bytes, QIODevice::ReadOnly);
    stream.skipRawData(8);
    quint32 length = 0;
    stream >> length;
    return QJsonDocument::fromJson(archive.mid(12, length)).object();
}

void primitiveProjects(QTemporaryDir& temp)
{
    using namespace modeling;
    const std::vector<FeatureParams> primitives{
        BoxPrimitiveParams{{{3, 4, 5}, {0, 1, 0}, {1, 0, 0}}, 10, 12, 14, false},
        SpherePrimitiveParams{{3, 4, 5}, 6}, CylinderPrimitiveParams{{3, 4, 5}, {0, 1, 0}, 6, 10},
        ConePrimitiveParams{{3, 4, 5}, {0, 1, 0}, 6, 2, 10},
        TorusPrimitiveParams{{3, 4, 5}, {0, 1, 0}, 10, 2}};
    QString error;
    for (const auto& params : primitives) {
        ProjectSnapshot state;
        state.features = {{70, "重建基元", featureTypeOf(params), params}};
        state.features[0].reconstructed = true;
        if (std::holds_alternative<BoxPrimitiveParams>(params)) {
            state.features.push_back({80, "通孔", FeatureType::ThroughHolePrimitive,
                                      ThroughHolePrimitiveParams{{3, 9, 11}, {1, 0, 0}, 1}});
        }
        ModelingCore model;
        if (!model.restoreFeatures(state.features))
            throw std::runtime_error(model.lastError());
        state.currentBody = model.bodyShape();
        const auto path = temp.filePath("primitive.pcad");
        require(ProjectStore::save(path, state, error), "save recognized primitive parameters");
        auto loaded = ProjectStore::load(path, error);
        require(loaded && loaded->model.features()[0].type == featureTypeOf(params),
                "load primitive feature type");
        require(loaded->model.features()[0].reconstructed,
                "project retains reconstruction provenance for subsequent replacement");
        GProp_GProps before, after;
        BRepGProp::VolumeProperties(model.bodyShape(), before);
        BRepGProp::VolumeProperties(loaded->model.bodyShape(), after);
        require(std::abs(before.Mass() - after.Mass()) < 1e-5 &&
                    before.CentreOfMass().Distance(after.CentreOfMass()) < 1e-6,
                "restored primitive dimensions, pose and holes rebuild the original geometry");
    }
}
} // namespace
int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory");
        QString error;
        PointCloudDocument cloud;
        PlyCloud observations;
        for (int i = 0; i < 6; ++i) {
            observations.positions.push_back({static_cast<float>(i), 0, 0});
            observations.normals.push_back({0, 0, 1});
            observations.colors.push_back({10, 20, 30});
        }
        // Invalid observations retain their original IDs and fingerprint; the
        // document masks them rather than compacting them out of the source.
        observations.positions.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
        observations.normals.push_back({0, 0, 1});
        observations.colors.push_back({10, 20, 30});
        const auto cloudPath = temp.filePath("observations.ply");
        require(PlyWriter::write(observations, cloudPath, error), "write source cloud");
        write(temp.filePath("observations.json"), "{\"unit\":\"mm\"}");
        require(cloud.load(cloudPath, error), "load cloud");
        cloud.select({0, 1});
        const int first = cloud.assignRegion("plane", "顶面");
        cloud.select({2, 3});
        const int second = cloud.assignRegion("plane", "侧面");
        const int relation = cloud.setRelation(0, first, second, "perpendicular");
        require(relation != 0, "create relationship");
        cloud.selectPoint(4);
        cloud.deleteSelectedPoints();
        cloud.selectRelation(relation);
        ProjectSnapshot snapshot;
        snapshot.features = history();
        snapshot.cloud = &cloud;
        modeling::ModelingCore model;
        if (!model.restoreFeatures(snapshot.features))
            throw std::runtime_error(model.lastError());
        const double modeledVolume = volume(model.bodyShape());
        require(modeledVolume > 400 && modeledVolume < 480, "cut volume");
        snapshot.currentBody = BRepPrimAPI_MakeBox(gp_Pnt(100, -40, 15), 2, 3, 4).Shape();
        snapshot.view.camera.position[0] = 42;
        snapshot.view.camera.position[1] = -20;
        snapshot.view.camera.position[2] = 80;
        snapshot.view.camera.target[0] = 5;
        snapshot.view.camera.target[1] = 4;
        snapshot.view.camera.target[2] = 1;
        snapshot.view.camera.parallelProjection = true;
        snapshot.view.parallelScale = 17.321;
        snapshot.view.originLock = false;
        snapshot.view.object.position = {11, 12, 13};
        snapshot.view.object.scale = {2, 3, 4};
        snapshot.view.object.opacity = 0.3;
        snapshot.view.cloudVisible = false;
        snapshot.view.pointSize = 5;
        snapshot.view.selectionMode = 2;
        QImage texture(16, 16, QImage::Format_RGBA8888);
        texture.fill(Qt::cyan);
        snapshot.view.render.texturePath = temp.filePath("texture.png");
        require(texture.save(snapshot.view.render.texturePath), "texture fixture");
        snapshot.view.render.textureEnabled = true;
        snapshot.view.render.textureProjection = 4;
        const auto path = temp.filePath(QStringLiteral("完整项目.pcad"));
        require(ProjectStore::save(path, snapshot, error), "save standalone project");
        const auto original = read(path);
        require(QFile::remove(cloudPath) && QFile::remove(temp.filePath("observations.json")) &&
                    QFile::remove(snapshot.view.render.texturePath),
                "remove external source files");
        auto restored = ProjectStore::load(path, error);
        if (!restored)
            throw std::runtime_error(error.toStdString());
        require(restored->model.features().size() == 6 && restored->model.features()[0].id == 10 &&
                    restored->model.features()[4].suppressed &&
                    restored->model.features()[0].name == "底面",
                "IDs, names and suppression survive load");
        require(std::abs(volume(restored->model.bodyShape()) - modeledVolume) < 1e-6,
                "editable history rebuilds same solid");
        GProp_GProps properties;
        BRepGProp::VolumeProperties(restored->currentBody, properties);
        require(std::abs(properties.Mass() - 24) < 1e-6 &&
                    properties.CentreOfMass().Distance(gp_Pnt(101, -38.5, 17)) < 1e-6,
                "active imported body and placement survive separately from history");
        require(restored->cloud->sourceCloud().positions.size() == 7 &&
                    std::memcmp(restored->cloud->sourceCloud().positions.data(),
                                cloud.sourceCloud().positions.data(),
                                7 * sizeof(observations.positions[0])) == 0 &&
                    restored->cloud->pointCount() == 5 && !restored->cloud->selection().active(4) &&
                    restored->cloud->selection().relation(relation),
                "original observations, deletion mask and relations survive");
        require(restored->cloud->selection().selected() == std::vector<vtkIdType>({0, 1, 2, 3}),
                "original point selection survives");
        require(restored->cloud->selection().region(first)->templateLocked
                && restored->cloud->selection().region(second)->templateLocked,
                "manual template locks survive standalone project restoration");
        require(PointCloudAdapter::lengthUnitFor(restored->cloud->sourcePath()) ==
                    reconstruction::LengthUnit::Millimeter,
                "embedded scale metadata survives removal of source files");
        require(restored->view.camera.parallelProjection &&
                    restored->view.parallelScale == 17.321 &&
                    restored->view.camera.target[0] == 5 && restored->view.object.scale[2] == 4 &&
                    restored->view.render.textureProjection == 4 &&
                    !QImage(restored->view.render.texturePath).isNull(),
                "camera, transforms and self-contained texture survive");
        modeling::FeatureId next =
            restored->model.execute(modeling::AddFeatureCommand{modeling::SketchFeatureParams{}})
                .featureId;
        require(next > 60, "subsequent edits allocate fresh feature IDs");
        const auto& plane = std::get<modeling::Plane3d>(
            std::get<modeling::SketchFeatureParams>(restored->model.features()[5].params).plane);
        require(plane.origin.z == 5 && plane.normal.y == 1,
                "explicit sketch plane persists world placement");
        const auto& face = std::get<modeling::FaceReference>(
            std::get<modeling::SketchFeatureParams>(restored->model.features()[2].params).plane);
        require(face.normal.z == 1 && face.centroid.y == 4 && face.area == 80 &&
                    face.transientFaceIndexHint == 5,
                "face reference retains semantic and geometric hints");
        snapshot.features[1].params = modeling::ExtrudeFeatureParams{999, 6, false};
        snapshot.view.render.texturePath.clear();
        require(!ProjectStore::save(path, snapshot, error) && !error.isEmpty() &&
                    read(path) == original,
                "unrebuildable history cannot replace an existing project");
        const auto bad = temp.filePath("bad.pcad");
        write(bad, original.left(original.size() - 5));
        require(!ProjectStore::load(bad, error) && !error.isEmpty(), "truncated archive rejected");
        QByteArray corrupt = original;
        corrupt[corrupt.size() - 1] = static_cast<char>(corrupt.back() ^ 1);
        write(bad, corrupt);
        require(!ProjectStore::load(bad, error) && error.contains("checksum"),
                "corrupt asset rejected");
        auto metadata = manifest(original);
        metadata["version"] = 2;
        write(bad, withManifest(original, metadata));
        require(!ProjectStore::load(bad, error) && error.contains("version"),
                "future version rejected");
        metadata = manifest(original);
        metadata["cloudName"] = "../escape.ply";
        write(bad, withManifest(original, metadata));
        require(!ProjectStore::load(bad, error), "invalid embedded source path rejected");
        metadata = manifest(original);
        auto view = metadata["view"].toObject();
        view["target"] = view["position"];
        metadata["view"] = view;
        write(bad, withManifest(original, metadata));
        require(!ProjectStore::load(bad, error), "degenerate camera rejected");
        metadata = manifest(original);
        auto features = metadata["features"].toArray();
        auto duplicate = features[1].toObject();
        duplicate["id"] = features[0].toObject()["id"];
        features[1] = duplicate;
        metadata["features"] = features;
        write(bad, withManifest(original, metadata));
        require(!ProjectStore::load(bad, error), "duplicate stable feature IDs rejected");
        ProjectSnapshot empty;
        empty.view.render.textureEnabled = true;
        const auto emptyPath = temp.filePath("empty.pcad");
        require(ProjectStore::save(emptyPath, empty, error), "save empty project");
        auto blank = ProjectStore::load(emptyPath, error);
        require(blank && blank->model.features().empty() && blank->currentBody.IsNull() &&
                    !blank->cloud->hasCloud() && blank->view.render.textureEnabled &&
                    blank->texture.isNull(),
                "load empty project");
        primitiveProjects(temp);
        std::cout << "Project store tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
