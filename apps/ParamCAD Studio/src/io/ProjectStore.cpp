#include "io/ProjectStore.h"
#include "io/FeatureCodec.h"
#include "io/PlyWriter.h"
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <algorithm>

namespace {
double volume(const TopoDS_Shape& shape)
{
    if (shape.IsNull()) return 0;
    GProp_GProps properties; BRepGProp::VolumeProperties(shape,properties);
    return std::abs(properties.Mass());
}
bool sameMaterial(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull()) return false;
    const double tolerance=std::max(1.0,volume(a))*1e-8;
    if (std::abs(volume(a)-volume(b))>tolerance) return false;
    // Equal bounds/volume alone cannot distinguish relocated holes. Test the
    // material difference in both directions for legacy/source validation.
    BRepAlgoAPI_Cut first(a,b); first.SetRunParallel(false); first.Build();
    if (!first.IsDone() || volume(first.Shape())>tolerance) return false;
    BRepAlgoAPI_Cut second(b,a); second.SetRunParallel(false); second.Build();
    return second.IsDone() && volume(second.Shape())<=tolerance;
}
void check(bool ok, const QString& message = QStringLiteral("Invalid or incomplete project data"))
{
    if (!ok)
        throw std::runtime_error(message.isEmpty() ? "Invalid or incomplete project data"
                                                   : message.toStdString());
}
QJsonArray array(const double* values)
{
    return {values[0], values[1], values[2]};
}
void readVector(const QJsonValue& value, double* result)
{
    check(value.isArray() && value.toArray().size() == 3);
    for (int i = 0; i < 3; ++i) {
        const auto n = value.toArray()[i];
        check(n.isDouble() && std::isfinite(n.toDouble()));
        result[i] = n.toDouble();
    }
}
double number(const QJsonValue& value, double minimum, double maximum)
{
    check(value.isDouble());
    const double result = value.toDouble();
    check(std::isfinite(result) && result >= minimum && result <= maximum);
    return result;
}
int integer(const QJsonValue& value, int minimum, int maximum)
{
    const double result = number(value, minimum, maximum);
    check(std::floor(result) == result);
    return static_cast<int>(result);
}
bool boolean(const QJsonValue& value)
{
    check(value.isBool());
    return value.toBool();
}
QColor color(const QJsonValue& value)
{
    check(value.isString());
    QColor result(value.toString());
    check(result.isValid());
    return result;
}
QJsonObject viewJson(const ProjectViewState& v)
{
    return {{"position", array(v.camera.position)},
            {"target", array(v.camera.target)},
            {"up", array(v.camera.up)},
            {"fov", v.camera.fieldOfView},
            {"parallel", v.camera.parallelProjection},
            {"parallelScale", v.parallelScale},
            {"cloudVisible", v.cloudVisible},
            {"pointSize", v.pointSize},
            {"selectionMode", v.selectionMode},
            {"datumPlanes", v.datumPlanes},
            {"originLock", v.originLock},
            {"object", QJsonObject{{"position", array(v.object.position.data())},
                                   {"rotation", array(v.object.rotation.data())},
                                   {"scale", array(v.object.scale.data())},
                                   {"visible", v.object.visible},
                                   {"representation", v.object.representation},
                                   {"opacity", v.object.opacity},
                                   {"color", v.object.color.name(QColor::HexArgb)}}},
            {"render", QJsonObject{{"background", v.render.background.name(QColor::HexArgb)},
                                   {"background2", v.render.background2.name(QColor::HexArgb)},
                                   {"gradientBackground", v.render.gradientBackground},
                                   {"lighting", v.render.lighting},
                                   {"width", v.render.width},
                                   {"height", v.render.height},
                                   {"textureEnabled", v.render.textureEnabled},
                                   {"textureProjection", v.render.textureProjection}}}};
}
ProjectViewState readView(const QJsonObject& j)
{
    ProjectViewState v;
    readVector(j["position"], v.camera.position);
    readVector(j["target"], v.camera.target);
    readVector(j["up"], v.camera.up);
    v.camera.fieldOfView = number(j["fov"], 1, 179);
    v.camera.parallelProjection = boolean(j["parallel"]);
    v.parallelScale = number(j["parallelScale"], 1e-12, 1e15);
    v.cloudVisible = boolean(j["cloudVisible"]);
    v.pointSize = number(j["pointSize"], 1, 100);
    v.selectionMode = integer(j["selectionMode"], 0, 2);
    v.datumPlanes = boolean(j["datumPlanes"]);
    v.originLock = boolean(j["originLock"]);
    double direction[3];
    for (int i = 0; i < 3; ++i)
        direction[i] = v.camera.target[i] - v.camera.position[i];
    const double length = std::hypot(direction[0], direction[1], direction[2]);
    const double cross = std::hypot(direction[1] * v.camera.up[2] - direction[2] * v.camera.up[1],
                                    direction[2] * v.camera.up[0] - direction[0] * v.camera.up[2],
                                    direction[0] * v.camera.up[1] - direction[1] * v.camera.up[0]);
    check(length > 1e-12 && cross > 1e-12, QStringLiteral("Project camera is degenerate"));
    check(!v.originLock ||
          std::hypot(v.camera.target[0], v.camera.target[1], v.camera.target[2]) < 1e-9);
    check(j["object"].isObject() && j["render"].isObject());
    const auto o = j["object"].toObject();
    readVector(o["position"], v.object.position.data());
    readVector(o["rotation"], v.object.rotation.data());
    readVector(o["scale"], v.object.scale.data());
    for (int i = 0; i < 3; ++i) {
        check(std::abs(v.object.position[i]) <= 100000 && std::abs(v.object.rotation[i]) <= 360);
        check(v.object.scale[i] >= 0.001 && v.object.scale[i] <= 1000);
    }
    v.object.visible = boolean(o["visible"]);
    v.object.representation = integer(o["representation"], 0, 2);
    v.object.opacity = number(o["opacity"], 0, 1);
    v.object.color = color(o["color"]);
    const auto r = j["render"].toObject();
    v.render.background = color(r["background"]);
    v.render.lighting = boolean(r["lighting"]);
    v.render.background2 = color(r["background2"]);
    v.render.gradientBackground = boolean(r["gradientBackground"]);
    v.render.width = integer(r["width"], 64, 16384);
    v.render.height = integer(r["height"], 64, 16384);
    v.render.textureEnabled = boolean(r["textureEnabled"]);
    v.render.textureProjection = integer(r["textureProjection"], 0, 4);
    return v;
}
std::filesystem::path nativePath(const QString& path)
{
    return std::filesystem::u8path(path.toUtf8().toStdString());
}
QByteArray digest(const QString& path)
{
    QFile file(path);
    check(file.open(QIODevice::ReadOnly), file.errorString());
    QCryptographicHash hash(QCryptographicHash::Sha256);
    check(hash.addData(&file), file.errorString());
    return hash.result().toHex();
}
void writeJson(const QString& path, const QJsonObject& json)
{
    QFile file(path);
    check(file.open(QIODevice::WriteOnly), file.errorString());
    const auto bytes = QJsonDocument(json).toJson();
    check(file.write(bytes) == bytes.size(), file.errorString());
}
void transfer(QFile& input, QIODevice& output, qint64 count, QCryptographicHash* hash = nullptr)
{
    while (count > 0) {
        const auto bytes = input.read(std::min<qint64>(count, 65536));
        check(!bytes.isEmpty(), QStringLiteral("Truncated project asset"));
        check(output.write(bytes) == bytes.size(), output.errorString());
        if (hash)
            hash->addData(bytes);
        count -= bytes.size();
    }
}
} // namespace

bool ProjectStore::save(const QString& path, const ProjectSnapshot& state, QString& error)
{
    error.clear();
    try {
        QTemporaryDir stage;
        check(stage.isValid(), QStringLiteral("Cannot create project staging directory"));
        // Reject a history that cannot be loaded before replacing an existing save.
        modeling::ModelingCore checked;
        if (!checked.restoreFeatures(state.features))
            check(false, QString::fromStdString(checked.lastError()));
        QJsonObject manifest{{"schema", "paramcad.project"},
                             {"version", 2},
                             {"features", FeatureCodec::encode(state.features)},
                             {"parametricBody", state.parametricBody},
                             {"view", viewJson(state.view)}};
        std::vector<modeling::Feature> decoded;
        check(FeatureCodec::decode(manifest["features"].toArray(), decoded, error), error);
        readView(manifest["view"].toObject());
        QStringList names;
        if (!state.currentBody.IsNull()) {
            if (state.parametricBody) check(sameMaterial(state.currentBody,checked.bodyShape()),
                QStringLiteral("Current body does not match its parametric features"));
            check(BRepCheck_Analyzer(state.currentBody).IsValid(),
                  QStringLiteral("Invalid project CAD body"));
            std::ofstream output(nativePath(stage.filePath("body.brep")), std::ios::binary);
            BRepTools::Write(state.currentBody, output);
            output.flush();
            check(output.good(), QStringLiteral("Cannot store project body"));
            names << "body.brep";
        }
        QJsonArray selected;
        if (state.cloud && state.cloud->hasCloud()) {
            check(PlyWriter::write(state.cloud->sourceCloud(), stage.filePath("cloud.ply"), error),
                  error);
            check(state.cloud->saveAnnotations(stage.filePath("annotations.json"), error), error);
            names << "cloud.ply" << "annotations.json";
            manifest["cloudName"] = QFileInfo(state.cloud->sourcePath()).fileName();
            manifest["cloudUnit"] = state.cloud->sourceUnit();
            for (auto id : state.cloud->selection().selected())
                selected.append(QString::number(id));
        }
        manifest["selection"] = selected;
        if (!state.view.render.texturePath.isEmpty()) {
            QImage image(state.view.render.texturePath);
            check(!image.isNull(), QStringLiteral("Cannot read project texture"));
            check(image.save(stage.filePath("texture.png")),
                  QStringLiteral("Cannot store project texture"));
            names << "texture.png";
        }
        QJsonArray assets;
        for (const auto& name : names)
            assets.append(
                QJsonObject{{"name", name},
                            {"bytes", QString::number(QFileInfo(stage.filePath(name)).size())},
                            {"sha256", QString::fromLatin1(digest(stage.filePath(name)))}});
        manifest["assets"] = assets;
        const auto json = QJsonDocument(manifest).toJson(QJsonDocument::Compact);
        check(json.size() <= 16 * 1024 * 1024, QStringLiteral("Project metadata is too large"));
        QSaveFile output(path);
        check(output.open(QIODevice::WriteOnly), output.errorString());
        check(output.write("PCADPRJ1", 8) == 8, output.errorString());
        QDataStream stream(&output);
        stream << static_cast<quint32>(json.size());
        check(stream.status() == QDataStream::Ok && output.write(json) == json.size(),
              output.errorString());
        for (const auto& name : names) {
            QFile input(stage.filePath(name));
            check(input.open(QIODevice::ReadOnly), input.errorString());
            transfer(input, output, input.size());
        }
        check(output.commit(), output.errorString());
        return true;
    } catch (const Standard_Failure& failure) {
        error = QString::fromUtf8(failure.what());
        return false;
    } catch (const std::exception& failure) {
        error = QString::fromUtf8(failure.what());
        return false;
    }
}

std::unique_ptr<PreparedProject> ProjectStore::load(const QString& path, QString& error)
{
    error.clear();
    try {
        QFile input(path);
        check(input.open(QIODevice::ReadOnly), input.errorString());
        check(input.read(8) == "PCADPRJ1", QStringLiteral("Not a supported ParamCAD project"));
        QDataStream stream(&input);
        quint32 length = 0;
        stream >> length;
        check(stream.status() == QDataStream::Ok && length > 0 && length <= 16 * 1024 * 1024 &&
              length <= input.size() - input.pos());
        const auto json = input.read(length);
        check(json.size() == length);
        QJsonParseError parse;
        const auto document = QJsonDocument::fromJson(json, &parse);
        check(parse.error == QJsonParseError::NoError && document.isObject());
        const auto manifest = document.object();
        check(manifest["schema"] == "paramcad.project" &&
                  (manifest["version"] == 1 || manifest["version"] == 2),
              QStringLiteral("Unsupported project format/version"));
        check(manifest["features"].isArray() && manifest["view"].isObject() &&
              manifest["assets"].isArray() && manifest["selection"].isArray());
        auto result = std::make_unique<PreparedProject>();
        result->assets = std::make_shared<QTemporaryDir>();
        check(result->assets->isValid());
        result->view = readView(manifest["view"].toObject());
        std::set<QString> names;
        for (const auto& value : manifest["assets"].toArray()) {
            check(value.isObject());
            const auto asset = value.toObject();
            const auto name = asset["name"].toString();
            check((name == "body.brep" || name == "cloud.ply" || name == "annotations.json" ||
                   name == "texture.png") &&
                  names.insert(name).second);
            bool ok = false;
            const auto size = asset["bytes"].toString().toLongLong(&ok);
            check(ok && size > 0 && size <= input.size() - input.pos());
            QFile output(result->assets->filePath(name));
            check(output.open(QIODevice::WriteOnly), output.errorString());
            QCryptographicHash hash(QCryptographicHash::Sha256);
            transfer(input, output, size, &hash);
            output.close();
            check(asset["sha256"].isString() &&
                      hash.result().toHex() == asset["sha256"].toString().toLatin1(),
                  QStringLiteral("Project asset checksum mismatch"));
        }
        check(input.atEnd(), QStringLiteral("Unexpected trailing project data"));
        std::vector<modeling::Feature> features;
        check(FeatureCodec::decode(manifest["features"].toArray(), features, error), error);
        if (!result->model.restoreFeatures(features))
            check(false, QString::fromStdString(result->model.lastError()));
        if (names.count("body.brep")) {
            std::ifstream body(nativePath(result->assets->filePath("body.brep")), std::ios::binary);
            BRep_Builder builder;
            BRepTools::Read(result->currentBody, body, builder);
            check(!body.fail() && !result->currentBody.IsNull() &&
                      BRepCheck_Analyzer(result->currentBody).IsValid(),
                  QStringLiteral("Invalid project CAD body"));
            const bool parametric = manifest.contains("parametricBody")
                ? boolean(manifest["parametricBody"])
                : sameMaterial(result->currentBody,result->model.bodyShape());
            if (parametric) {
                check(sameMaterial(result->currentBody,result->model.bodyShape()),
                    QStringLiteral("Stored body does not match its parametric features"));
                result->currentBody=result->model.bodyShape();
            }
        } else if (manifest.contains("parametricBody")) {
            check(!boolean(manifest["parametricBody"]),QStringLiteral("Parametric body asset is missing"));
        }
        check(names.count("cloud.ply") == names.count("annotations.json"));
        if (names.count("cloud.ply")) {
            const auto name = manifest["cloudName"].toString();
            check(!name.isEmpty() && QFileInfo(name).fileName() == name &&
                  QFileInfo(name).suffix().compare("ply", Qt::CaseInsensitive) == 0);
            const auto unit = manifest["cloudUnit"].toString();
            check(unit == "mm" || unit == "m" || unit == "arbitrary");
            QDir folder(result->assets->path());
            check(folder.mkdir("source"));
            const auto source = folder.filePath("source/" + name);
            check(QFile::copy(result->assets->filePath("cloud.ply"), source));
            writeJson(
                QFileInfo(source).dir().filePath(QFileInfo(source).completeBaseName() + ".json"),
                {{"unit", unit}});
            check(result->cloud->load(source, error), error);
            check(
                result->cloud->loadAnnotations(result->assets->filePath("annotations.json"), error),
                error);
        }
        std::vector<vtkIdType> selected;
        for (const auto& value : manifest["selection"].toArray()) {
            bool ok = false;
            const auto id = value.toString().toLongLong(&ok);
            check(ok && result->cloud->selection().active(id));
            selected.push_back(id);
        }
        result->cloud->select(selected);
        if (names.count("texture.png")) {
            result->view.render.texturePath = result->assets->filePath("texture.png");
            result->texture = QImage(result->view.render.texturePath);
            check(!result->texture.isNull(), QStringLiteral("Invalid project texture"));
        }
        return result;
    } catch (const Standard_Failure& failure) {
        error = QString::fromUtf8(failure.what());
        return nullptr;
    } catch (const std::exception& failure) {
        error = QString::fromUtf8(failure.what());
        return nullptr;
    }
}
