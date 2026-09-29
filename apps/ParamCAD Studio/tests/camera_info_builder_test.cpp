#include "core/OrbitCamera.h"
#include "io/CameraInfoBuilder.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

QJsonArray vectorJson(const double values[3])
{
    return {values[0], values[1], values[2]};
}

QJsonObject cameraJson(const OrbitParameters& orbit)
{
    const CameraParameters camera =
        OrbitCamera::toCamera(orbit, CameraParameters{}, true);
    QJsonObject result;
    result.insert(QStringLiteral("position"), vectorJson(camera.position));
    result.insert(QStringLiteral("target"), vectorJson(camera.target));
    result.insert(QStringLiteral("up"), vectorJson(camera.up));
    result.insert(QStringLiteral("azimuthDeg"), orbit.azimuthDeg);
    result.insert(QStringLiteral("elevationDeg"), orbit.elevationDeg);
    result.insert(QStringLiteral("distance"), orbit.distance);
    result.insert(QStringLiteral("fieldOfViewDeg"), camera.fieldOfView);
    result.insert(QStringLiteral("projection"), QStringLiteral("perspective"));
    return result;
}

void writeEmptyImage(const QString& path)
{
    QFile file(path);
    require(file.open(QIODevice::WriteOnly), "cannot create image placeholder");
}

void testAggregateManifestBuildsAllGridPoses()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "cannot create temporary capture directory");

    std::vector<OrbitParameters> views;
    for (double latitude : {-60.0, -30.0, 0.0, 30.0, 60.0}) {
        for (int longitude = 0; longitude < 360; longitude += 30) {
            views.push_back({10.0, static_cast<double>(longitude), latitude});
        }
    }
    views.push_back({10.0, 0.0, 90.0});
    views.push_back({10.0, 0.0, -90.0});
    require(views.size() == 62U, "30-degree grid must contain 62 unique views");

    QJsonArray shots;
    for (std::size_t index = 0; index < views.size(); ++index) {
        const QString name = QStringLiteral("shot_%1.png")
                                 .arg(index, 3, 10, QLatin1Char('0'));
        writeEmptyImage(temporary.filePath(name));
        QJsonObject shot;
        shot.insert(QStringLiteral("image"), name);
        shot.insert(QStringLiteral("camera"), cameraJson(views[index]));
        shots.append(shot);
    }

    QJsonObject dimensions;
    dimensions.insert(QStringLiteral("width"), 1920);
    dimensions.insert(QStringLiteral("height"), 1080);
    QJsonObject render;
    render.insert(QStringLiteral("width"), 1280);
    render.insert(QStringLiteral("height"), 720);
    QJsonObject manifest;
    manifest.insert(QStringLiteral("schema"), QStringLiteral("cadpc.capture.manifest"));
    manifest.insert(QStringLiteral("version"), 2);
    manifest.insert(QStringLiteral("output"), dimensions);
    manifest.insert(QStringLiteral("render"), render);
    manifest.insert(QStringLiteral("shots"), shots);

    QFile manifestFile(temporary.filePath(QStringLiteral("manifest.json")));
    require(manifestFile.open(QIODevice::WriteOnly), "cannot create aggregate manifest");
    require(manifestFile.write(QJsonDocument(manifest).toJson()) > 0,
            "cannot write aggregate manifest");
    manifestFile.close();

    const QString outputPath = temporary.filePath(QStringLiteral("camera_info.json"));
    QString error;
    const CameraInfoBuildResult result =
        CameraInfoBuilder::build(temporary.path(), outputPath, error);
    require(error.isEmpty(), error.toStdString());
    require(result.built && result.totalImages == 62 && result.posedImages == 62,
            "aggregate manifest must provide all 62 poses without sidecars");

    QFile output(outputPath);
    require(output.open(QIODevice::ReadOnly), "cannot read generated camera info");
    const QJsonDocument document = QJsonDocument::fromJson(output.readAll());
    require(document.isObject(), "generated camera info must be an object");
    const QJsonObject root = document.object();
    require(root.value(QStringLiteral("images")).toArray().size() == 62,
            "generated camera info must contain 62 images");
    require(root.value(QStringLiteral("cameras")).toObject().size() == 1,
            "shared capture intrinsics should produce one COLMAP camera");
}

} // namespace

int main()
{
    try {
        testAggregateManifestBuildsAllGridPoses();
        std::cout << "CameraInfo aggregate-manifest test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "CameraInfo aggregate-manifest test failed: "
                  << exception.what() << '\n';
        return 1;
    }
}
