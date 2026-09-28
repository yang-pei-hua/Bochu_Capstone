#include "io/CameraInfoBuilder.h"

#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>

#include <cmath>
#include <map>
#include <vector>

namespace {

constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

const QStringList& imageSuffixes()
{
    static const QStringList suffixes{
        QStringLiteral("jpg"),  QStringLiteral("jpeg"), QStringLiteral("png"),
        QStringLiteral("bmp"),  QStringLiteral("tif"),  QStringLiteral("tiff"),
        QStringLiteral("exr"),  QStringLiteral("ppm"),  QStringLiteral("pgm"),
    };
    return suffixes;
}

struct Vec3
{
    double value[3] = {0.0, 0.0, 0.0};
};

double dot(const Vec3& left, const Vec3& right)
{
    return left.value[0] * right.value[0] + left.value[1] * right.value[1]
           + left.value[2] * right.value[2];
}

Vec3 cross(const Vec3& left, const Vec3& right)
{
    Vec3 result;
    result.value[0] = left.value[1] * right.value[2] - left.value[2] * right.value[1];
    result.value[1] = left.value[2] * right.value[0] - left.value[0] * right.value[2];
    result.value[2] = left.value[0] * right.value[1] - left.value[1] * right.value[0];
    return result;
}

bool normalize(Vec3& vector)
{
    const double length = std::sqrt(dot(vector, vector));
    if (length < 1e-12) {
        return false;
    }
    for (double& component : vector.value) {
        component /= length;
    }
    return true;
}

struct Quaternion
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Shepperd's method: the branch widest from the trace keeps the division stable
// for rotations near 180 degrees.
Quaternion quaternionFromMatrix(const double matrix[3][3])
{
    Quaternion result;
    const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        result.w = 0.25 * scale;
        result.x = (matrix[2][1] - matrix[1][2]) / scale;
        result.y = (matrix[0][2] - matrix[2][0]) / scale;
        result.z = (matrix[1][0] - matrix[0][1]) / scale;
    } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
        const double scale = std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0;
        result.w = (matrix[2][1] - matrix[1][2]) / scale;
        result.x = 0.25 * scale;
        result.y = (matrix[0][1] + matrix[1][0]) / scale;
        result.z = (matrix[0][2] + matrix[2][0]) / scale;
    } else if (matrix[1][1] > matrix[2][2]) {
        const double scale = std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0;
        result.w = (matrix[0][2] - matrix[2][0]) / scale;
        result.x = (matrix[0][1] + matrix[1][0]) / scale;
        result.y = 0.25 * scale;
        result.z = (matrix[1][2] + matrix[2][1]) / scale;
    } else {
        const double scale = std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0;
        result.w = (matrix[1][0] - matrix[0][1]) / scale;
        result.x = (matrix[0][2] + matrix[2][0]) / scale;
        result.y = (matrix[1][2] + matrix[2][1]) / scale;
        result.z = 0.25 * scale;
    }
    return result;
}

bool readVec3(const QJsonValue& value, Vec3& result)
{
    if (!value.isArray()) {
        return false;
    }
    const QJsonArray array = value.toArray();
    if (array.size() != 3) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        const QJsonValue item = array.at(index);
        if (!item.isDouble()) {
            return false;
        }
        result.value[index] = item.toDouble();
    }
    return true;
}

bool readPositiveInt(const QJsonValue& value, int& result)
{
    if (!value.isDouble()) {
        return false;
    }
    const double raw = value.toDouble();
    if (!(raw > 0.0)) {
        return false;
    }
    result = static_cast<int>(std::lround(raw));
    return result > 0;
}

struct SidecarPose
{
    Vec3 position;
    Vec3 target;
    Vec3 up;
    double fieldOfViewDeg = 0.0;
    int outputWidth = 0;
    int outputHeight = 0;
    int renderWidth = 0;
    int renderHeight = 0;
};

bool parseSidecar(const QJsonObject& root, const QString& expectedImage, SidecarPose& pose)
{
    if (root.value(QStringLiteral("image")).toString() != expectedImage) {
        return false;
    }
    const QJsonObject camera = root.value(QStringLiteral("camera")).toObject();
    if (!readVec3(camera.value(QStringLiteral("position")), pose.position)
        || !readVec3(camera.value(QStringLiteral("target")), pose.target)
        || !readVec3(camera.value(QStringLiteral("up")), pose.up)) {
        return false;
    }
    const QJsonValue fieldOfView = camera.value(QStringLiteral("fieldOfViewDeg"));
    if (!fieldOfView.isDouble() || !(fieldOfView.toDouble() > 0.0)) {
        return false;
    }
    pose.fieldOfViewDeg = fieldOfView.toDouble();

    const QJsonObject output = root.value(QStringLiteral("output")).toObject();
    if (!readPositiveInt(output.value(QStringLiteral("width")), pose.outputWidth)
        || !readPositiveInt(output.value(QStringLiteral("height")), pose.outputHeight)) {
        return false;
    }

    // Sidecars written before the render size was recorded fall back to square
    // pixels, which makes the estimated focal lengths only approximate.
    pose.renderWidth = pose.outputWidth;
    pose.renderHeight = pose.outputHeight;
    int renderWidth = 0;
    int renderHeight = 0;
    const QJsonObject render = root.value(QStringLiteral("render")).toObject();
    if (readPositiveInt(render.value(QStringLiteral("width")), renderWidth)
        && readPositiveInt(render.value(QStringLiteral("height")), renderHeight)) {
        pose.renderWidth = renderWidth;
        pose.renderHeight = renderHeight;
    }
    return true;
}

struct PreparedImage
{
    QString name;
    Vec3 position;
    Quaternion orientation;
    double focalX = 0.0;
    double focalY = 0.0;
    int width = 0;
    int height = 0;
    double fieldOfViewDeg = 0.0;
    int renderWidth = 0;
    int renderHeight = 0;
    QString cameraKey;
};

// Builds the camera-to-world rotation and the pinhole intrinsics of one shot.
bool prepareImage(const QString& name, const SidecarPose& pose, PreparedImage& image)
{
    Vec3 forward;
    for (int axis = 0; axis < 3; ++axis) {
        forward.value[axis] = pose.target.value[axis] - pose.position.value[axis];
    }
    if (!normalize(forward)) {
        return false; // the shot has no view direction
    }
    Vec3 right = cross(forward, pose.up);
    if (!normalize(right)) {
        return false; // the up vector is parallel to the view direction
    }
    Vec3 down = cross(forward, right);
    if (!normalize(down)) {
        return false;
    }

    // Column-wise camera basis: +X right, +Y down, +Z forward.
    const double rotation[3][3] = {
        {right.value[0], down.value[0], forward.value[0]},
        {right.value[1], down.value[1], forward.value[1]},
        {right.value[2], down.value[2], forward.value[2]},
    };

    image.name = name;
    image.position = pose.position;
    image.orientation = quaternionFromMatrix(rotation);
    image.width = pose.outputWidth;
    image.height = pose.outputHeight;
    image.fieldOfViewDeg = pose.fieldOfViewDeg;
    image.renderWidth = pose.renderWidth;
    image.renderHeight = pose.renderHeight;

    // The field of view is a vertical angle, and the captured image is stretched
    // from the render window to the output size, so the two focal lengths differ.
    image.focalY = (image.height / 2.0) / std::tan(pose.fieldOfViewDeg * kRadiansPerDegree / 2.0);
    image.focalX = image.focalY * (static_cast<double>(image.width) * image.renderHeight)
                   / (static_cast<double>(image.height) * image.renderWidth);
    return true;
}

QString cameraSignature(const PreparedImage& image)
{
    return QStringLiteral("%1x%2|%3x%4|%5")
        .arg(image.width)
        .arg(image.height)
        .arg(image.renderWidth)
        .arg(image.renderHeight)
        .arg(image.fieldOfViewDeg, 0, 'g', 17);
}

bool writeCameraInfo(const QString& path, const std::vector<PreparedImage>& images,
                     const std::vector<QString>& cameraKeys, QString& error)
{
    QJsonObject cameras;
    for (const QString& key : cameraKeys) {
        const PreparedImage* sample = nullptr;
        for (const PreparedImage& image : images) {
            if (image.cameraKey == key) {
                sample = &image;
                break;
            }
        }
        if (sample == nullptr) {
            continue;
        }
        QJsonArray params;
        params.append(sample->focalX);
        params.append(sample->focalY);
        params.append(sample->width / 2.0);
        params.append(sample->height / 2.0);

        QJsonObject camera;
        camera.insert(QStringLiteral("model"), QStringLiteral("PINHOLE"));
        camera.insert(QStringLiteral("width"), sample->width);
        camera.insert(QStringLiteral("height"), sample->height);
        camera.insert(QStringLiteral("params"), params);
        cameras.insert(key, camera);
    }

    QJsonArray entries;
    for (const PreparedImage& image : images) {
        QJsonArray qvec;
        qvec.append(image.orientation.w);
        qvec.append(image.orientation.x);
        qvec.append(image.orientation.y);
        qvec.append(image.orientation.z);

        QJsonArray position;
        for (double component : image.position.value) {
            position.append(component);
        }

        QJsonObject cameraToWorld;
        cameraToWorld.insert(QStringLiteral("qvec"), qvec);
        cameraToWorld.insert(QStringLiteral("position"), position);

        QJsonObject entry;
        entry.insert(QStringLiteral("name"), image.name);
        entry.insert(QStringLiteral("camera"), image.cameraKey);
        entry.insert(QStringLiteral("camera_to_world"), cameraToWorld);
        entries.append(entry);
    }

    QJsonObject root;
    root.insert(QStringLiteral("cameras"), cameras);
    root.insert(QStringLiteral("images"), entries);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        error = QStringLiteral("cannot open %1 for writing: %2").arg(path, file.errorString());
        return false;
    }
    if (file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) == -1) {
        error = QStringLiteral("cannot write %1").arg(path);
        return false;
    }
    return true;
}

} // namespace

QStringList CameraInfoBuilder::findImages(const QString& directory)
{
    const QDir dir(directory);
    QStringList names;
    const QStringList entries = dir.entryList(QDir::Files | QDir::NoDotAndDotDot);
    for (const QString& entry : entries) {
        if (imageSuffixes().contains(QFileInfo(entry).suffix().toLower())) {
            names.append(entry);
        }
    }
    names.sort();
    return names;
}

CameraInfoBuildResult CameraInfoBuilder::build(const QString& imageDirectory,
                                               const QString& outputJsonPath,
                                               QString& error)
{
    error.clear();

    CameraInfoBuildResult result;
    const QStringList names = findImages(imageDirectory);
    result.totalImages = names.size();
    result.cameraMode = QStringLiteral("estimated");

    if (names.size() < 2) {
        result.skipReason = QStringLiteral("图片数量不足");
        return result;
    }

    const QDir directory(imageDirectory);
    std::vector<PreparedImage> prepared;
    prepared.reserve(static_cast<std::size_t>(names.size()));
    int missing = 0;
    int unusable = 0;
    int orthographic = 0;

    for (const QString& name : names) {
        QFile file(directory.filePath(QFileInfo(name).completeBaseName() + QStringLiteral(".json")));
        if (!file.open(QIODevice::ReadOnly)) {
            ++missing;
            continue;
        }
        QJsonParseError parseError{};
        const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
        if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
            ++missing;
            continue;
        }
        const QJsonObject root = document.object();
        const QString projection =
            root.value(QStringLiteral("camera")).toObject().value(QStringLiteral("projection")).toString();
        if (projection.compare(QStringLiteral("perspective"), Qt::CaseInsensitive) != 0) {
            // An orthographic shot has no pinhole equivalent.
            if (projection.isEmpty()) {
                ++missing;
            } else {
                ++orthographic;
            }
            continue;
        }

        SidecarPose pose;
        PreparedImage image;
        if (!parseSidecar(root, name, pose) || !prepareImage(name, pose, image)) {
            ++unusable;
            continue;
        }
        prepared.push_back(image);
    }

    if (missing + unusable == names.size()) {
        result.skipReason = QStringLiteral("目录内没有拍摄元数据(sidecar)");
        return result;
    }
    if (missing + unusable > 0) {
        result.skipReason = QStringLiteral("部分图片缺少拍摄元数据");
        return result;
    }
    if (orthographic > 0) {
        result.skipReason = QStringLiteral("含正交投影(orthographic)照片");
        return result;
    }

    // One COLMAP camera per distinct set of intrinsics.
    std::vector<QString> cameraKeys;
    std::map<QString, QString> keyBySignature;
    for (PreparedImage& image : prepared) {
        const QString signature = cameraSignature(image);
        const auto existing = keyBySignature.find(signature);
        if (existing != keyBySignature.end()) {
            image.cameraKey = existing->second;
            continue;
        }
        const QString key = QStringLiteral("cam_%1").arg(cameraKeys.size());
        keyBySignature.emplace(signature, key);
        cameraKeys.push_back(key);
        image.cameraKey = key;
    }

    if (!writeCameraInfo(outputJsonPath, prepared, cameraKeys, error)) {
        return result;
    }

    result.built = true;
    result.posedImages = static_cast<int>(prepared.size());
    result.cameraMode = QStringLiteral("provided");
    return result;
}