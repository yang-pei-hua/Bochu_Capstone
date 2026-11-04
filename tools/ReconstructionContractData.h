#pragma once

#include <reconstruction/data/PointStore.h>
#include <reconstruction/recognition/ProjectionExtrusion.h>
#include <QJsonArray>
#include <QJsonObject>

#include <cmath>
#include <stdexcept>
#include <unordered_map>

struct PlyCloud;

namespace contract {
using reconstruction::PointId;
using reconstruction::PointSample;
using reconstruction::PointStore;
using modeling::Vec3;

inline void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

inline QJsonArray vector(Vec3 v) { return {v.x, v.y, v.z}; }

inline Vec3 vector(const QJsonValue& value) {
    const auto a = value.toArray();
    require(a.size() == 3 && a[0].isDouble() && a[1].isDouble() && a[2].isDouble(), "Expected a 3D vector");
    return {a[0].toDouble(), a[1].toDouble(), a[2].toDouble()};
}

inline QJsonArray ids(const std::vector<PointId>& values) {
    QJsonArray result;
    for (const auto id : values) result.append(QString::number(id));
    return result;
}

inline std::vector<PointId> ids(const QJsonValue& value) {
    require(value.isArray(), "Expected point IDs");
    std::vector<PointId> result;
    for (const auto& entry : value.toArray()) {
        bool ok = false;
        const auto id = entry.toString().toULongLong(&ok);
        require(ok && id, "Point IDs must be nonzero decimal strings");
        result.push_back(id);
    }
    return result;
}

using PointIndex = std::unordered_map<PointId, const PointSample*>;
inline PointIndex pointIndex(const std::vector<PointSample>& points) {
    PointIndex index;
    for (const auto& p : points) index.emplace(p.id, &p);
    return index;
}

inline std::vector<PointSample> observations(const QJsonValue& value, const PointIndex& index) {
    std::vector<PointSample> result;
    for (const auto id : ids(value)) {
        const auto found = index.find(id);
        require(found != index.end(), "Artifact references a point absent from its source");
        result.push_back(*found->second);
    }
    return result;
}

QJsonObject analysisStage(int module, const reconstruction::ObservationSnapshot& source,
    const QJsonObject& options, QJsonObject input, QJsonObject& metrics, std::size_t skipped);

QJsonObject assemblyStage(int module, const PointStore& source, const QJsonObject& options,
    QJsonObject input, QJsonObject& metrics, bool backfill);
QJsonObject projectionStage(int module, const PointStore& source, const QJsonObject& options,
    QJsonObject input, QJsonObject& metrics);
std::vector<reconstruction::ProjectionOperation> readProjectionProgram(const QJsonValue& value);
QJsonObject modelingStage(int module, QJsonObject input, QJsonObject& metrics, const QString& artifactPath);
QJsonObject workflowStage(PlyCloud cloud, reconstruction::LengthUnit unit, const QJsonObject& settings,
    QJsonObject input, QJsonObject& metrics);
} // namespace contract
