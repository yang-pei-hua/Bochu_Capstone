#include "core/PointCloudSelection.h"

#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPlanes.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

bool PointCloudSelection::validGeometry(const QString& geometry)
{
    return geometry == "cuboid" || geometry == "sphere" || geometry == "cylinder"
        || geometry == "plane" || geometry == "ring" || geometry == "cone";
}

PointCloudSelection::PointCloudSelection()
    : m_data(vtkSmartPointer<vtkPolyData>::New())
    , m_activeData(vtkSmartPointer<vtkPolyData>::New())
    , m_locator(vtkSmartPointer<vtkStaticPointLocator>::New())
{
}

void PointCloudSelection::setCloud(const PlyCloud& cloud)
{
    ++m_revision;
    m_sourceCloud = cloud;
    m_deleted.assign(cloud.positions.size(), 0);
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(cloud.positions.size()));
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        points->SetPoint(i, cloud.positions[static_cast<size_t>(i)].data());
    }
    m_data->SetPoints(points);
    for (size_t i = 0; i < cloud.positions.size(); ++i) {
        const auto& p = cloud.positions[i];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) m_deleted[i] = 1;
    }
    rebuildLocator();
    QCryptographicHash hash(QCryptographicHash::Sha256);
    for (const auto& position : cloud.positions) {
        hash.addData(QByteArrayView(reinterpret_cast<const char*>(position.data()),
                                    3 * sizeof(float)));
    }
    m_fingerprint = QString::fromLatin1(hash.result().toHex());
    m_selected.clear();
    m_regions.clear();
    m_memberships.assign(cloud.positions.size(), 0);
    m_nextRegionId = 1;
    m_nextRelationId = 1;
    m_relations.clear();
    m_suppressed.clear();
    m_undo.clear();
}

void PointCloudSelection::select(const std::vector<vtkIdType>& ids, bool add)
{
    if (!add) m_selected.clear();
    for (vtkIdType id : ids) {
        if (active(id)) m_selected.push_back(id);
    }
    std::sort(m_selected.begin(), m_selected.end());
    m_selected.erase(std::unique(m_selected.begin(), m_selected.end()), m_selected.end());
}

void PointCloudSelection::adoptProject(PointCloudSelection&& prepared)
{
    const auto nextRevision = m_revision + 1;
    *this = std::move(prepared);
    m_revision = nextRevision;
    m_undo.clear();
}

void PointCloudSelection::toggle(vtkIdType id)
{
    if (!active(id)) return;
    const auto found = std::lower_bound(m_selected.begin(), m_selected.end(), id);
    if (found != m_selected.end() && *found == id) m_selected.erase(found);
    else m_selected.insert(found, id);
}

void PointCloudSelection::selectBox(vtkPlanes* planes, bool add)
{
    if (!planes) return;
    std::vector<vtkIdType> inside;
    double point[3];
    for (vtkIdType i = 0; i < m_data->GetNumberOfPoints(); ++i) {
        m_data->GetPoint(i, point);
        if (active(i) && planes->FunctionValue(point) <= 0.0) inside.push_back(i);
    }
    select(inside, add);
}

void PointCloudSelection::expand(double radius)
{
    if (!std::isfinite(radius) || radius <= 0.0 || m_selected.empty()
        || m_selected.size() == m_activeIds.size()) return;
    // Freeze the seeds: newly discovered points participate only on the next
    // button press. A chain of neighbours must never flood in a single step.
    const auto seeds = m_selected;
    std::vector<unsigned char> included(m_memberships.size(), 0);
    for (vtkIdType id : seeds) included[static_cast<size_t>(id)] = 1;
    size_t includedCount = seeds.size();
    vtkNew<vtkIdList> neighbours;
    double point[3];
    for (vtkIdType id : seeds) {
        m_data->GetPoint(id, point);
        m_locator->FindPointsWithinRadius(radius, point, neighbours);
        for (vtkIdType j = 0; j < neighbours->GetNumberOfIds(); ++j) {
            auto& flag = included[static_cast<size_t>(m_activeIds[static_cast<size_t>(neighbours->GetId(j))])];
            if (!flag) { flag = 1; ++includedCount; }
        }
        if (includedCount == m_activeIds.size()) break;
    }
    m_selected.clear();
    for (size_t i = 0; i < included.size(); ++i) {
        if (included[i]) m_selected.push_back(static_cast<vtkIdType>(i));
    }
}

int PointCloudSelection::assign(const QString& geometry, const QString& name, int regionId)
{
    if (m_selected.empty() || !validGeometry(geometry)) return 0;
    auto found = std::find_if(m_regions.begin(), m_regions.end(),
                             [regionId](const auto& r) { return r.id == regionId; });
    if (regionId != 0 && found == m_regions.end()) return 0;
    checkpoint();
    if (regionId == 0) {
        regionId = m_nextRegionId++;
        m_regions.push_back({regionId, name.trimmed().isEmpty()
            ? QStringLiteral("%1 %2").arg(geometry).arg(regionId) : name.trimmed(), geometry});
    }
    for (vtkIdType id : m_selected) {
        const auto index = static_cast<size_t>(id);
        if (m_memberships[index] != regionId) invalidateRegion(m_memberships[index]);
        m_memberships[index] = regionId;
    }
    invalidateRegion(regionId);
    for (auto& region : m_regions) if (region.id == regionId) {
        region.templateLocked = true; region.source = "manual";
    }
    return regionId;
}

void PointCloudSelection::selectRegion(int regionId)
{
    m_selected.clear();
    if (regionId <= 0) return;
    for (size_t i = 0; i < m_memberships.size(); ++i) {
        if (active(static_cast<vtkIdType>(i)) && m_memberships[i] == regionId) m_selected.push_back(static_cast<vtkIdType>(i));
    }
}

void PointCloudSelection::removeRegion(int regionId)
{
    if (!region(regionId)) return;
    checkpoint();
    for (int& id : m_memberships) if (id == regionId) id = 0;
    m_regions.erase(std::remove_if(m_regions.begin(), m_regions.end(),
                                  [regionId](const auto& r) { return r.id == regionId; }), m_regions.end());
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(),
        [regionId](const auto& r) { return r.first == regionId || r.second == regionId; }), m_relations.end());
}

void PointCloudSelection::unassign()
{
    if (m_selected.empty()) return;
    checkpoint();
    for (vtkIdType id : m_selected) {
        invalidateRegion(m_memberships[static_cast<size_t>(id)]);
        m_memberships[static_cast<size_t>(id)] = 0;
    }
}

vtkPolyData* PointCloudSelection::data() const { return m_data; }

int PointCloudSelection::regionPointCount(int regionId) const
{
    int count = 0;
    for (size_t i = 0; i < m_memberships.size(); ++i) {
        if (!m_deleted[i] && m_memberships[i] == regionId) ++count;
    }
    return count;
}

bool PointCloudSelection::save(const QString& path, const QString& sourcePath, QString& error) const
{
    QJsonArray regions;
    for (const auto& region : m_regions) {
        QJsonArray ids;
        for (size_t i = 0; i < m_memberships.size(); ++i) {
            if (m_memberships[i] == region.id) ids.append(static_cast<qint64>(i));
        }
        regions.append(QJsonObject{{"id", region.id}, {"name", region.name},
            {"geometry", region.geometry}, {"pointIndices", ids}, {"parameters", region.parameters},
            {"source", region.source}, {"residual", region.residual}, {"confidence", region.confidence},
            {"fitStale", region.fitStale}, {"templateLocked", region.templateLocked}});
    }
    QJsonArray deleted, relations, suppressed;
    for (size_t i = 0; i < m_deleted.size(); ++i) if (m_deleted[i]) deleted.append(static_cast<qint64>(i));
    for (const auto& r : m_relations) relations.append(QJsonObject{{"id", r.id}, {"first", r.first},
        {"second", r.second}, {"type", r.type}, {"automatic", r.automatic}, {"confidence", r.confidence}});
    for (const auto& key : m_suppressed) suppressed.append(key);
    const QJsonDocument document(QJsonObject{{"format", "paramcad-point-regions"}, {"version", 2},
        {"source", sourcePath}, {"pointCount", static_cast<qint64>(m_memberships.size())},
        {"positionsSha256", m_fingerprint}, {"regions", regions}, {"deletedPointIndices", deleted},
        {"relations", relations}, {"suppressedRelations", suppressed},
        {"nextRegionId", m_nextRegionId}, {"nextRelationId", m_nextRelationId}});
    QSaveFile file(path);
    const auto bytes = document.toJson();
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) {
        error = file.errorString();
        return false;
    }
    return true;
}

bool PointCloudSelection::load(const QString& path, QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) { error = file.errorString(); return false; }
    QJsonParseError parseError;
    const auto document = QJsonDocument::fromJson(file.readAll(), &parseError);
    const auto root = document.object();
    const int version = root["version"].toInt();
    if (parseError.error != QJsonParseError::NoError || !document.isObject()
        || root["format"] != "paramcad-point-regions" || (version != 1 && version != 2)
        || root["pointCount"].toInteger(-1) != static_cast<qint64>(m_memberships.size())
        || root["positionsSha256"].toString() != m_fingerprint || !root["regions"].isArray()) {
        error = QStringLiteral("标注格式无效，或点云坐标/点顺序与当前点云不一致。");
        return false;
    }
    // Validate into temporary state so a bad import cannot damage current work.
    std::vector<PointCloudRegion> regions;
    std::vector<int> memberships(m_memberships.size(), 0);
    std::vector<unsigned char> deleted(m_memberships.size(), 0);
    for (size_t i = 0; i < m_sourceCloud.positions.size(); ++i) {
        const auto& p = m_sourceCloud.positions[i];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) deleted[i] = 1;
    }
    if (version == 2) {
        if (!root["deletedPointIndices"].isArray() || !root["relations"].isArray()
            || !root["suppressedRelations"].isArray()) {
            error = QStringLiteral("标注缺少点删除状态或关系数据。"); return false;
        }
        std::set<qint64> deletedIds;
        for (const auto& value : root["deletedPointIndices"].toArray()) {
            const qint64 id = value.toInteger(-1);
            if (!value.isDouble() || value.toDouble() != static_cast<double>(id) || id < 0
                || id >= static_cast<qint64>(deleted.size()) || !deletedIds.insert(id).second) {
                error = QStringLiteral("无效的已删除点索引。"); return false;
            }
            deleted[static_cast<size_t>(id)] = 1;
        }
    }
    std::set<int> seen;
    int nextId = 1;
    for (const auto& value : root["regions"].toArray()) {
        const auto region = value.toObject();
        const int id = region["id"].toInt();
        if (id <= 0 || id == std::numeric_limits<int>::max() || !seen.insert(id).second
            || !validGeometry(region["geometry"].toString()) || !region["name"].isString()
            || !region["pointIndices"].isArray()) {
            error = QStringLiteral("标注包含无效或重复的分组。"); return false;
        }
        for (const auto& index : region["pointIndices"].toArray()) {
            const qint64 point = index.toInteger(-1);
            if (!index.isDouble() || index.toDouble() != static_cast<double>(point)
                || point < 0 || point >= static_cast<qint64>(memberships.size())
                || memberships[static_cast<size_t>(point)] != 0 || deleted[static_cast<size_t>(point)]) {
                error = QStringLiteral("标注包含无效或重复的点索引。"); return false;
            }
            memberships[static_cast<size_t>(point)] = id;
        }
        PointCloudRegion surface{id, region["name"].toString(), region["geometry"].toString()};
        surface.templateLocked = region.contains("templateLocked")
            ? region["templateLocked"].toBool() : region["source"].toString("manual") == "manual";
        if (region.contains("templateLocked") && !region["templateLocked"].isBool()) {
            error = QStringLiteral("无效的模板锁定状态。"); return false;
        }
        if (version == 2) {
            surface.parameters = region["parameters"].toObject();
            surface.source = region["source"].toString();
            surface.residual = region["residual"].toDouble(-1);
            surface.confidence = region["confidence"].toDouble(-1);
            surface.fitStale = region["fitStale"].toBool();
            if (!region["parameters"].isObject() || !region["fitStale"].isBool()
                || (surface.source != "manual" && surface.source != "detected")
                || !std::isfinite(surface.residual) || surface.residual < 0
                || !std::isfinite(surface.confidence) || surface.confidence < 0 || surface.confidence > 1) {
                error = QStringLiteral("无效的表面拟合信息。"); return false;
            }
            if (!surface.parameters.isEmpty()) {
                const auto validVector = [&](const char* key, bool unit) {
                    const auto v = surface.parameters[key].toArray();
                    if (v.size() != 3) return false;
                    double squared = 0;
                    for (const auto& component : v) {
                        if (!component.isDouble() || !std::isfinite(component.toDouble())) return false;
                        squared += component.toDouble() * component.toDouble();
                    }
                    return !unit || std::abs(squared - 1.0) <= 1.0e-5;
                };
                const auto validNumber = [&](const char* key, bool positive) {
                    const auto n = surface.parameters[key];
                    return n.isDouble() && std::isfinite(n.toDouble()) && (!positive || n.toDouble() > 0);
                };
                bool valid = false;
                if (surface.geometry == "plane") valid = validVector("normal", true) && validNumber("offset", false);
                if (surface.geometry == "sphere") valid = validVector("center", false) && validNumber("radius", true);
                if (surface.geometry == "cylinder") valid = validVector("center", false) && validVector("axis", true) && validNumber("radius", true);
                if (surface.geometry == "ring") valid = validVector("center", false) && validVector("axis", true)
                    && validNumber("major_radius", true) && validNumber("minor_radius", true);
                if (surface.geometry == "cone") valid = validVector("center", false) && validVector("axis", true)
                    && validNumber("angle", true) && surface.parameters["angle"].toDouble() < 1.5707963267948966;
                if (surface.geometry == "cuboid") valid = validVector("origin", false)
                    && validVector("x_direction", true) && validVector("z_direction", true)
                    && validNumber("size_x", true) && validNumber("size_y", true) && validNumber("size_z", true);
                if (!valid) { error = QStringLiteral("无效的表面参数。"); return false; }
            }
        }
        regions.push_back(surface);
        nextId = std::max(nextId, id + 1);
    }
    std::vector<PointCloudRelation> relations;
    std::vector<QString> suppressed;
    std::set<int> relationIds;
    std::set<QString> relationKeys;
    int nextRelationId = 1;
    if (version == 2) {
        for (const auto& value : root["relations"].toArray()) {
            const auto object = value.toObject();
            PointCloudRelation relation{object["id"].toInt(), object["first"].toInt(), object["second"].toInt(),
                object["type"].toString(), object["automatic"].toBool(), object["confidence"].toDouble(-1)};
            const auto key = QStringLiteral("%1:%2:%3").arg(std::min(relation.first, relation.second))
                .arg(std::max(relation.first, relation.second)).arg(relation.type);
            if (relation.id <= 0 || relation.id == std::numeric_limits<int>::max()
                || !relationIds.insert(relation.id).second || !relationKeys.insert(key).second
                || !seen.count(relation.first) || !seen.count(relation.second) || relation.first == relation.second
                || !validRelation(relation.type) || !object["automatic"].isBool()
                || !std::isfinite(relation.confidence) || relation.confidence < 0 || relation.confidence > 1) {
                error = QStringLiteral("关系包含无效端点、类型或重复项。"); return false;
            }
            nextRelationId = std::max(nextRelationId, relation.id + 1);
            relations.push_back(relation);
        }
        for (const auto& value : root["suppressedRelations"].toArray()) {
            const auto parts = value.toString().split(':');
            if (!value.isString() || parts.size() != 3 || parts[0].toInt() <= 0
                || parts[1].toInt() <= parts[0].toInt() || !validRelation(parts[2])) {
                error = QStringLiteral("无效的已删除关系记录。"); return false;
            }
            suppressed.push_back(value.toString());
            // Old version 2 documents did not store ID counters. Retain IDs
            // mentioned by suppression records so a new group cannot inherit
            // a deleted group's relationships after reloading.
            if (parts[1].toInt() == std::numeric_limits<int>::max()) {
                error = QStringLiteral("无效的已删除关系记录。"); return false;
            }
            nextId = std::max(nextId, parts[1].toInt() + 1);
        }
        const auto counter = [&](const char* key, int& next) {
            if (!root.contains(key)) return true;
            const int saved = root[key].toInt(-1);
            if (saved < next || saved == std::numeric_limits<int>::max()) return false;
            next = saved; return true;
        };
        if (!counter("nextRegionId", nextId) || !counter("nextRelationId", nextRelationId)) {
            error = QStringLiteral("无效的集合或关系编号状态。"); return false;
        }
    }
    checkpoint();
    m_regions = std::move(regions);
    m_memberships = std::move(memberships);
    m_nextRegionId = nextId;
    m_nextRelationId = nextRelationId;
    m_relations = std::move(relations);
    m_suppressed = std::move(suppressed);
    m_deleted = std::move(deleted);
    m_selected.clear();
    rebuildLocator();
    return true;
}
