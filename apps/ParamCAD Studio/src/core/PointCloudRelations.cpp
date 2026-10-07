#include "core/PointCloudSelection.h"

#include <QJsonArray>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace {
using Vector = std::array<double, 3>;
Vector vector(const QJsonObject& object, const char* key)
{
    const auto array = object[key].toArray();
    return {array[0].toDouble(), array[1].toDouble(), array[2].toDouble()};
}
double dot(const Vector& a, const Vector& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
Vector subtract(const Vector& a, const Vector& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
double length(const Vector& a) { return std::sqrt(dot(a, a)); }
QString key(int first, int second, const QString& type)
{
    return QStringLiteral("%1:%2:%3").arg(std::min(first, second)).arg(std::max(first, second)).arg(type);
}
bool axisSurface(const PointCloudRegion& r)
{
    return r.geometry == "cylinder" || r.geometry == "cone" || r.geometry == "ring";
}
}

bool PointCloudSelection::validRelation(const QString& type)
{
    return type == "parallel" || type == "perpendicular" || type == "adjacent"
        || type == "intersect" || type == "coaxial" || type == "tangent"
        || type == "same_radius" || type == "coplanar";
}

int PointCloudSelection::setRelation(int id, int first, int second, const QString& type)
{
    if (!region(first) || !region(second) || first == second || !validRelation(type)
        || regionPointCount(first) == 0 || regionPointCount(second) == 0 || (id && !relation(id))) return 0;
    const QString newKey = key(first, second, type);
    for (const auto& r : m_relations) {
        if (r.id != id && key(r.first, r.second, r.type) == newKey) return 0;
    }
    checkpoint();
    if (id) {
        for (auto& r : m_relations) if (r.id == id) {
            const auto oldKey = key(r.first, r.second, r.type);
            if (oldKey != newKey) m_suppressed.push_back(oldKey);
            r = {id, std::min(first, second), std::max(first, second), type, false, 1.0};
        }
    } else {
        id = m_nextRelationId++;
        m_relations.push_back({id, std::min(first, second), std::max(first, second), type, false, 1.0});
    }
    m_suppressed.erase(std::remove(m_suppressed.begin(), m_suppressed.end(), newKey), m_suppressed.end());
    return id;
}

void PointCloudSelection::removeRelation(int id)
{
    const auto* r = relation(id);
    if (!r) return;
    const QString removed = key(r->first, r->second, r->type);
    checkpoint();
    m_suppressed.push_back(removed);
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(),
                                   [id](const auto& value) { return value.id == id; }), m_relations.end());
}

void PointCloudSelection::inferRelations(double tolerance, double angleDegrees, bool recordUndo)
{
    if (!std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(angleDegrees)
        || angleDegrees <= 0 || angleDegrees > 30) return;
    checkpoint(recordUndo);
    std::map<QString, int> previousIds;
    for (const auto& r : m_relations) previousIds[key(r.first, r.second, r.type)] = r.id;
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(),
                                   [](const auto& r) { return r.automatic; }), m_relations.end());
    std::map<int, std::vector<vtkIdType>> points;
    for (size_t i = 0; i < m_memberships.size(); ++i) {
        if (!m_deleted[i] && m_memberships[i]) points[m_memberships[i]].push_back(static_cast<vtkIdType>(i));
    }
    std::map<int, vtkSmartPointer<vtkStaticPointLocator>> locators;
    for (const auto& [id, ids] : points) {
        vtkNew<vtkPoints> coordinates;
        for (vtkIdType point : ids) coordinates->InsertNextPoint(m_data->GetPoint(point));
        vtkNew<vtkPolyData> data;
        data->SetPoints(coordinates);
        auto locator = vtkSmartPointer<vtkStaticPointLocator>::New();
        locator->SetDataSet(data); locator->BuildLocator();
        locators[id] = locator;
    }
    const double radians = angleDegrees * 3.14159265358979323846 / 180.0;
    const double parallelDot = std::cos(radians), perpendicularDot = std::sin(radians);
    const auto append = [&](const PointCloudRegion& a, const PointCloudRegion& b, const QString& type, double confidence) {
        const auto relationKey = key(a.id, b.id, type);
        if (std::find(m_suppressed.begin(), m_suppressed.end(), relationKey) != m_suppressed.end()) return;
        for (const auto& existing : m_relations) {
            if (key(existing.first, existing.second, existing.type) == relationKey) return;
        }
        const auto previous = previousIds.find(relationKey);
        const int id = previous == previousIds.end() ? m_nextRelationId++ : previous->second;
        m_relations.push_back({id, a.id, b.id, type, true, confidence});
    };
    for (size_t i = 0; i < m_regions.size(); ++i) for (size_t j = i + 1; j < m_regions.size(); ++j) {
        const auto& a = m_regions[i]; const auto& b = m_regions[j];
        if (points[a.id].empty() || points[b.id].empty()) continue;
        // Finite observed patches, not intersections of their infinite surfaces.
        // Every point in the smaller patch is queried to avoid missing a short
        // contact edge. Limit each search to the adjacency tolerance: finding
        // an unrestricted nearest point on a distant dense patch can scan most
        // locator buckets for every query and block the GUI thread for minutes.
        const auto& smaller = points[a.id].size() < points[b.id].size() ? points[a.id] : points[b.id];
        auto locator = points[a.id].size() < points[b.id].size() ? locators[b.id] : locators[a.id];
        bool adjacent = false;
        for (vtkIdType pointId : smaller) {
            double p[3], distanceSquared;
            m_data->GetPoint(pointId, p);
            if (locator->FindClosestPointWithinRadius(tolerance, p, distanceSquared) >= 0) {
                adjacent = true;
                break;
            }
        }
        if (adjacent) append(a, b, "adjacent", 0.75);
        if (a.fitStale || b.fitStale || a.parameters.isEmpty() || b.parameters.isEmpty()) continue;
        const double confidence = std::min({a.confidence, b.confidence, 0.95});
        if (a.geometry == "plane" && b.geometry == "plane") {
            const double alignment = dot(vector(a.parameters, "normal"), vector(b.parameters, "normal"));
            if (std::abs(alignment) >= parallelDot) {
                append(a, b, "parallel", confidence);
                const double distance = std::abs(a.parameters["offset"].toDouble()
                    - (alignment < 0 ? -1.0 : 1.0)*b.parameters["offset"].toDouble());
                if (distance <= tolerance) append(a, b, "coplanar", confidence);
            } else {
                if (std::abs(alignment) <= perpendicularDot) append(a, b, "perpendicular", confidence);
                if (adjacent) append(a, b, "intersect", confidence * 0.8);
            }
        }
        if (axisSurface(a) && axisSurface(b)) {
            const auto u = vector(a.parameters, "axis"), v = vector(b.parameters, "axis");
            if (std::abs(dot(u, v)) >= parallelDot) {
                const auto delta = subtract(vector(a.parameters, "center"), vector(b.parameters, "center"));
                const double projection = dot(delta, u);
                if (std::sqrt(std::max(0.0, dot(delta, delta)-projection*projection)) <= tolerance)
                    append(a, b, "coaxial", confidence);
            }
        }
        if (a.geometry == b.geometry && (a.geometry == "sphere" || a.geometry == "cylinder")
            && std::abs(a.parameters["radius"].toDouble()-b.parameters["radius"].toDouble()) <= tolerance)
            append(a, b, "same_radius", confidence);
        const auto tangentToPlane = [&](const PointCloudRegion& plane, const PointCloudRegion& curved) {
            if (plane.geometry != "plane" || (curved.geometry != "sphere" && curved.geometry != "cylinder")) return false;
            const auto normal = vector(plane.parameters, "normal");
            if (curved.geometry == "cylinder" && std::abs(dot(normal, vector(curved.parameters, "axis"))) > perpendicularDot) return false;
            const double distance = std::abs(dot(normal, vector(curved.parameters, "center")) + plane.parameters["offset"].toDouble());
            return std::abs(distance-curved.parameters["radius"].toDouble()) <= tolerance;
        };
        bool tangent = tangentToPlane(a, b) || tangentToPlane(b, a);
        if (a.geometry == "sphere" && b.geometry == "sphere") {
            const double separation = length(subtract(vector(a.parameters, "center"), vector(b.parameters, "center")));
            const double r1 = a.parameters["radius"].toDouble(), r2 = b.parameters["radius"].toDouble();
            tangent = std::abs(separation-r1-r2) <= tolerance
                || (separation > tolerance && std::abs(separation-std::abs(r1-r2)) <= tolerance);
        }
        if (tangent && adjacent) append(a, b, "tangent", confidence * 0.8);
    }
}
