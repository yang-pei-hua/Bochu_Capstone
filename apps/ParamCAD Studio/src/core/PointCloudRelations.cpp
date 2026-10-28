#include "core/PointCloudSelection.h"
#include "io/SurfaceEvidenceAdapter.h"

#include <vtkPolyData.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace {
QString key(int first, int second, const QString& type)
{
    return QStringLiteral("%1:%2:%3").arg(std::min(first, second)).arg(std::max(first, second)).arg(type);
}
} // namespace

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
    std::vector<reconstruction::PointSample> observations;
    for (size_t i=0; i<m_memberships.size(); ++i) {
        if (m_deleted[i]) continue;
        double p[3]; m_data->GetPoint(static_cast<vtkIdType>(i), p);
        reconstruction::PointSample sample; sample.id=i+1; sample.position={p[0],p[1],p[2]};
        observations.push_back(sample);
    }
    std::vector<reconstruction::RelationSurface> nodes;
    for (const auto& region : m_regions) {
        reconstruction::RelationSurface node;
        node.id=region.id; node.evidence=regionEvidence(region);
        for(auto id : points[region.id]) node.supportPointIds.push_back(id+1);
        nodes.push_back(std::move(node));
    }
    reconstruction::SurfaceRelationOptions options;
    options.distanceTolerance=tolerance;
    options.angularToleranceRadians=angleDegrees*3.14159265358979323846/180;
    std::vector<reconstruction::SurfaceRelation> inferred;
    std::string error;
    if(!reconstruction::inferSurfaceRelations(reconstruction::PointStore(std::move(observations)),
            nodes,options,inferred,error)) return;
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
    for (const auto& r : inferred) {
        const QString type=QString::fromLatin1(reconstruction::surfaceRelationName(r.kind));
        // The UI preserves its existing vocabulary; axis-plane relations are
        // nevertheless available to semantic recognizers through core analysis.
        if (!validRelation(type)) continue;
        const auto* a=region(static_cast<int>(r.first));
        const auto* b=region(static_cast<int>(r.second));
        if (a && b) append(*a,*b,type,r.confidence);
    }
}
