#include "core/PointCloudSelection.h"

#include "io/PlyWriter.h"
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

#include <algorithm>
#include <set>

bool PointCloudSelection::active(vtkIdType id) const
{
    return id >= 0 && static_cast<size_t>(id) < m_deleted.size() && !m_deleted[static_cast<size_t>(id)];
}
int PointCloudSelection::activePointCount() const { return static_cast<int>(m_activeIds.size()); }

const PointCloudRegion* PointCloudSelection::region(int id) const
{
    const auto found = std::find_if(m_regions.begin(), m_regions.end(), [id](const auto& r) { return r.id == id; });
    return found == m_regions.end() ? nullptr : &*found;
}
const PointCloudRelation* PointCloudSelection::relation(int id) const
{
    const auto found = std::find_if(m_relations.begin(), m_relations.end(), [id](const auto& r) { return r.id == id; });
    return found == m_relations.end() ? nullptr : &*found;
}

void PointCloudSelection::checkpoint()
{
    if (m_undo.size() >= 20) m_undo.erase(m_undo.begin());
    m_undo.push_back({m_memberships, m_deleted, m_regions, m_relations, m_suppressed,
                      m_selected, m_nextRegionId, m_nextRelationId});
    ++m_revision;
}

bool PointCloudSelection::undo()
{
    if (m_undo.empty()) return false;
    auto previous = std::move(m_undo.back());
    m_undo.pop_back();
    m_memberships = std::move(previous.memberships);
    m_deleted = std::move(previous.deleted);
    m_regions = std::move(previous.regions);
    m_relations = std::move(previous.relations);
    m_suppressed = std::move(previous.suppressed);
    m_selected = std::move(previous.selected);
    m_nextRegionId = previous.nextRegionId;
    m_nextRelationId = previous.nextRelationId;
    ++m_revision;
    rebuildLocator();
    return true;
}

void PointCloudSelection::rebuildLocator()
{
    vtkNew<vtkPoints> points;
    m_activeIds.clear();
    for (size_t i = 0; i < m_deleted.size(); ++i) {
        if (!m_deleted[i]) {
            points->InsertNextPoint(m_sourceCloud.positions[i].data());
            m_activeIds.push_back(static_cast<vtkIdType>(i));
        }
    }
    m_activeData->SetPoints(points);
    m_locator->Initialize();
    m_locator->SetDataSet(m_activeData);
    if (!m_activeIds.empty()) m_locator->BuildLocator();
}

void PointCloudSelection::invalidateRegion(int id)
{
    if (id <= 0) return;
    for (auto& r : m_regions) if (r.id == id && !r.parameters.isEmpty()) r.fitStale = true;
    // Inferred relationships no longer have valid evidence after membership
    // edits. User-authored relationships are kept as explicit annotations.
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(),
        [id](const auto& r) { return r.automatic && (r.first == id || r.second == id); }), m_relations.end());
}

bool PointCloudSelection::editRegion(int id, const QString& name, const QString& geometry)
{
    if (!region(id) || name.trimmed().isEmpty() || !validGeometry(geometry)) return false;
    checkpoint();
    for (auto& r : m_regions) if (r.id == id) {
        if (r.geometry != geometry) { r.parameters = {}; r.source = "manual"; r.fitStale = false; invalidateRegion(id); }
        r.geometry = geometry;
        r.name = name.trimmed();
    }
    return true;
}

std::vector<vtkIdType> PointCloudSelection::regionPoints(int id) const
{
    std::vector<vtkIdType> points;
    for (size_t i = 0; i < m_memberships.size(); ++i) {
        if (!m_deleted[i] && m_memberships[i] == id) points.push_back(static_cast<vtkIdType>(i));
    }
    return points;
}
void PointCloudSelection::replaceRegionPoints(int id)
{
    if (!region(id) || m_selected.empty()) return;
    checkpoint();
    invalidateRegion(id);
    for (int& membership : m_memberships) if (membership == id) membership = 0;
    for (vtkIdType point : m_selected) {
        invalidateRegion(m_memberships[static_cast<size_t>(point)]);
        m_memberships[static_cast<size_t>(point)] = id;
    }
}
void PointCloudSelection::updateRegionFit(int id, const PointCloudRegion& fitted)
{
    const auto* original = region(id);
    if (!original || original->geometry != fitted.geometry || fitted.parameters.isEmpty()) return;
    checkpoint();
    invalidateRegion(id);
    for (auto& r : m_regions) if (r.id == id) {
        r.parameters = fitted.parameters; r.residual = fitted.residual;
        r.confidence = fitted.confidence; r.source = "detected"; r.fitStale = false;
    }
}
void PointCloudSelection::selectUnassigned() { select(regionPoints(0)); }

void PointCloudSelection::selectRelation(int id)
{
    const auto* r = relation(id);
    if (!r) return;
    select(regionPoints(r->first));
    select(regionPoints(r->second), true);
}

void PointCloudSelection::deletePoints(const std::vector<vtkIdType>& ids)
{
    if (ids.empty()) return;
    checkpoint();
    std::set<int> affected;
    for (vtkIdType id : ids) if (active(id)) {
        const size_t index = static_cast<size_t>(id);
        affected.insert(m_memberships[index]);
        m_memberships[index] = 0;
        m_deleted[index] = 1;
    }
    for (int id : affected) invalidateRegion(id);
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(), [this](const auto& r) {
        return regionPointCount(r.first) == 0 || regionPointCount(r.second) == 0;
    }), m_relations.end());
    const auto remaining = m_selected;
    select(remaining);
    rebuildLocator();
}
void PointCloudSelection::deleteSelectedPoints() { const auto ids = m_selected; deletePoints(ids); }
void PointCloudSelection::deleteRegionPoints(int id) { deletePoints(regionPoints(id)); }

void PointCloudSelection::addSurfaces(const std::vector<DetectedPointCloudSurface>& surfaces)
{
    if (surfaces.empty()) return;
    checkpoint();
    for (const auto& detected : surfaces) {
        if (!validGeometry(detected.surface.geometry)) continue;
        std::vector<vtkIdType> ids;
        for (vtkIdType id : detected.points) {
            if (active(id) && m_memberships[static_cast<size_t>(id)] == 0) ids.push_back(id);
        }
        if (ids.empty()) continue;
        auto surface = detected.surface;
        surface.id = m_nextRegionId++;
        surface.source = "detected";
        m_regions.push_back(surface);
        for (vtkIdType id : ids) m_memberships[static_cast<size_t>(id)] = surface.id;
    }
}

PlyCloud PointCloudSelection::workingCloud(std::vector<vtkIdType>* originalIds, bool unassignedOnly, int regionId) const
{
    PlyCloud result;
    if (originalIds) originalIds->clear();
    const bool normals = m_sourceCloud.normals.size() == m_sourceCloud.positions.size();
    const bool colors = m_sourceCloud.colors.size() == m_sourceCloud.positions.size();
    for (vtkIdType id : m_activeIds) {
        const size_t i = static_cast<size_t>(id);
        if (unassignedOnly && m_memberships[i] != 0) continue;
        if (regionId >= 0 && m_memberships[i] != regionId) continue;
        result.positions.push_back(m_sourceCloud.positions[i]);
        if (normals) result.normals.push_back(m_sourceCloud.normals[i]);
        if (colors) result.colors.push_back(m_sourceCloud.colors[i]);
        if (originalIds) originalIds->push_back(id);
    }
    return result;
}

bool PointCloudSelection::exportCloud(const QString& path, QString& error) const
{
    return PlyWriter::write(workingCloud(), path, error);
}
