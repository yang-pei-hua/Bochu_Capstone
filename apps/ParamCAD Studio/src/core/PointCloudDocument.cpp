#include "core/PointCloudDocument.h"
#include "io/PointCloudMetadata.h"

#include <QCoreApplication>

PointCloudDocument::PointCloudDocument(QObject* parent) : QObject(parent) {}

bool PointCloudDocument::load(const QString& path, QString& error)
{
    PlyCloud cloud;
    if (!PlyReader::read(path, cloud, error)) return false;
    if (cloud.positions.empty()) {
        error = QCoreApplication::translate("VTKViewer", "Point cloud is empty: %1").arg(path);
        return false;
    }
    m_selection.setCloud(cloud);
    m_sourcePath = path;
    m_sourceUnit = pointCloudUnitName(path);
    emit cloudChanged();
    notifyEdit();
    return true;
}

void PointCloudDocument::clear()
{
    m_selection.setCloud(PlyCloud{});
    m_sourcePath.clear();
    m_sourceUnit = QStringLiteral("arbitrary");
    emit cloudChanged();
    notifyEdit();
}

void PointCloudDocument::adoptProject(PointCloudDocument&& prepared)
{
    m_selection.adoptProject(std::move(prepared.m_selection));
    m_sourcePath = std::move(prepared.m_sourcePath);
    m_sourceUnit = std::move(prepared.m_sourceUnit);
    emit cloudChanged();
    notifyEdit();
}

PlyCloud PointCloudDocument::workingCloud(std::vector<vtkIdType>* ids,
                                        bool unassignedOnly, int regionId) const
{
    return m_selection.workingCloud(ids, unassignedOnly, regionId);
}

bool PointCloudDocument::saveAnnotations(const QString& path, QString& error) const
{
    return m_selection.save(path, m_sourcePath, error);
}

bool PointCloudDocument::loadAnnotations(const QString& path, QString& error)
{
    if (!m_selection.load(path, error)) return false;
    notifyEdit();
    return true;
}

bool PointCloudDocument::exportCloud(const QString& path, QString& error) const
{
    return m_selection.exportCloud(path, error);
}

void PointCloudDocument::notifyEdit()
{
    emit documentChanged();
    emit selectionChanged();
}

void PointCloudDocument::select(const std::vector<vtkIdType>& ids, bool add)
{
    m_selection.select(ids, add);
    emit selectionChanged();
}
void PointCloudDocument::toggle(vtkIdType id)
{
    m_selection.toggle(id);
    emit selectionChanged();
}
void PointCloudDocument::selectBox(vtkPlanes* planes, bool add)
{
    m_selection.selectBox(planes, add);
    emit selectionChanged();
}
void PointCloudDocument::expandSelection(double radius)
{
    m_selection.expand(radius);
    emit selectionChanged();
}
void PointCloudDocument::clearSelection() { select({}); }
void PointCloudDocument::selectRegion(int id)
{
    m_selection.selectRegion(id);
    emit selectionChanged();
}
void PointCloudDocument::selectUnassigned()
{
    m_selection.selectUnassigned();
    emit selectionChanged();
}
void PointCloudDocument::selectPoint(vtkIdType id) { select({id}); }
void PointCloudDocument::selectRelation(int id)
{
    m_selection.selectRelation(id);
    emit selectionChanged();
}

int PointCloudDocument::assignRegion(const QString& geometry, const QString& name, int id)
{
    const int result = m_selection.assign(geometry, name, id);
    notifyEdit();
    return result;
}
void PointCloudDocument::removeRegion(int id)
{
    m_selection.removeRegion(id);
    notifyEdit();
}
void PointCloudDocument::unassignSelection()
{
    m_selection.unassign();
    notifyEdit();
}
bool PointCloudDocument::editRegion(int id, const QString& name, const QString& geometry)
{
    const bool result = m_selection.editRegion(id, name, geometry);
    notifyEdit();
    return result;
}
void PointCloudDocument::replaceRegionPoints(int id)
{
    m_selection.replaceRegionPoints(id);
    notifyEdit();
}
void PointCloudDocument::updateRegionFit(int id, const PointCloudRegion& fitted, bool recordUndo)
{
    m_selection.updateRegionFit(id, fitted, recordUndo);
    notifyEdit();
}
void PointCloudDocument::deleteRegionPoints(int id)
{
    m_selection.deleteRegionPoints(id);
    notifyEdit();
}
void PointCloudDocument::lockTemplate(int id)
{
    m_selection.lockTemplate(id);
    notifyEdit();
}
void PointCloudDocument::deleteSelectedPoints()
{
    m_selection.deleteSelectedPoints();
    notifyEdit();
}
int PointCloudDocument::setRelation(int id, int first, int second, const QString& type)
{
    const int result = m_selection.setRelation(id, first, second, type);
    notifyEdit();
    return result;
}
void PointCloudDocument::removeRelation(int id)
{
    m_selection.removeRelation(id);
    notifyEdit();
}
void PointCloudDocument::inferRelations(double distanceTolerance, double angleDegrees, bool recordUndo)
{
    m_selection.inferRelations(distanceTolerance, angleDegrees, recordUndo);
    notifyEdit();
}
void PointCloudDocument::addSurfaces(const std::vector<DetectedPointCloudSurface>& surfaces, bool recordUndo)
{
    m_selection.addSurfaces(surfaces, recordUndo);
    notifyEdit();
}
bool PointCloudDocument::undo()
{
    if (!m_selection.undo()) return false;
    notifyEdit();
    return true;
}
void PointCloudDocument::replacePartCandidates(const std::vector<DetectedPointCloudSurface>& parts,bool recordUndo)
{
    m_selection.replacePartCandidates(parts,recordUndo);
    notifyEdit();
}
