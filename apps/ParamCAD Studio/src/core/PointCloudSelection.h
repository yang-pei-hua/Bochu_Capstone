#pragma once

#include "io/PlyReader.h"

#include <QString>
#include <QJsonObject>
#include <vtkSmartPointer.h>

#include <vector>

class vtkPlanes;
class vtkPolyData;
class vtkStaticPointLocator;

struct PointCloudRegion
{
    int id = 0;
    QString name;
    QString geometry; // Stable keys: cuboid, sphere, cylinder, cone, plane, ring.
    QJsonObject parameters; // Fitted equations in original cloud units.
    QString source = QStringLiteral("manual");
    double residual = 0.0;
    double confidence = 0.0;
    bool fitStale = false;
    bool templateLocked = false;
};

struct PointCloudRelation
{
    int id = 0;
    int first = 0;
    int second = 0;
    QString type;
    bool automatic = false;
    double confidence = 1.0;
};

struct DetectedPointCloudSurface
{
    PointCloudRegion surface;
    std::vector<vtkIdType> points;
};

// Selection and semantic annotations use original PLY point indices. Geometry
// Locked geometry labels guide CAD reconstruction; annotations never modify
// the source observations themselves.
class PointCloudSelection
{
public:
    PointCloudSelection();
    void setCloud(const PlyCloud& cloud);
    void adoptProject(PointCloudSelection&& prepared);
    void select(const std::vector<vtkIdType>& ids, bool add = false);
    void toggle(vtkIdType id);
    void selectBox(vtkPlanes* planes, bool add);
    void expand(double radius);
    int assign(const QString& geometry, const QString& name, int regionId = 0);
    void selectRegion(int regionId);
    void removeRegion(int regionId);
    void unassign();
    bool editRegion(int regionId, const QString& name, const QString& geometry);
    void replaceRegionPoints(int regionId);
    void updateRegionFit(int regionId, const PointCloudRegion& fitted, bool recordUndo = true);
    void lockTemplate(int regionId);
    void deleteRegionPoints(int regionId); // 0 means the unassigned collection.
    void deleteSelectedPoints();
    void selectUnassigned();
    void selectRelation(int relationId);
    void addSurfaces(const std::vector<DetectedPointCloudSurface>& surfaces, bool recordUndo = true);
    int setRelation(int relationId, int first, int second, const QString& type);
    void removeRelation(int relationId);
    void inferRelations(double distanceTolerance, double angleDegrees, bool recordUndo = true);
    bool undo();
    bool canUndo() const { return !m_undo.empty(); }
    bool active(vtkIdType id) const;
    int activePointCount() const;
    quint64 revision() const { return m_revision; }
    const PlyCloud& sourceCloud() const noexcept { return m_sourceCloud; }
    const std::vector<unsigned char>& deleted() const { return m_deleted; }
    const std::vector<PointCloudRelation>& relations() const { return m_relations; }
    const PointCloudRegion* region(int id) const;
    const PointCloudRelation* relation(int id) const;
    std::vector<vtkIdType> regionPoints(int regionId) const;
    PlyCloud workingCloud(std::vector<vtkIdType>* originalIds = nullptr,
                          bool unassignedOnly = false, int regionId = -1) const;
    bool exportCloud(const QString& path, QString& error) const;
    static bool validGeometry(const QString& geometry);
    static bool validRelation(const QString& type);

    const std::vector<vtkIdType>& selected() const { return m_selected; }
    const std::vector<int>& memberships() const { return m_memberships; }
    const std::vector<PointCloudRegion>& regions() const { return m_regions; }
    vtkPolyData* data() const;
    int regionPointCount(int regionId) const;
    bool save(const QString& path, const QString& sourcePath, QString& error) const;
    bool load(const QString& path, QString& error);

private:
    struct EditState {
        std::vector<int> memberships;
        std::vector<unsigned char> deleted;
        std::vector<PointCloudRegion> regions;
        std::vector<PointCloudRelation> relations;
        std::vector<QString> suppressed;
        std::vector<vtkIdType> selected;
        int nextRegionId = 1;
        int nextRelationId = 1;
    };
    void checkpoint(bool recordUndo = true);
    void rebuildLocator();
    void invalidateRegion(int id);
    void deletePoints(const std::vector<vtkIdType>& ids);
    vtkSmartPointer<vtkPolyData> m_data;
    vtkSmartPointer<vtkPolyData> m_activeData;
    vtkSmartPointer<vtkStaticPointLocator> m_locator;
    PlyCloud m_sourceCloud;
    std::vector<vtkIdType> m_activeIds;
    std::vector<unsigned char> m_deleted;
    std::vector<vtkIdType> m_selected;
    std::vector<int> m_memberships;
    std::vector<PointCloudRegion> m_regions;
    QString m_fingerprint;
    int m_nextRegionId = 1;
    int m_nextRelationId = 1;
    std::vector<PointCloudRelation> m_relations;
    std::vector<QString> m_suppressed;
    std::vector<EditState> m_undo;
    quint64 m_revision = 0;
};
