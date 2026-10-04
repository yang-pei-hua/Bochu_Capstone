#pragma once

#include "core/PointCloudSelection.h"
#include <QObject>

// GUI-thread document. Owns source observations, selection, annotations, edit
// history and their revision; no widgets, render actors or camera state.
// PointCloudSelection remains the spatial selection/edit engine.
class PointCloudDocument final : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudDocument(QObject* parent = nullptr);
    bool load(const QString& path, QString& error);
    void clear();
    void adoptProject(PointCloudDocument&& prepared);
    bool hasCloud() const noexcept { return !sourceCloud().positions.empty(); }
    int pointCount() const noexcept { return m_selection.activePointCount(); }
    const QString& sourcePath() const noexcept { return m_sourcePath; }
    const QString& sourceUnit() const noexcept { return m_sourceUnit; }
    const PlyCloud& sourceCloud() const noexcept { return m_selection.sourceCloud(); }
    const PointCloudSelection& selection() const noexcept { return m_selection; }
    quint64 revision() const noexcept { return m_selection.revision(); }
    PlyCloud workingCloud(std::vector<vtkIdType>* ids = nullptr,
                          bool unassignedOnly = false, int regionId = -1) const;
    bool saveAnnotations(const QString& path, QString& error) const;
    bool loadAnnotations(const QString& path, QString& error);
    bool exportCloud(const QString& path, QString& error) const;

    void select(const std::vector<vtkIdType>& ids, bool add = false);
    void toggle(vtkIdType id);
    void selectBox(vtkPlanes* planes, bool add);
    void expandSelection(double radius);
    void clearSelection();
    void selectRegion(int id);
    void selectUnassigned();
    void selectPoint(vtkIdType id);
    void selectRelation(int id);
    int assignRegion(const QString& geometry, const QString& name, int id = 0);
    void removeRegion(int id);
    void unassignSelection();
    bool editRegion(int id, const QString& name, const QString& geometry);
    void replaceRegionPoints(int id);
    void updateRegionFit(int id, const PointCloudRegion& fitted);
    void deleteRegionPoints(int id);
    void deleteSelectedPoints();
    int setRelation(int id, int first, int second, const QString& type);
    void removeRelation(int id);
    void inferRelations(double distanceTolerance, double angleDegrees);
    void addSurfaces(const std::vector<DetectedPointCloudSurface>& surfaces);
    bool undo();

signals:
    // Emitted first after load/clear, allowing views to replace render data
    // before panels inspect the new document. Failed loads emit no signals.
    void cloudChanged();
    void documentChanged();
    void selectionChanged();

private:
    void notifyEdit();
    PointCloudSelection m_selection;
    QString m_sourcePath;
    QString m_sourceUnit = QStringLiteral("arbitrary");
};
