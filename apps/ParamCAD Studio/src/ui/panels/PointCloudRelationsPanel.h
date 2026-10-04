#pragma once

#include "core/PointCloudSelection.h"
#include <QWidget>
#include <map>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

class PointCloudRelationsPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudRelationsPanel(QWidget* parent = nullptr);
    void setDocument(const PointCloudSelection& document, bool available);
    void updateSelection();
    void setBusy(bool busy);
    void setStatus(const QString& text);
    void setDefaultTolerances(double diagonal);
    double surfaceTolerance() const;
    double relationTolerance() const;
    double angleTolerance() const;

signals:
    void detectRequested();
    void inferRequested();
    void refitRequested(int id);
    void regionSelected(int id);
    void relationSelected(int id);
    void pointSelected(vtkIdType id);
    void editRegionRequested(int id, const QString& name, const QString& geometry);
    void appendSelectionRequested(int id);
    void replaceSelectionRequested(int id);
    void removeRegionRequested(int id);
    void deleteRegionPointsRequested(int id);
    void deleteSelectedPointsRequested();
    void setRelationRequested(int id, int first, int second, const QString& type);
    void removeRelationRequested(int id);
    void undoRequested();
    void saveRequested();
    void loadRequested();
    void exportRequested();

private:
    int selectedRegion() const;
    int selectedRelation() const;
    void updateActions();
    void editRegion();
    void editRelation(bool create);
    void loadPointPage(QTreeWidgetItem* item, int offset);
    void applyFilter();
    const PointCloudSelection* m_document = nullptr;
    std::map<int, std::vector<vtkIdType>> m_points;
    bool m_busy = false;
    QTreeWidget* m_tree;
    QLabel* m_summary;
    QLabel* m_selection;
    QLabel* m_status;
    QLineEdit* m_filter;
    QLineEdit* m_pointId;
    QDoubleSpinBox* m_surfaceTolerance;
    QDoubleSpinBox* m_relationTolerance;
    QDoubleSpinBox* m_angleTolerance;
    QPushButton* m_detect;
    QPushButton* m_infer;
    QPushButton* m_refit;
    QPushButton* m_editRegion;
    QPushButton* m_append;
    QPushButton* m_replace;
    QPushButton* m_removeRegion;
    QPushButton* m_deleteRegionPoints;
    QPushButton* m_deleteSelected;
    QPushButton* m_addRelation;
    QPushButton* m_editRelation;
    QPushButton* m_removeRelation;
    QPushButton* m_undo;
};
