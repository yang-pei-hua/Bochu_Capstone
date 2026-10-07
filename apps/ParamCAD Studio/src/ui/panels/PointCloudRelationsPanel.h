#pragma once
#include "core/PointCloudSelection.h"
#include <QWidget>
#include <map>

class QAction;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QMenu;
class QPushButton;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

class PointCloudRelationsPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudRelationsPanel(QWidget* parent = nullptr);
    void insertSelectionPage(QWidget* page);
    QMenu* createSelectionMenu(QWidget* parent);
    void setSourceInfo(const QString& path, const QString& unit);
    void setDocument(const PointCloudSelection& document, bool available);
    void updateSelection();
    void setBusy(bool busy);
    void setCadReconstructionBusy(bool busy, bool cancellable);
    void setStatus(const QString& text);
    void setDefaultTolerances(double diagonal);
    double surfaceTolerance() const;
    double relationTolerance() const;
    double angleTolerance() const;

signals:
    void detectRequested();
    void reconstructRequested();
    void cancelCadRequested();
    void inferRequested();
    void refitRequested(int id);
    void regionSelected(int id);
    void relationSelected(int id);
    void pointSelected(vtkIdType id);
    void assignRequested(const QString& geometry, const QString& name, int id);
    void unassignRequested();
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
    void createRegion();
    void editRegion();
    void editRelation(bool create);
    void showContextMenu(QTreeWidget* tree, const QPoint& position);
    void loadPointPage(QTreeWidgetItem* item, int offset);
    void applyFilter();
    const PointCloudSelection* m_document = nullptr;
    std::map<int, std::vector<vtkIdType>> m_points;
    bool m_busy = false;
    QTabWidget* m_pages;
    QTreeWidget* m_tree;
    QTreeWidget* m_relationTree;
    QTreeWidget* m_activeTree;
    QLabel* m_source;
    QLabel* m_summary;
    QLabel* m_selection;
    QLabel* m_status;
    QLineEdit* m_filter;
    QLineEdit* m_relationFilter;
    QDoubleSpinBox* m_surfaceTolerance;
    QDoubleSpinBox* m_relationTolerance;
    QDoubleSpinBox* m_angleTolerance;
    QPushButton* m_detect;
    QPushButton* m_cancelCad;
    QPushButton* m_infer;
    QToolButton* m_reconstruct;
    QAction* m_reconstructAction;
    QAction* m_newRegion;
    QAction* m_unassign;
    QAction* m_refit;
    QAction* m_editRegion;
    QAction* m_append;
    QAction* m_replace;
    QAction* m_removeRegion;
    QAction* m_deleteRegionPoints;
    QAction* m_deleteSelected;
    QAction* m_addRelation;
    QAction* m_editRelation;
    QAction* m_removeRelation;
    QAction* m_undo;
};
