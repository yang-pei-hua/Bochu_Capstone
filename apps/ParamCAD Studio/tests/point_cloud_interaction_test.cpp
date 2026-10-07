#include "app/MainWindow.h"
#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QApplication>
#include <QAction>
#include <QDialog>
#include <QMenu>
#include <QTimer>
#include <QDockWidget>
#include <QToolButton>
#include "ui/panels/PropertyPanel.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QTabWidget>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
bool changed(const double a[3], const double b[3])
{
    return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]) > 1.0e-8;
}
void pick(VTKViewer* viewer, double x, double y, double z, bool control = false)
{
    auto* renderer = viewer->renderWindow()->GetRenderers()->GetFirstRenderer();
    renderer->SetWorldPoint(x, y, z, 1);
    renderer->WorldToDisplay();
    const double* position = renderer->GetDisplayPoint();
    auto* interactor = viewer->renderWindow()->GetInteractor();
    interactor->SetEventInformation(static_cast<int>(std::round(position[0])),
        static_cast<int>(std::round(position[1])), control ? 1 : 0, 0);
    interactor->LeftButtonPressEvent();
    interactor->LeftButtonReleaseEvent();
}
QTreeWidgetItem* findNode(QTreeWidget* tree, int kind, qint64 id)
{
    QTreeWidgetItemIterator items(tree);
    while (*items) {
        if ((*items)->data(0,Qt::UserRole).toInt() == kind && (*items)->data(0,Qt::UserRole+1).toLongLong() == id) return *items;
        ++items;
    }
    return nullptr;
}
void createCollection(PointCloudRelationsPanel* panel, const QString& geometry)
{
    bool accepted = false;
    QTimer::singleShot(0, panel, [&] {
        auto* dialog = panel->findChild<QDialog*>("createCloudCollectionDialog");
        if (!dialog) return;
        dialog->setAttribute(Qt::WA_DontShowOnScreen);
        auto* type = dialog->findChild<QComboBox*>("pointCloudTemplateGeometry");
        type->setCurrentIndex(type->findData(geometry));
        accepted = true; dialog->accept();
    });
    panel->findChild<QAction*>("createPointCollectionAction")->trigger();
    require(accepted, "collection dialog accepted through menu action");
}
void contextAction(QTreeWidget* tree, QTreeWidgetItem* item, const char* name, const QString& newName = {})
{
    require(item, "context menu target exists");
    auto* pages = tree->window()->findChild<QTabWidget*>("pointCloudPages");
    pages->setCurrentWidget(tree->parentWidget());
    tree->scrollToItem(item);
    QCoreApplication::processEvents();
    bool invoked = false;
    QTimer::singleShot(0, tree, [&] {
        auto* menu = qobject_cast<QMenu*>(QApplication::activePopupWidget());
        if (!menu) return;
        menu->setAttribute(Qt::WA_DontShowOnScreen);
        for (auto* command : menu->actions()) if (command->objectName() == name && command->isEnabled()) {
            invoked = true; menu->close();
            if (!newName.isEmpty()) QTimer::singleShot(0, tree, [&] {
                auto* dialog = tree->window()->findChild<QDialog*>("editCloudCollectionDialog");
                if (!dialog) return;
                dialog->setAttribute(Qt::WA_DontShowOnScreen);
                dialog->findChild<QLineEdit*>("pointCloudCollectionName")->setText(newName);
                dialog->accept();
            });
            command->trigger(); return;
        }
        menu->close();
    });
    tree->customContextMenuRequested(tree->visualItemRect(item).center());
    require(invoked, "right-click menu exposes enabled command for its target");
}
void chooseCloudFile(MainWindow& window, const char* name, const QString& path)
{
    bool accepted = false;
    QTimer::singleShot(0, &window, [&] {
        auto* dialog = window.findChild<QFileDialog*>();
        if (!dialog) return;
        dialog->setAttribute(Qt::WA_DontShowOnScreen);
        dialog->selectFile(path); accepted = true;
        QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
    });
    window.findChild<QAction*>(name)->trigger();
    require(accepted, "cloud file menu opens and accepts its native operation");
}
void waitForAnalysis(QLabel* status)
{
    QElapsedTimer elapsed; elapsed.start();
    while (status->text().startsWith(QStringLiteral("正在后台")) && elapsed.elapsed() < 30000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,50); QThread::msleep(5);
    }
    require(!status->text().startsWith(QStringLiteral("正在后台")), "background analysis completed within deadline");
}
}

int main(int argc, char** argv)
{
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication application(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory");
        QFile ply(temp.filePath("cloud.ply"));
        require(ply.open(QIODevice::WriteOnly), "create cloud fixture");
        ply.write("ply\nformat ascii 1.0\nelement vertex 7\nproperty float x\n"
                  "property float y\nproperty float z\nend_header\n"
                  "0 0 0\n1 0 0\n2 0 0\n3 0 0\n-3 -3 -3\n3 3 3\n0 0 2\n");
        ply.close();
        MainWindow window;
        // Initialize a real Qt/VTK rendering surface without covering the user's
        // desktop or taking keyboard focus during automated verification.
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        application.processEvents();
        auto* viewer = window.findChild<VTKViewer*>();
        auto* panel = window.findChild<PointCloudPanel*>();
        require(viewer && panel && !panel->isEnabled(), "empty cloud disables panel");
        auto* document = window.findChild<PointCloudDocument*>();
        require(document && document == viewer->pointCloudDocument(),
                "composition root supplies one document to the viewport and workspace");
        QString error;
        require(viewer->pointCloudDocument()->load(ply.fileName(), error), "load cloud into viewport");
        viewer->resetCameraToPointCloud();
        viewer->setStandardView(CameraController::StandardView::Top);
        application.processEvents();
        require(panel->isEnabled() && viewer->pointCloudSelectionMode() == 1, "loaded cloud enables point picking");
        auto* mode = panel->findChild<QComboBox*>("pointCloudSelectionMode");
        auto* radius = panel->findChild<QDoubleSpinBox*>("pointCloudNeighbourRadius");
        auto* expand = panel->findChild<QPushButton*>("expandPointCloudSelection");
        auto* relationships = panel->relationsPanel();
        auto* pages = panel->findChild<QTabWidget*>("pointCloudPages");
        auto* tree = panel->findChild<QTreeWidget*>("pointCloudRelationshipTree");
        auto* relationTree = panel->findChild<QTreeWidget*>("cloudRelationsTree");
        auto* property = window.findChild<PropertyPanel*>();
        auto* cadDock = window.findChild<QDockWidget*>("modelingDock");
        require(mode && radius && expand && relationships && pages && tree && relationTree, "unified selection and collection controls available");
        require(property->isAncestorOf(relationships) && pages->count() == 4
                && pages->tabText(0) == "选点" && pages->tabText(1) == "集合"
                && pages->tabText(2) == "关系" && pages->tabText(3) == "设置",
                "all point-cloud pages belong to Properties");
        require(!window.findChild<QTabWidget*>("structureTabs") && !cadDock->isAncestorOf(relationships),
                "left CAD dock contains no duplicate point-cloud panel");
        pages->setCurrentIndex(0);
        require(!expand->isEnabled(), "empty selection disables expansion");
        // Pick a point that does not share a projection with another point.
        pick(viewer, 1, 0, 0);
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>{1}, "actual VTK point pick reports original ID");
        radius->setValue(1.01);
        expand->click();
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({0, 1, 2}), "panel button expands one round");
        expand->click();
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({0, 1, 2, 3}), "second panel click expands current set");
        createCollection(relationships, "cuboid");
        require(viewer->pointCloudDocument()->selection().regions().size() == 1
                && viewer->pointCloudDocument()->selection().regionPointCount(1) == 4, "panel assigns cuboid group");
        viewer->pointCloudDocument()->clearSelection();
        pick(viewer, 1, 0, 0);
        pick(viewer, 3, 0, 0, true);
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({1, 3}), "Ctrl picking adds a second point");
        pick(viewer, 1, 0, 0, true);
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>{3}, "Ctrl picking removes selected point");
        mode->setCurrentIndex(2);
        viewer->selectPointCloudBox(false);
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({0, 1}), "3D box has finite depth and original IDs");
        // Box mode must preserve both camera gestures even over the box.
        const auto cameraBefore = viewer->cameraParameters();
        auto* interactor = viewer->renderWindow()->GetInteractor();
        const int* size = viewer->renderWindow()->GetSize();
        interactor->SetEventInformation(size[0] / 2, size[1] / 2);
        interactor->MiddleButtonPressEvent();
        interactor->SetEventInformation(size[0] / 2 + 40, size[1] / 2 + 20);
        interactor->MouseMoveEvent();
        interactor->MiddleButtonReleaseEvent();
        const auto cameraRotated = viewer->cameraParameters();
        require(changed(cameraRotated.position, cameraBefore.position), "middle drag rotates camera in box mode");
        interactor->SetEventInformation(size[0] / 2, size[1] / 2);
        interactor->RightButtonPressEvent();
        interactor->SetEventInformation(size[0] / 2 + 40, size[1] / 2 + 20);
        interactor->MouseMoveEvent();
        interactor->RightButtonReleaseEvent();
        require(changed(viewer->cameraParameters().target, cameraRotated.target)
            && viewer->pointCloudSelectionMode() == 2, "right drag pans without cancelling box mode");
        if (argc > 1) {
            require(window.grab().save(QString::fromLocal8Bit(argv[1])), "save UI preview");
        }

        const auto photo = temp.filePath("capture.png");
        require(viewer->captureImage(photo, 640, 480), "capture while box selection is active");
        require(!QImage(photo).isNull() && viewer->pointCloudSelectionMode() == 2,
                "capture succeeds and restores box mode");
        viewer->selectPointCloudBox(false);
        require(viewer->pointCloudDocument()->selection().selected().size() == 2, "capture preserves selection box bounds");
        require(viewer->pointCloudDocument()->saveAnnotations(temp.filePath("regions.json"), error), "save viewer annotations");
        require(viewer->pointCloudDocument()->loadAnnotations(temp.filePath("regions.json"), error), "reload viewer annotations");
        require(findNode(tree, 1, 1), "loaded groups refresh the single collection tree");
        viewer->setSketchInteractionEnabled(true);
        require(viewer->pointCloudSelectionMode() == 0 && mode->currentIndex() == 0, "sketch entry exits point-cloud selection");
        mode->setCurrentIndex(2);
        require(viewer->pointCloudSelectionMode() == 0 && mode->currentIndex() == 0, "cloud mode cannot steal active sketch input");
        viewer->setSketchInteractionEnabled(false);
        require(viewer->pointCloudDocument()->load(ply.fileName(), error), "replace cloud");
        require(viewer->pointCloudDocument()->selection().regions().empty()
            && viewer->pointCloudDocument()->selection().selected().empty() && !findNode(tree,1,1),
            "replacing cloud resets annotations, selection and region controls");

        viewer->pointCloudDocument()->selectPoint(0); viewer->pointCloudDocument()->assignRegion("plane","顶面",0);
        viewer->pointCloudDocument()->selectPoint(1); viewer->pointCloudDocument()->assignRegion("plane","顶面",1);
        viewer->pointCloudDocument()->selectPoint(2); viewer->pointCloudDocument()->assignRegion("plane","侧面",0);
        viewer->pointCloudDocument()->selectPoint(3); viewer->pointCloudDocument()->assignRegion("plane","侧面",2);
        const int relation = viewer->pointCloudDocument()->setRelation(0,1,2,"perpendicular");
        require(relation && findNode(relationTree,2,relation), "relation is visible in Properties relationship tree");
        auto* collectionFilter = panel->findChild<QLineEdit*>("cloudCollectionFilter");
        auto* relationFilter = panel->findChild<QLineEdit*>("cloudRelationFilter");
        require(collectionFilter && relationFilter, "each list exposes its own search");
        collectionFilter->setText("不存在的集合");
        require(findNode(tree,1,1)->isHidden() && !findNode(relationTree,2,relation)->isHidden(),
            "collection search does not hide relationships on another page");
        collectionFilter->clear();
        relationFilter->setText("不存在的关系");
        require(findNode(relationTree,2,relation)->isHidden() && !findNode(tree,1,1)->isHidden(),
            "relationship search does not hide collections on another page");
        relationFilter->clear();
        relationTree->setCurrentItem(findNode(relationTree,2,relation));
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({0,1,2,3}), "clicking relationship highlights both point collections");
        auto* undo = window.findChild<QAction*>("undoCloudEditAction");
        contextAction(relationTree, findNode(relationTree,2,relation), "removeCloudRelationAction");
        require(viewer->pointCloudDocument()->selection().relations().empty() && viewer->pointCloudDocument()->pointCount() == 7,
            "delete relationship preserves collections and observations");
        undo->trigger();
        require(viewer->pointCloudDocument()->selection().relation(relation), "left undo restores relationship");
        tree->setCurrentItem(findNode(tree,1,1));
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>({0,1}), "collection click highlights source points");
        auto* points = findNode(tree,3,1); require(points,"point list exists"); points->setExpanded(true);
        require(points->childCount() == 2 && findNode(tree,4,0), "source point list loads lazily with stable IDs");
        contextAction(tree, findNode(tree,1,1), "deleteRegionPointsAction");
        require(viewer->pointCloudDocument()->pointCount() == 5 && viewer->pointCloudDocument()->workingCloud().positions.size() == 5,
            "left collection point deletion reaches rendering and CAD working input");
        undo->trigger(); require(viewer->pointCloudDocument()->pointCount() == 7, "left undo restores removed points");
        tree->setCurrentItem(findNode(tree,1,1));
        contextAction(tree, findNode(tree,1,1), "removePointCollectionAction");
        require(!viewer->pointCloudDocument()->selection().region(1) && viewer->pointCloudDocument()->pointCount() == 7,
            "remove collection unassigns instead of deleting source points");
        undo->trigger();
        document->selectPoint(4);
        contextAction(tree, findNode(tree,1,1), "appendCloudSelectionAction");
        require(document->selection().regionPointCount(1) == 3
                && document->selection().memberships()[4] == 1
                && document->selection().selected() == std::vector<vtkIdType>{4},
                "right-click append preserves the picked 3D point instead of selecting the whole target");
        undo->trigger();
        document->selectPoint(4);
        contextAction(tree, findNode(tree,1,1), "replaceCloudCollectionAction");
        require(document->selection().regionPoints(1) == std::vector<vtkIdType>{4}
                && document->selection().memberships()[0] == 0,
                "right-click replacement uses the current picked point and releases old members");
        undo->trigger();
        contextAction(tree, findNode(tree,1,1), "editPointCollectionAction", "重命名的顶面");
        require(document->selection().region(1)->name == "重命名的顶面"
                && tree->currentItem()->data(0,Qt::UserRole+1).toInt() == 1,
                "collection rename keeps the editing target by original collection ID");
        undo->trigger();
        auto* pointId = window.findChild<QLineEdit*>("relationshipPointId");
        pointId->setText("1"); pointId->returnPressed();
        require(viewer->pointCloudDocument()->selection().selected() == std::vector<vtkIdType>{1}, "point ID lookup selects the source observation");
        window.findChild<QAction*>("deleteSelectedCloudPointsAction")->trigger();
        viewer->resetCameraToPointCloud(); viewer->setStandardView(CameraController::StandardView::Top);
        pick(viewer,1,0,0);
        require(viewer->pointCloudDocument()->selection().selected().empty(), "VTK picking excludes deleted vertices even when original IDs are retained");
        undo->trigger(); require(viewer->pointCloudDocument()->pointCount() == 7,"undo selected point deletion");

        // Run the real asynchronous controller on a closed six-face fixture.
        QFile box(temp.filePath("box.ply")); require(box.open(QIODevice::WriteOnly),"create face fixture");
        box.write("ply\nformat ascii 1.0\nelement vertex 600\nproperty float x\nproperty float y\nproperty float z\n"
                  "property float nx\nproperty float ny\nproperty float nz\nend_header\n");
        for (int axis=0;axis<3;++axis) for (float sign : {-1.0F,1.0F}) for(int i=0;i<10;++i) for(int j=0;j<10;++j) {
            float p[3]{},n[3]{}; p[axis]=3*sign; n[axis]=sign;
            p[(axis+1)%3]=-3+(i+0.5F)*0.6F; p[(axis+2)%3]=-3+(j+0.5F)*0.6F;
            const auto line = QStringLiteral("%1 %2 %3 %4 %5 %6\n").arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]);
            box.write(line.toUtf8());
        }
        box.close(); require(viewer->pointCloudDocument()->load(box.fileName(),error),"load analysis fixture");
        auto* detect = window.findChild<QPushButton*>("detectCloudSurfacesButton");
        auto* status = window.findChild<QLabel*>("pointCloudRelationStatus");
        detect->click();
        viewer->pointCloudDocument()->selectPoint(0); viewer->pointCloudDocument()->assignRegion("plane","用户编辑",0);
        waitForAnalysis(status);
        require(status->text().contains("未应用") && viewer->pointCloudDocument()->selection().regions().size()==1,
            "concurrent user edits prevent stale background evidence from overwriting work");
        undo->trigger(); detect->click(); waitForAnalysis(status);
        require(viewer->pointCloudDocument()->selection().regions().size()==6 && !viewer->pointCloudDocument()->selection().relations().empty(),
            "background detection publishes fitted faces, original memberships and relationships to window");
        const auto annotationPath = temp.filePath("workspace-annotations.json");
        chooseCloudFile(window, "saveCloudAnnotationsAction", annotationPath);
        require(QFile::exists(annotationPath), "More menu saves cloud annotation JSON");
        document->removeRegion(document->selection().regions().front().id);
        chooseCloudFile(window, "loadCloudAnnotationsAction", annotationPath);
        require(document->selection().regions().size() == 6,
                "More menu restores fitted collections from matching original cloud");
        const auto exportedPath = temp.filePath("working-cloud.ply");
        chooseCloudFile(window, "exportWorkingCloudAction", exportedPath);
        PlyCloud exported;
        require(PlyReader::read(exportedPath, exported, error)
                && exported.positions.size() == 600
                && QFile::exists(temp.filePath("working-cloud.json")),
                "More menu exports working observations with unit sidecar");
        viewer->resetCameraToPointCloud(); application.processEvents();
        auto* firstFace = findNode(tree,1,viewer->pointCloudDocument()->selection().regions().front().id);
        firstFace->setExpanded(true); tree->setCurrentItem(firstFace);
        pages->setCurrentIndex(1);
        if(argc>1) require(window.grab().save(QString::fromLocal8Bit(argv[1])),"save relationship UI preview");
        viewer->pointCloudDocument()->clear();
        require(!panel->isEnabled() && !relationships->isEnabled(), "clearing cloud disables both point-cloud panels");
        require(document->sourcePath().isEmpty(), "clear resets the shared source identity");
        window.close();

        // A document can be edited before any view exists and survives a view.
        PointCloudDocument detached;
        require(detached.load(ply.fileName(), error), "load detached document");
        detached.selectPoint(1);
        detached.deleteSelectedPoints();
        {
            VTKViewer attached(nullptr, &detached);
            attached.setAttribute(Qt::WA_DontShowOnScreen);
            attached.show();
            application.processEvents();
            attached.resetCameraToPointCloud();
            attached.setStandardView(CameraController::StandardView::Top);
            pick(&attached, 1, 0, 0);
            require(detached.selection().selected().empty(), "new view respects preexisting deletion mask");
            require(detached.undo() && detached.pointCount() == 7, "document undo reaches attached view");
            pick(&attached, 1, 0, 0);
            require(detached.selection().selected() == std::vector<vtkIdType>{1},
                    "attached view picks restored original observation");
        }
        require(detached.hasCloud() && detached.pointCount() == 7,
                "destroying a borrowed view preserves document data");
        std::cout << "Point cloud interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
