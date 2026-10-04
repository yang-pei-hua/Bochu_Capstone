#include "app/MainWindow.h"
#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
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
        QString error;
        require(viewer->loadPointCloud(ply.fileName(), error), "load cloud into viewport");
        viewer->resetCameraToPointCloud();
        viewer->setStandardView(CameraController::StandardView::Top);
        application.processEvents();
        require(panel->isEnabled() && viewer->pointCloudSelectionMode() == 1, "loaded cloud enables point picking");
        auto* mode = panel->findChild<QComboBox*>("pointCloudSelectionMode");
        auto* radius = panel->findChild<QDoubleSpinBox*>("pointCloudNeighbourRadius");
        auto* expand = panel->findChild<QPushButton*>("expandPointCloudSelection");
        auto* assign = panel->findChild<QPushButton*>("assignPointCloudRegion");
        auto* regions = panel->findChild<QComboBox*>("pointCloudRegions");
        require(mode && radius && expand && assign && regions, "all selection controls available");
        require(!expand->isEnabled(), "empty selection disables expansion");
        // Pick a point that does not share a projection with another point.
        pick(viewer, 1, 0, 0);
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>{1}, "actual VTK point pick reports original ID");
        radius->setValue(1.01);
        expand->click();
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({0, 1, 2}), "panel button expands one round");
        expand->click();
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({0, 1, 2, 3}), "second panel click expands current set");
        assign->click();
        require(viewer->pointCloudSelection().regions().size() == 1
                && viewer->pointCloudSelection().regionPointCount(1) == 4, "panel assigns cuboid group");
        viewer->clearPointCloudSelection();
        pick(viewer, 1, 0, 0);
        pick(viewer, 3, 0, 0, true);
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({1, 3}), "Ctrl picking adds a second point");
        pick(viewer, 1, 0, 0, true);
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>{3}, "Ctrl picking removes selected point");
        mode->setCurrentIndex(2);
        viewer->selectPointCloudBox(false);
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({0, 1}), "3D box has finite depth and original IDs");
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
        require(viewer->pointCloudSelection().selected().size() == 2, "capture preserves selection box bounds");
        require(viewer->savePointCloudAnnotations(temp.filePath("regions.json"), error), "save viewer annotations");
        require(viewer->loadPointCloudAnnotations(temp.filePath("regions.json"), error), "reload viewer annotations");
        require(regions->count() == 2, "loaded groups refresh panel list");
        viewer->setSketchInteractionEnabled(true);
        require(viewer->pointCloudSelectionMode() == 0 && mode->currentIndex() == 0, "sketch entry exits point-cloud selection");
        mode->setCurrentIndex(2);
        require(viewer->pointCloudSelectionMode() == 0 && mode->currentIndex() == 0, "cloud mode cannot steal active sketch input");
        viewer->setSketchInteractionEnabled(false);
        require(viewer->loadPointCloud(ply.fileName(), error), "replace cloud");
        require(viewer->pointCloudSelection().regions().empty()
            && viewer->pointCloudSelection().selected().empty() && regions->count() == 1,
            "replacing cloud resets annotations, selection and region controls");

        auto* relationships = window.findChild<PointCloudRelationsPanel*>();
        auto* structure = window.findChild<QTabWidget*>("structureTabs");
        auto* tree = window.findChild<QTreeWidget*>("pointCloudRelationshipTree");
        require(relationships && tree && structure && structure->count() == 2
            && structure->tabText(1) == "CAD 特征", "left structure dock retains CAD features alongside point relationships");
        viewer->selectPointCloudPoint(0); viewer->assignPointCloudRegion("plane","顶面",0);
        viewer->selectPointCloudPoint(1); viewer->assignPointCloudRegion("plane","顶面",1);
        viewer->selectPointCloudPoint(2); viewer->assignPointCloudRegion("plane","侧面",0);
        viewer->selectPointCloudPoint(3); viewer->assignPointCloudRegion("plane","侧面",2);
        const int relation = viewer->setPointCloudRelation(0,1,2,"perpendicular");
        require(relation && findNode(tree,2,relation), "relation is visible in actual left tree");
        tree->setCurrentItem(findNode(tree,2,relation));
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({0,1,2,3}), "clicking relationship highlights both point collections");
        auto* removeRelation = window.findChild<QPushButton*>("removeCloudRelationButton");
        auto* undo = window.findChild<QPushButton*>("undoCloudEditButton");
        removeRelation->click();
        require(viewer->pointCloudSelection().relations().empty() && viewer->pointCloudPointCount() == 7,
            "delete relationship preserves collections and observations");
        undo->click();
        require(viewer->pointCloudSelection().relation(relation), "left undo restores relationship");
        tree->setCurrentItem(findNode(tree,1,1));
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>({0,1}), "collection click highlights source points");
        auto* points = findNode(tree,3,1); require(points,"point list exists"); points->setExpanded(true);
        require(points->childCount() == 2 && findNode(tree,4,0), "source point list loads lazily with stable IDs");
        window.findChild<QPushButton*>("deleteRegionPointsButton")->click();
        require(viewer->pointCloudPointCount() == 5 && viewer->workingPointCloud().positions.size() == 5,
            "left collection point deletion reaches rendering and CAD working input");
        undo->click(); require(viewer->pointCloudPointCount() == 7, "left undo restores removed points");
        tree->setCurrentItem(findNode(tree,1,1));
        window.findChild<QPushButton*>("removePointCollectionButton")->click();
        require(!viewer->pointCloudSelection().region(1) && viewer->pointCloudPointCount() == 7,
            "remove collection unassigns instead of deleting source points");
        undo->click();
        auto* pointId = window.findChild<QLineEdit*>("relationshipPointId");
        pointId->setText("1"); window.findChild<QPushButton*>("relationshipLocatePoint")->click();
        require(viewer->pointCloudSelection().selected() == std::vector<vtkIdType>{1}, "point ID lookup selects the source observation");
        window.findChild<QPushButton*>("deleteSelectedCloudPointsButton")->click();
        viewer->resetCameraToPointCloud(); viewer->setStandardView(CameraController::StandardView::Top);
        pick(viewer,1,0,0);
        require(viewer->pointCloudSelection().selected().empty(), "VTK picking excludes deleted vertices even when original IDs are retained");
        undo->click(); require(viewer->pointCloudPointCount() == 7,"undo selected point deletion");

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
        box.close(); require(viewer->loadPointCloud(box.fileName(),error),"load analysis fixture");
        auto* detect = window.findChild<QPushButton*>("detectCloudSurfacesButton");
        auto* status = window.findChild<QLabel*>("pointCloudRelationStatus");
        detect->click();
        viewer->selectPointCloudPoint(0); viewer->assignPointCloudRegion("plane","用户编辑",0);
        waitForAnalysis(status);
        require(status->text().contains("未应用") && viewer->pointCloudSelection().regions().size()==1,
            "concurrent user edits prevent stale background evidence from overwriting work");
        undo->click(); detect->click(); waitForAnalysis(status);
        require(viewer->pointCloudSelection().regions().size()==6 && !viewer->pointCloudSelection().relations().empty(),
            "background detection publishes fitted faces, original memberships and relationships to window");
        viewer->resetCameraToPointCloud(); application.processEvents();
        auto* firstFace = findNode(tree,1,viewer->pointCloudSelection().regions().front().id);
        firstFace->setExpanded(true); tree->setCurrentItem(firstFace);
        if(argc>1) require(window.grab().save(QString::fromLocal8Bit(argv[1])),"save relationship UI preview");
        viewer->clearPointCloud();
        require(!panel->isEnabled() && !relationships->isEnabled(), "clearing cloud disables both point-cloud panels");
        window.close();
        std::cout << "Point cloud interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
