#include "app/MainWindow.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/ProjectController.h"
#include "core/PointCloudDocument.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <QAction>
#include <QApplication>
#include <QContextMenuEvent>
#include <QFile>
#include <QMenu>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QTimer>
#include <QTreeWidget>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

QTreeWidgetItem* node(QTreeWidget* tree, SceneNodeType type)
{
    QTreeWidgetItemIterator items(tree);
    while (*items) {
        if ((*items)->data(0, Qt::UserRole + 1).toInt() == static_cast<int>(type)) return *items;
        ++items;
    }
    throw std::runtime_error("missing scene node");
}

void contextAction(QTreeWidget* tree, SceneNodeType type, const QString& text)
{
    auto* item = node(tree, type);
    require(!item->isHidden(), "right-click target exists in scene");
    tree->scrollToItem(item);
    QCoreApplication::processEvents();
    bool invoked = false;
    QTimer::singleShot(0, tree, [&] {
        auto* menu = qobject_cast<QMenu*>(QApplication::activePopupWidget());
        if (!menu) return;
        for (auto* command : menu->actions()) {
            if (command->text() == text && command->isEnabled()) {
                invoked = true;
                menu->close();
                command->trigger();
                return;
            }
        }
        menu->close();
    });
    const auto position = tree->visualItemRect(item).center();
    QContextMenuEvent event(QContextMenuEvent::Mouse, position, tree->viewport()->mapToGlobal(position));
    QApplication::sendEvent(tree->viewport(), &event);
    require(invoked, "right-click exposes and executes expected scene action");
}
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary fixture directory");
        MainWindow window;
        window.setAttribute(Qt::WA_DontShowOnScreen);
        window.show();
        app.processEvents();
        auto* scene = window.findChild<ScenePanel*>();
        auto* tree = scene->findChild<QTreeWidget*>();
        auto* viewer = window.findChild<VTKViewer*>();
        auto* model = window.findChild<ModelingController*>();
        auto* modelView = window.findChild<ModelViewController*>();
        auto* properties = window.findChild<PropertyPanel*>();
        auto* project = window.findChild<ProjectController*>();
        auto* cloud = viewer->pointCloudDocument();
        QTreeWidgetItemIterator items(tree);
        while (*items) {
            require(!(*items)->text(0).contains("Light"), "scene has no light group or leaf");
            ++items;
        }
        require(node(tree, SceneNodeType::Model)->isHidden()
                && node(tree, SceneNodeType::PointCloud)->isHidden(), "empty scene has no asset leaves");
        tree->customContextMenuRequested(tree->visualItemRect(node(tree, SceneNodeType::Camera)).center());
        require(!QApplication::activePopupWidget(), "camera has no asset context menu");

        modeling::SketchFeatureParams sketch;
        sketch.entities = {{1, modeling::Rectangle2D{0, 0, 10, 8}}};
        modeling::FeatureId sketchId = 0, extrudeId = 0;
        require(model->addSketch(sketch, sketchId)
                && model->extrude(sketchId, 6, false, modeling::ExtrudeOperation::NewBody, extrudeId),
                "create parametric model through real controller");
        const auto modelRevision = model->revision();
        auto* actors = viewer->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors();
        actors->InitTraversal();
        auto* surface = actors->GetNextActor();
        contextAction(tree, SceneNodeType::Model, "隐藏");
        require(viewer->hasBody() && !viewer->objectVisible() && !surface->GetVisibility()
                && !properties->presentation().visible && model->revision() == modelRevision,
                "hiding model updates actor and properties while preserving geometry");
        contextAction(tree, SceneNodeType::Model, "显示");
        require(surface->GetVisibility() && properties->presentation().visible, "show restores model");
        auto state = properties->presentation();
        state.visible = false;
        properties->setPresentation(state);
        properties->objectVisibilityChanged(false);
        contextAction(tree, SceneNodeType::Model, "显示");
        require(viewer->objectVisible(), "property visibility stays synchronized with scene menu");

        const auto cloudPath = temp.filePath("cloud.ply");
        QFile source(cloudPath);
        require(source.open(QIODevice::WriteOnly), "cloud fixture file");
        source.write("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n");
        source.close();
        QString error;
        require(cloud->load(cloudPath, error), "load cloud through shared document");
        cloud->select({0, 1});
        const auto cloudRevision = cloud->revision();
        contextAction(tree, SceneNodeType::PointCloud, "隐藏");
        require(!viewer->pointCloudVisible() && viewer->pointCloudSelectionMode() == 0
                && cloud->pointCount() == 3 && cloud->revision() == cloudRevision,
                "hiding cloud disables picking and preserves observations");
        contextAction(tree, SceneNodeType::PointCloud, "显示");
        require(viewer->pointCloudVisible(), "show restores point cloud");

        contextAction(tree, SceneNodeType::Model, "删除");
        require(!viewer->hasBody() && modelView->currentBody().IsNull() && model->features().empty()
                && model->revision() > modelRevision && node(tree, SceneNodeType::Model)->isHidden()
                && cloud->hasCloud(), "model deletion clears body and history while retaining cloud");
        require(!modelView->exportStepTo(temp.filePath("deleted.step"), error), "deleted model cannot export");
        model->modelRebuilt();
        require(!viewer->hasBody(), "rebuild cannot resurrect deleted model");
        contextAction(tree, SceneNodeType::PointCloud, "删除");
        require(!cloud->hasCloud() && cloud->sourcePath().isEmpty() && cloud->pointCount() == 0
                && cloud->revision() > cloudRevision && viewer->pointCloudSelectionMode() == 0
                && node(tree, SceneNodeType::PointCloud)->isHidden() && QFile::exists(cloudPath),
                "cloud deletion clears document and scene while preserving source file");

        const auto projectPath = temp.filePath("deleted.pcad");
        require(project->saveTo(projectPath, error) && project->loadFrom(projectPath, error),
                "save and reopen project after deletion");
        require(!viewer->hasBody() && !cloud->hasCloud() && model->features().empty(),
                "deleted assets stay absent after project round trip");
        viewer->setBodyShape(BRepPrimAPI_MakeBox(2, 3, 4).Shape());
        require(cloud->load(cloudPath, error), "reload cloud after deletion");
        require(!node(tree, SceneNodeType::Model)->isHidden()
                && !node(tree, SceneNodeType::PointCloud)->isHidden(), "new assets return to scene");
        contextAction(tree, SceneNodeType::Model, "删除");
        require(modelView->currentBody().IsNull() && !viewer->hasBody() && cloud->hasCloud(),
                "native imported body is also deleted independently of cloud");
        std::cout << "Scene interaction tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
