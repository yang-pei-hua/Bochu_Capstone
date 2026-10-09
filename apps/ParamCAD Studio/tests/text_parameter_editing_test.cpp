#include "app/MainWindow.h"
#include "app/ModelingController.h"
#include "app/ProjectController.h"
#include "app/ModelViewController.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/TextParameterPanel.h"
#include "ui/viewport/VTKViewer.h"
#include <modeling/SemanticFaces.h>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
bool near(double a, double b) { return std::abs(a-b) < 1e-6; }
const Feature& feature(ModelingController* model, FeatureId id) {
    for (const auto& f : model->features()) if (f.id==id) return f;
    throw std::runtime_error("feature missing");
}
double top(const TopoDS_Shape& shape) {
    Bnd_Box bounds; BRepBndLib::AddOptimal(shape,bounds,false,false);
    double x0,y0,z0,x1,y1,z1; bounds.Get(x0,y0,z0,x1,y1,z1); return z1;
}
double volume(const TopoDS_Shape& shape) { GProp_GProps p; BRepGProp::VolumeProperties(shape,p); return p.Mass(); }
void workflow(QApplication& app) {
    MainWindow window; window.setAttribute(Qt::WA_DontShowOnScreen); window.show(); app.processEvents();
    auto* model=window.findChild<ModelingController*>();
    auto* viewer=window.findChild<VTKViewer*>();
    auto* tree=window.findChild<ModelingPanel*>();
    auto* panel=window.findChild<TextParameterPanel*>();
    auto* input=panel->findChild<QLineEdit*>("textParameterInput");
    auto* preview=panel->findChild<QPushButton*>("previewTextParametersButton");
    auto* apply=panel->findChild<QPushButton*>("applyTextParametersButton");
    auto* cancel=panel->findChild<QPushButton*>("cancelTextParametersButton");
    auto* message=panel->findChild<QPlainTextEdit*>("textParameterMessage");
    require(input && preview && apply && cancel && message, "text controls are connected in the actual window");
    reconstruction::BoxWithThroughHolesCandidate candidate;
    candidate.box.primitive.sizeX=10; candidate.box.primitive.sizeY=20; candidate.box.primitive.sizeZ=30;
    reconstruction::ThroughHoleCandidate hole; hole.primitive={{5,10,0},{0,0,1},2}; candidate.throughHoles.push_back(hole);
    require(model->commitReconstructedBoxWithThroughHoles(candidate), "create box-hole fixture");
    const auto boxId=model->features()[0].id, holeId=model->features()[1].id;
    model->clearHistory(); tree->selectFeature(holeId);
    auto* textAction=window.findChild<QAction*>("textParameterEditAction");
    auto* propertyDock=window.findChild<QDockWidget*>("propertyDock");
    require(textAction && propertyDock && textAction->isEnabled(), "text editing has a discoverable toolbar/menu action");
    propertyDock->hide(); textAction->trigger(); app.processEvents();
    require(propertyDock->isVisible() && panel->isVisible(), "text action opens even a hidden Properties dock");
    input->setText("孔直径增加2mm");
    const auto originalRevision=model->revision(); const auto originalBody=model->bodyShape();
    preview->click();
    require(apply->isEnabled() && message->toPlainText().contains("4 → 6 mm"), "actual UI exposes the resolved edit");
    require(model->revision()==originalRevision && model->bodyShape().IsSame(originalBody)
        && !model->canUndo() && !model->hasProtectedReconstruction(), "preview has no geometry history revision or protection side effects");
    apply->click();
    require(near(std::get<ThroughHolePrimitiveParams>(feature(model,holeId).params).radius,3)
        && model->hasProtectedReconstruction() && model->canUndo() && !apply->isEnabled(), "text apply commits once through existing manual edit semantics");
    require(model->undo() && !model->hasProtectedReconstruction() && model->canRedo(), "undo restores manual flags and geometry");
    input->setText(QString("#%1 的宽改为1mm，固定Y-面").arg(boxId)); preview->click();
    require(!apply->isEnabled() && model->canRedo() && near(std::get<ThroughHolePrimitiveParams>(feature(model,holeId).params).radius,2), "failed rebuild preview preserves redo and parameters");
    require(model->redo(), "redo remains available after failed preview");
    input->setText("孔直径减少1mm"); preview->click(); require(apply->isEnabled(), "new relative edit previews");
    input->setText("孔直径减少2mm"); require(!apply->isEnabled(), "changing text invalidates its previous proposal");
    preview->click(); cancel->click(); require(!apply->isEnabled() && input->text().isEmpty(), "cancel discards the proposal");
    input->setText("孔直径改为4mm"); preview->click(); tree->selectFeature(boxId);
    require(!apply->isEnabled(), "selection changes cannot reuse a previous target");
    input->setText("宽改为1mm，固定Y-面"); const auto failureBody=model->bodyShape(); const auto failureRevision=model->revision(); preview->click();
    require(!apply->isEnabled() && model->revision()==failureRevision && model->bodyShape().IsSame(failureBody), "dependent through-hole failure is detected in the isolated preview");

    FeatureId sketchId, extrudeId;
    require(model->addSketch({featureFace(boxId,FaceRole::BoxZMax),{{1,Rectangle2D{1,1,2,2}}}},sketchId)
        && model->extrude(sketchId,5,false,ExtrudeOperation::Join,extrudeId), "attach dependent extrusion to semantic box top");
    tree->selectFeature(boxId); input->setText("高度改为40mm，固定Z-面");
    preview->click();
    if (!apply->isEnabled() || !near(top(model->bodyShape()),35))
        std::cerr << "Preview enabled=" << apply->isEnabled() << " top=" << top(model->bodyShape())
                  << " sketch=" << viewer->sketchInteractionEnabled() << " message=" << message->toPlainText().toStdString() << '\n';
    require(apply->isEnabled() && near(top(model->bodyShape()),35), "dependency preview leaves the displayed body unchanged");
    const auto screenshot=qEnvironmentVariable("CAD_TEXT_PREVIEW");
    app.processEvents();
    if (!screenshot.isEmpty()) require(window.grab().save(screenshot), "save native text preview screenshot");
    require(message->height() >= 120 && message->isReadOnly(), "preview remains readable and scrollable after layout");
    const auto staleRevision=model->revision();
    QString staleError;
    require(model->editPrimitive(holeId,{4,1,1},ResizeAnchor::Center,staleRevision), "intervening edit changes the revision");
    require(!apply->isEnabled(), "intervening model rebuild invalidates the preview");
    require(!model->validatePrimitiveEdit(boxId,{10,20,40},ResizeAnchor::ZMin,staleRevision,staleError), "stale proposal cannot validate");
    preview->click(); require(apply->isEnabled(), "refreshed preview uses the current revision"); apply->click();
    require(near(top(model->bodyShape()),45) && feature(model,boxId).resizeAnchor==ResizeAnchor::ZMin, "text box edit moves the downstream extrusion through the semantic reference");
    require(model->undo() && near(top(model->bodyShape()),35) && model->redo() && near(top(model->bodyShape()),45), "undo and redo include all dependent geometry");

    input->setText("高度增加1mm"); preview->click(); viewer->setSketchInteractionEnabled(true);
    require(!apply->isEnabled() && !input->isEnabled(), "sketch interaction invalidates and disables text editing");
    viewer->setSketchInteractionEnabled(false);
    require(input->isEnabled() && !apply->isEnabled(), "exiting sketch requires a fresh preview");
    preview->click(); require(apply->isEnabled(), "preview before project replacement");
    QTemporaryDir temporary; require(temporary.isValid(), "project directory");
    auto* project=window.findChild<ProjectController*>(); QString error;
    require(project->saveTo(temporary.filePath("text-edited.pcad"),error)
        && project->loadFrom(temporary.filePath("text-edited.pcad"),error), "text edited model persists and reopens");
    require(!apply->isEnabled() && near(top(model->bodyShape()),45) && model->hasProtectedReconstruction()
        && !model->canUndo(), "reopened project retains edit geometry and protection but invalidates pending preview/history");
    tree->selectFeature(boxId); input->setText("高度增加1mm，固定Z-面"); preview->click(); apply->click();
    require(near(top(model->bodyShape()),46), "text editing continues after project reopen");
    auto* modelView=window.findChild<ModelViewController*>();
    require(modelView->exportStepTo(temporary.filePath("text-edited.step"),error), "text changes export through the existing STEP path");
    const double committedVolume=volume(model->bodyShape());
    input->setText("高度增加1mm"); preview->click(); viewer->setBodyShape(originalBody);
    require(!apply->isEnabled() && !input->isEnabled() && near(volume(model->bodyShape()),committedVolume), "displaying a different body invalidates text edits without changing the model");
}
}
int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat()); QApplication app(argc,argv);
    try { workflow(app); std::cout << "Text parameter editing workflow passed.\n"; return 0; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
