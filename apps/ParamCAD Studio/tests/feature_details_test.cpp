#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/ModelingPanel.h"

#include <QApplication>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QUrl>
#include <iostream>
#include <stdexcept>

using namespace modeling;
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
Feature feature(FeatureId id, const FeatureParams& params, const std::string& name) {
    return {id,name,featureTypeOf(params),params,false,true,{},true};
}
}
int main(int argc, char** argv)
{
    QApplication app(argc,argv);
    try {
        // A reproducible L bracket in world coordinates, including its triangular rib.
        SketchFeatureParams sketch;
        sketch.plane = Plane3d{{36,8,8},{2,0,0},{0,3,0}};
        sketch.entities = {{1,Line2D{0,0,40,0}}, {2,Line2D{40,0,0,50}}, {3,Line2D{0,50,0,0}}};
        std::vector<Feature> features{
            feature(1,BoxPrimitiveParams{{},80,60,8,true},"Base"),
            feature(2,BoxPrimitiveParams{{{0,0,8},{1,0,0},{0,0,1}},80,8,60,true},"Upright"),
            feature(3,sketch,"RibSketch"),
            feature(4,ExtrudeFeatureParams{3,8,false,ExtrudeOperation::Join},"RibExtrude")};
        FeatureDetailsPanel details;
        auto* browser = details.findChild<QTextBrowser*>("featureDetailsText");
        const auto text = [&] { return browser->toPlainText(); };
        details.setFeature(features,2);
        require(text().contains("80 mm") && text().contains("60 mm") && text().contains("(0, 0, 8)")
                && text().contains("(80, 8)") && text().contains("Join"),
                "box exposes dimensions, placement, fusion and equivalent rectangle");
        details.setFeature(features,4);
        require(text().contains("RibSketch (#3)") && text().contains("8 mm") && text().contains("Join")
                && text().contains("沿草图法向 +N") && text().contains("(36, 48, 8)")
                && text().contains("(36, 8, 58)"),
                "extrusion embeds its resolved triangular sketch and world coordinates");
        auto reverse = std::get<ExtrudeFeatureParams>(features[3].params);
        reverse.reverse = true; features[3].params = reverse;
        details.setFeature(features,4);
        require(text().contains("沿草图法向 -N") && text().contains("(-1, 0, 0)"),
                "reverse extrusion reports the actual normalized world direction");
        features.push_back(feature(5,CutFeatureParams{3,4,false,false},"Cut"));
        details.setFeature(features,5);
        require(text().contains("沿草图法向 -N") && text().contains("4 mm"),
                "default cut direction is opposite the sketch normal");
        features[4].params = CutFeatureParams{3,4,true,true};
        details.setFeature(features,5);
        require(text().contains("贯穿全部") && text().contains("沿草图法向 +N") && !text().contains("4 mm"),
                "through cut omits unused depth and respects reverse semantics");
        const std::vector<FeatureParams> curved{
            ThroughHolePrimitiveParams{{1,2,3},{0,0,1},6},
            SpherePrimitiveParams{{1,2,3},6},
            CylinderPrimitiveParams{{1,2,3},{0,0,1},6,20},
            ConePrimitiveParams{{1,2,3},{0,0,1},6,3,20},
            TorusPrimitiveParams{{1,2,3},{0,0,1},20,3}};
        for (const auto& params : curved) {
            features.push_back(feature(6,params,"Curved"));
            details.setFeature(features,6);
            require(text().contains("(1, 2, 3)") && text().contains("mm"), "curved features expose their own placement and sizes");
            features.pop_back();
        }
        auto referenced = sketch;
        referenced.plane = featureFace(4,FaceRole::EndFace);
        features.push_back(feature(7,referenced,"FaceSketch"));
        details.setFeature(features,7);
        require(text().contains("RibExtrude (#4)") && text().contains("(28, 8, 8)"),
                "face sketch resolves the end of the reversed extrusion");
        features.back().params = SketchFeatureParams{offsetDatumPlane(DatumPlane::XZ,5),{}};
        details.setFeature(features,7);
        require(text().contains("XZ Plane") && text().contains("(0, -5, 0)"), "offset follows datum normal, including XZ negative Y");
        features[3].params = ExtrudeFeatureParams{999,8,false,ExtrudeOperation::Join};
        details.setFeature(features,4);
        require(text().contains("引用不存在") && !text().contains("(36, 48, 8)"), "lost sketch clears old contour instead of inventing a plane");
        features[3].params = ExtrudeFeatureParams{3,8,false,ExtrudeOperation::Join};
        features[2].params = SketchFeatureParams{Plane3d{{},{},{0,1,0}},sketch.entities};
        details.setFeature(features,4);
        require(text().contains("平面引用无法解析") && !text().contains("世界方向"), "invalid explicit plane has no fabricated world direction");
        features[2].name = "<b>literal</b>";
        details.setFeature(features,3);
        require(text().contains("<b>literal</b>"), "feature names are displayed as text, not markup");
        FeatureId requested = 0;
        QObject::connect(&details,&FeatureDetailsPanel::featureRequested,[&](FeatureId id) { requested=id; });
        browser->anchorClicked(QUrl("feature:3"));
        require(requested==3,"source sketch link requests the referenced feature");
        ModelingPanel list;
        int selections = 0;
        QObject::connect(&list,&ModelingPanel::featureSelected,[&](FeatureId) { ++selections; });
        list.setFeatures(features); list.selectFeature(4);
        require(list.selectedFeatureId()==4 && selections==1,"feature selection publishes exactly once");
        list.setFeatures(features);
        require(list.selectedFeatureId()==4 && selections==1,"refresh retains selection without changing Properties tab");
        features.erase(features.begin()+3); list.setFeatures(features);
        details.setFeature(features,list.selectedFeatureId());
        require(list.selectedFeatureId()==kInvalidFeatureId && text().contains("选择一项"),"deleted selected feature clears stale details");
        std::cout << "Feature details tests passed.\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
