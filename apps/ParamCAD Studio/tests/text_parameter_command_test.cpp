#include "core/TextParameterCommand.h"
#include <QCoreApplication>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
bool near(double a, double b) { return std::abs(a-b) < 1e-8; }
Feature feature(FeatureId id, FeatureParams params) { return {id,"Test",featureTypeOf(params),std::move(params)}; }
void contracts() {
    const std::vector<Feature> features{
        feature(1, BoxPrimitiveParams{{}, 10, 20, 30, false}),
        feature(2, ThroughHolePrimitiveParams{{5,10,0},{0,0,1},2}),
        feature(3, CylinderPrimitiveParams{{},{0,0,1},4,10}),
        feature(4, SpherePrimitiveParams{{},4})};
    textcad::PrimitiveEdit output;
    QString error;
    const auto parse = [&](const QString& input, FeatureId selected = 1) {
        return textcad::parsePrimitiveEdit(input, features, selected, 42, output, error);
    };
    require(parse("把当前特征的长度改为 2 cm，宽增加 5 mm，高减少 10%，固定中心"), "multi-dimension Chinese command parses");
    require(output.featureId == 1 && output.expectedRevision == 42 && near(output.values[0],20)
        && near(output.values[1],25) && near(output.values[2],27), "units percentages and revision retain their meanings");
    require(output.summary.contains("10 → 20 mm") && output.summary.contains("局部 Z") && output.summary.contains("中心"), "preview labels show actual committed dimensions");
    require(parse("将 #2 的孔直径增加 2 毫米", 4) && near(output.values[0],6), "explicit target overrides selected unsupported feature");
    require(parse("#3 直径改为 1厘米，高度设为 .03m，固定顶面", 0)
        && near(output.values[0],5) && near(output.values[1],30) && output.anchor==ResizeAnchor::Top, "diameter converts to radius and cylinder top is explicit");
    require(parse("高度增大 1e1 mm，固定 Z− 面") && near(output.values[2],40) && output.anchor==ResizeAnchor::ZMin, "fixed local face and scientific notation parse");
    require(parse("X尺寸设置为 12") && near(output.values[0],12), "omitted unit is millimetres");
    require(parse("#3 直径减少 25%") && near(output.values[0],3), "percentage diameter edit preserves radius conversion");
    for (const QString input : {"", "高度改为30mm，然后删除所有特征", "高度改为30mm junk", "高度改为30mm，", "高度改为30mm;宽改为10mm",
            "高改为30mm，高增加1mm", "#3 半径改为5mm，直径改为10mm", "孔直径改为4mm", "高改为-1mm", "高改为1e309mm", "高改为1e308mm",
            "高减少100%", "高改为30%", "高减少-2mm", "高改为0mm", "高改为30mm，固定底面", "#0 高改为30mm",
            "#18446744073709551616 高改为30mm", "#999 高改为30mm", "#2 高度改为30mm", "#4 半径改为8mm"}) {
        require(!parse(input) && !error.isEmpty() && output.featureId==kInvalidFeatureId, "ambiguous invalid or partially understood input must fail without a proposal");
    }
    require(!parse("高度改为30mm", 0), "unselected implicit target is rejected");
    require(!parse(QString("高度改为30mm") + QString(600, ' ')), "oversized pasted commands cannot become a valid prefix");
    auto fixed = features; fixed[0].resizeAnchor=ResizeAnchor::XMin;
    require(!textcad::parsePrimitiveEdit("高度改为40mm", fixed, 1, 42, output, error), "default anchor honours existing fixed-face semantics");
    require(textcad::parsePrimitiveEdit("高度改为40mm，固定中心", fixed, 1, 42, output, error), "explicit anchor can override the persisted anchor");
    auto suppressed = features; suppressed[0].suppressed=true;
    require(!textcad::parsePrimitiveEdit("高度改为40mm", suppressed, 1, 42, output, error), "suppressed targets are unavailable");
    suppressed[0].suppressed=false; suppressed[0].valid=false;
    require(!textcad::parsePrimitiveEdit("高度改为40mm", suppressed, 1, 42, output, error), "invalid targets are unavailable");
}
}
int main(int argc, char** argv) {
    QCoreApplication app(argc,argv);
    try { contracts(); std::cout << "Text parameter command contracts passed.\n"; return 0; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
