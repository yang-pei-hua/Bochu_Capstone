#include "modeling/ModelingCore.h"
#include "modeling/StepExporter.h"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace modeling;

struct Bounds {
    double xMin;
    double yMin;
    double zMin;
    double xMax;
    double yMax;
    double zMax;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string& label) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            label + ": expected " + std::to_string(expected) +
            ", got " + std::to_string(actual));
    }
}

Bounds boundsOf(const TopoDS_Shape& shape) {
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    Bounds result{};
    box.Get(result.xMin, result.yMin, result.zMin,
            result.xMax, result.yMax, result.zMax);
    return result;
}

double volumeOf(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

SketchFeatureParams rectangleSketch() {
    SketchFeatureParams params;
    params.plane = datumPlane(DatumPlane::XY);
    params.entities.push_back({1, Rectangle2D{0.0, 0.0, 50.0, 30.0}});
    return params;
}

void requireBox(const TopoDS_Shape& shape, double height, double expectedVolume) {
    require(!shape.IsNull(), "Expected a non-null body");
    const Bounds bounds = boundsOf(shape);
    requireNear(bounds.xMin, 0.0, 1.0e-6, "minimum X");
    requireNear(bounds.yMin, 0.0, 1.0e-6, "minimum Y");
    requireNear(bounds.zMin, 0.0, 1.0e-6, "minimum Z");
    requireNear(bounds.xMax, 50.0, 1.0e-6, "maximum X");
    requireNear(bounds.yMax, 30.0, 1.0e-6, "maximum Y");
    requireNear(bounds.zMax, height, 1.0e-6, "maximum Z");
    requireNear(volumeOf(shape), expectedVolume, 1.0e-4, "body volume");
}

void requireExport(const TopoDS_Shape& shape, const std::filesystem::path& path) {
    std::string error;
    require(exportStep(shape, path, &error), "STEP export failed: " + error);
    require(std::filesystem::exists(path), "STEP file was not created");
    require(std::filesystem::file_size(path) > 0U, "STEP file is empty");
}

void testEndToEnd(const std::filesystem::path& outputDirectory) {
    PartDocument document;

    const FeatureId baseSketchId = document.addFeature(rectangleSketch());
    const FeatureId extrudeId = document.addFeature(
        ExtrudeFeatureParams{baseSketchId, 20.0, false});
    require(baseSketchId != extrudeId, "Feature IDs must be stable and unique");
    require(document.features().size() == 2U, "Feature history should contain two items");

    require(document.rebuild(), document.lastError());
    requireBox(document.bodyShape(), 20.0, 50.0 * 30.0 * 20.0);
    requireExport(document.bodyShape(), outputDirectory / "01_base.step");

    require(document.editFeature(
        extrudeId, ExtrudeFeatureParams{baseSketchId, 30.0, false}),
        document.lastError());
    require(document.rebuild(), document.lastError());
    requireBox(document.bodyShape(), 30.0, 50.0 * 30.0 * 30.0);
    requireExport(document.bodyShape(), outputDirectory / "02_modified.step");

    SketchFeatureParams holeSketch;
    holeSketch.plane = featureFace(extrudeId, FaceRole::EndFace);
    holeSketch.entities.push_back({2, Circle2D{25.0, 15.0, 5.0}});
    const FeatureId holeSketchId = document.addFeature(holeSketch);
    document.addFeature(CutFeatureParams{holeSketchId, 10.0, false});

    require(document.rebuild(), document.lastError());
    const double holeVolume = std::acos(-1.0) * 5.0 * 5.0 * 10.0;
    requireBox(document.bodyShape(), 30.0, 50.0 * 30.0 * 30.0 - holeVolume);
    requireExport(document.bodyShape(), outputDirectory / "03_cut.step");

    // The downstream sketch must follow the semantic EndFace after this edit.
    require(document.editFeature(
        extrudeId, ExtrudeFeatureParams{baseSketchId, 40.0, false}),
        document.lastError());
    require(document.rebuild(), document.lastError());
    requireBox(document.bodyShape(), 40.0, 50.0 * 30.0 * 40.0 - holeVolume);
    requireExport(document.bodyShape(), outputDirectory / "04_upstream_rebuilt.step");
}

void testReferenceFailureKeepsLastSuccessfulBody() {
    PartDocument document;
    const FeatureId sketchId = document.addFeature(rectangleSketch());
    document.addFeature(ExtrudeFeatureParams{sketchId, 20.0, false});

    SketchFeatureParams lostSketch;
    lostSketch.plane = featureFace(999999, FaceRole::EndFace);
    lostSketch.entities.push_back({2, Circle2D{25.0, 15.0, 5.0}});
    document.addFeature(lostSketch);

    require(!document.rebuild(), "A lost face reference must fail rebuild");
    require(document.lastError().find("ReferenceLost") != std::string::npos,
            "Lost reference should return an explicit ReferenceLost error");
    requireBox(document.bodyShape(), 20.0, 50.0 * 30.0 * 20.0);
}

void testCommandApi() {
    ModelingCore core;
    const ModelResult addSketch = core.execute(AddFeatureCommand{rectangleSketch()});
    require(addSketch.success, addSketch.error);

    const ModelResult addExtrude = core.execute(AddFeatureCommand{
        ExtrudeFeatureParams{addSketch.featureId, 12.0, false}});
    require(addExtrude.success, addExtrude.error);

    const ModelResult editExtrude = core.execute(EditFeatureCommand{
        addExtrude.featureId,
        ExtrudeFeatureParams{addSketch.featureId, 18.0, false}});
    require(editExtrude.success, editExtrude.error);
    require(core.document().rebuild(), core.document().lastError());
    requireBox(core.document().bodyShape(), 18.0, 50.0 * 30.0 * 18.0);

    const ModelResult removeExtrude = core.execute(
        RemoveFeatureCommand{addExtrude.featureId});
    require(removeExtrude.success, removeExtrude.error);
    require(core.document().features().size() == 1U,
            "Remove command should update feature history");
    require(core.document().rebuild(), core.document().lastError());
    require(core.document().bodyShape().IsNull(),
            "Removing the only body feature should rebuild to an empty body");
}

void testLineProfileAndDatumPlanes() {
    PartDocument yzDocument;
    SketchFeatureParams lineSketch;
    lineSketch.plane = datumPlane(DatumPlane::YZ);
    lineSketch.entities = {
        {1, Line2D{0.0, 0.0, 4.0, 0.0}},
        {2, Line2D{4.0, 0.0, 4.0, 5.0}},
        {3, Line2D{4.0, 5.0, 0.0, 5.0}},
        {4, Line2D{0.0, 5.0, 0.0, 0.0}},
    };
    const FeatureId lineSketchId = yzDocument.addFeature(lineSketch);
    yzDocument.addFeature(ExtrudeFeatureParams{lineSketchId, 3.0, false});
    require(yzDocument.rebuild(), yzDocument.lastError());
    const Bounds yzBounds = boundsOf(yzDocument.bodyShape());
    requireNear(yzBounds.xMin, 0.0, 1.0e-6, "YZ line profile minimum X");
    requireNear(yzBounds.xMax, 3.0, 1.0e-6, "YZ line profile maximum X");
    requireNear(yzBounds.yMax, 4.0, 1.0e-6, "YZ line profile maximum Y");
    requireNear(yzBounds.zMax, 5.0, 1.0e-6, "YZ line profile maximum Z");
    requireNear(volumeOf(yzDocument.bodyShape()), 60.0, 1.0e-6,
                "YZ line profile volume");

    PartDocument xzDocument;
    SketchFeatureParams xzSketch;
    xzSketch.plane = datumPlane(DatumPlane::XZ);
    xzSketch.entities.push_back({1, Rectangle2D{0.0, 0.0, 4.0, 5.0}});
    const FeatureId xzSketchId = xzDocument.addFeature(xzSketch);
    xzDocument.addFeature(ExtrudeFeatureParams{xzSketchId, 2.0, false});
    require(xzDocument.rebuild(), xzDocument.lastError());
    const Bounds xzBounds = boundsOf(xzDocument.bodyShape());
    requireNear(xzBounds.xMax, 4.0, 1.0e-6, "XZ rectangle maximum X");
    requireNear(xzBounds.yMin, -2.0, 1.0e-6, "XZ rectangle minimum Y");
    requireNear(xzBounds.yMax, 0.0, 1.0e-6, "XZ rectangle maximum Y");
    requireNear(xzBounds.zMax, 5.0, 1.0e-6, "XZ rectangle maximum Z");
    requireNear(volumeOf(xzDocument.bodyShape()), 40.0, 1.0e-6,
                "XZ rectangle volume");
}

}  // namespace

int main() {
    try {
        const std::filesystem::path outputDirectory = MODELING_TEST_OUTPUT_DIR;
        std::filesystem::create_directories(outputDirectory);
        testEndToEnd(outputDirectory);
        testReferenceFailureKeepsLastSuccessfulBody();
        testCommandApi();
        testLineProfileAndDatumPlanes();
        std::cout << "CadModelCore V0 smoke test passed. STEP output: "
                  << outputDirectory << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "CadModelCore V0 smoke test failed: "
                  << exception.what() << '\n';
        return 1;
    }
}
