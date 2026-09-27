#pragma once

#include "modeling/FeatureParams.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>

namespace modeling::occt {

struct PlaneFrame {
    gp_Pnt origin;
    gp_Dir xDirection{1.0, 0.0, 0.0};
    gp_Dir yDirection{0.0, 1.0, 0.0};
    gp_Dir normal{0.0, 0.0, 1.0};
};

struct ExtrudeResult {
    TopoDS_Shape shape;
    PlaneFrame startFace;
    PlaneFrame endFace;
};

PlaneFrame datumPlaneFrame(DatumPlane plane);

bool buildSketchFace(
    const SketchFeatureParams& params,
    const PlaneFrame& frame,
    TopoDS_Face& output,
    std::string& error);

bool buildExtrude(
    const TopoDS_Face& profile,
    const PlaneFrame& profileFrame,
    const ExtrudeFeatureParams& params,
    ExtrudeResult& output,
    std::string& error);

bool buildCut(
    const TopoDS_Shape& body,
    const TopoDS_Face& profile,
    const PlaneFrame& profileFrame,
    const CutFeatureParams& params,
    TopoDS_Shape& output,
    std::string& error);

bool buildBoxPrimitive(
    const BoxPrimitiveParams& params,
    TopoDS_Shape& output,
    std::string& error);

bool isValidShape(const TopoDS_Shape& shape, std::string& error);
double shapeVolume(const TopoDS_Shape& shape);
double throughAllDistance(const TopoDS_Shape& shape);

PlaneFrame translatedFrame(
    const PlaneFrame& source,
    const gp_Dir& normal,
    double distance);

}  // namespace modeling::occt
