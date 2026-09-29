#include "occt/Builders.h"

#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace modeling::occt {

bool isValidShape(const TopoDS_Shape& shape, std::string& error) {
    if (shape.IsNull()) {
        error = "Shape is null";
        return false;
    }
    if (!BRepCheck_Analyzer(shape).IsValid()) {
        error = "B-Rep validation failed";
        return false;
    }
    return true;
}

double shapeVolume(const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        return 0.0;
    }
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

double throughAllDistance(const TopoDS_Shape& shape) {
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        return 1.0;
    }

    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double diagonal = std::sqrt(
        std::pow(xMax - xMin, 2.0) +
        std::pow(yMax - yMin, 2.0) +
        std::pow(zMax - zMin, 2.0));
    return std::max(1.0, diagonal * 2.0 + Precision::Confusion());
}

PlaneFrame translatedFrame(
    const PlaneFrame& source,
    const gp_Dir& normal,
    double distance) {
    gp_Vec translation(normal);
    translation.Multiply(distance);

    // Preserve the sketch X axis and derive Y so that X x Y = normal.
    const gp_Dir yDirection(gp_Vec(normal).Crossed(gp_Vec(source.xDirection)));
    return {
        source.origin.Translated(translation),
        source.xDirection,
        yDirection,
        normal,
    };
}

}  // namespace modeling::occt
