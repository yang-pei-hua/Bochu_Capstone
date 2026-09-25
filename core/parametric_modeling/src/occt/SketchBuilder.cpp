#include "occt/Builders.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <GC_MakeCircle.hxx>
#include <Geom_Circle.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <sstream>
#include <type_traits>

namespace modeling::occt {
namespace {

bool finite(double value) {
    return std::isfinite(value);
}

gp_Pnt pointOnPlane(const PlaneFrame& frame, double x, double y) {
    gp_Vec displacement(frame.xDirection);
    displacement.Multiply(x);
    gp_Vec yDisplacement(frame.yDirection);
    yDisplacement.Multiply(y);
    displacement.Add(yDisplacement);
    return frame.origin.Translated(displacement);
}

bool buildRectangle(
    const Rectangle2D& rectangle,
    const PlaneFrame& frame,
    TopoDS_Wire& wire,
    std::string& error) {
    if (!finite(rectangle.x) || !finite(rectangle.y) ||
        !finite(rectangle.width) || !finite(rectangle.height) ||
        rectangle.width <= Precision::Confusion() ||
        rectangle.height <= Precision::Confusion()) {
        error = "Rectangle dimensions and coordinates must be finite and positive";
        return false;
    }

    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(pointOnPlane(frame, rectangle.x, rectangle.y));
    polygon.Add(pointOnPlane(frame, rectangle.x + rectangle.width, rectangle.y));
    polygon.Add(pointOnPlane(
        frame, rectangle.x + rectangle.width, rectangle.y + rectangle.height));
    polygon.Add(pointOnPlane(frame, rectangle.x, rectangle.y + rectangle.height));
    polygon.Close();
    if (!polygon.IsDone()) {
        error = "OpenCASCADE could not build the rectangle wire";
        return false;
    }
    wire = polygon.Wire();
    return true;
}

bool buildCircle(
    const Circle2D& circle,
    const PlaneFrame& frame,
    TopoDS_Wire& wire,
    std::string& error) {
    if (!finite(circle.x) || !finite(circle.y) || !finite(circle.radius) ||
        circle.radius <= Precision::Confusion()) {
        error = "Circle center and radius must be finite, with a positive radius";
        return false;
    }

    const gp_Pnt center = pointOnPlane(frame, circle.x, circle.y);
    const gp_Ax2 axes(center, frame.normal, frame.xDirection);
    const Handle(Geom_Circle) geometry = new Geom_Circle(axes, circle.radius);
    BRepBuilderAPI_MakeEdge edgeBuilder(geometry);
    if (!edgeBuilder.IsDone()) {
        error = "OpenCASCADE could not build the circle edge";
        return false;
    }
    BRepBuilderAPI_MakeWire wireBuilder(edgeBuilder.Edge());
    if (!wireBuilder.IsDone()) {
        error = "OpenCASCADE could not build the circle wire";
        return false;
    }
    wire = wireBuilder.Wire();
    return true;
}

bool buildLines(
    const std::vector<SketchEntity>& entities,
    const PlaneFrame& frame,
    TopoDS_Wire& wire,
    std::string& error) {
    BRepBuilderAPI_MakeWire wireBuilder;
    for (const SketchEntity& entity : entities) {
        const auto* line = std::get_if<Line2D>(&entity.geometry);
        if (line == nullptr) {
            error = "A line profile may contain only Line entities";
            return false;
        }
        if (!finite(line->x1) || !finite(line->y1) ||
            !finite(line->x2) || !finite(line->y2)) {
            error = "Line coordinates must be finite";
            return false;
        }
        const gp_Pnt start = pointOnPlane(frame, line->x1, line->y1);
        const gp_Pnt end = pointOnPlane(frame, line->x2, line->y2);
        if (start.Distance(end) <= Precision::Confusion()) {
            error = "A line profile contains a zero-length edge";
            return false;
        }
        BRepBuilderAPI_MakeEdge edgeBuilder(start, end);
        if (!edgeBuilder.IsDone()) {
            error = "OpenCASCADE could not build a line edge";
            return false;
        }
        wireBuilder.Add(edgeBuilder.Edge());
    }
    if (!wireBuilder.IsDone()) {
        error = "Line entities do not form a connected wire";
        return false;
    }
    wire = wireBuilder.Wire();
    return true;
}

}  // namespace

PlaneFrame datumPlaneFrame(DatumPlane plane) {
    switch (plane) {
    case DatumPlane::XY:
        return {};
    case DatumPlane::YZ:
        return {gp_Pnt(0.0, 0.0, 0.0),
                gp_Dir(0.0, 1.0, 0.0),
                gp_Dir(0.0, 0.0, 1.0),
                gp_Dir(1.0, 0.0, 0.0)};
    case DatumPlane::XZ:
        return {gp_Pnt(0.0, 0.0, 0.0),
                gp_Dir(1.0, 0.0, 0.0),
                gp_Dir(0.0, 0.0, 1.0),
                gp_Dir(0.0, -1.0, 0.0)};
    }
    return {};
}

bool buildSketchFace(
    const SketchFeatureParams& params,
    const PlaneFrame& frame,
    TopoDS_Face& output,
    std::string& error) {
    try {
        if (params.entities.empty()) {
            error = "Sketch must contain one closed profile";
            return false;
        }

        TopoDS_Wire wire;
        if (params.entities.size() == 1U) {
            const SketchGeometry& geometry = params.entities.front().geometry;
            if (const auto* rectangle = std::get_if<Rectangle2D>(&geometry)) {
                if (!buildRectangle(*rectangle, frame, wire, error)) {
                    return false;
                }
            } else if (const auto* circle = std::get_if<Circle2D>(&geometry)) {
                if (!buildCircle(*circle, frame, wire, error)) {
                    return false;
                }
            } else if (!buildLines(params.entities, frame, wire, error)) {
                return false;
            }
        } else if (!buildLines(params.entities, frame, wire, error)) {
            error = "V0 supports one Rectangle/Circle or one closed chain of Lines: " + error;
            return false;
        }

        BRepBuilderAPI_MakeFace faceBuilder(wire, true);
        if (!faceBuilder.IsDone()) {
            error = "Sketch wire is not a valid closed planar profile";
            return false;
        }
        output = faceBuilder.Face();
        if (!BRepCheck_Analyzer(output).IsValid()) {
            error = "Sketch produced an invalid OpenCASCADE face";
            return false;
        }
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE sketch failure: ") + failure.what();
        return false;
    }
}

}  // namespace modeling::occt
