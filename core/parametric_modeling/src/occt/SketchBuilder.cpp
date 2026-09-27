#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeCircle.hxx>
#include <Geom_Circle.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_State.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <type_traits>
#include <vector>

namespace modeling::occt {
namespace {

bool finite(double value) {
    return std::isfinite(value);
}

bool samePoint(double ax, double ay, double bx, double by) {
    return std::abs(ax - bx) <= Precision::Confusion() &&
        std::abs(ay - by) <= Precision::Confusion();
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

bool buildLineWires(
    const std::vector<SketchEntity>& entities,
    const PlaneFrame& frame,
    std::vector<TopoDS_Wire>& wires,
    std::string& error) {
    std::vector<Line2D> unordered;
    unordered.reserve(entities.size());
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
        if (samePoint(line->x1, line->y1, line->x2, line->y2)) {
            error = "A line profile contains a zero-length edge";
            return false;
        }
        unordered.push_back(*line);
    }
    if (unordered.empty()) {
        return true;
    }
    if (unordered.size() < 3U) {
        error = "A line profile needs at least three segments";
        return false;
    }

    std::vector<bool> used(unordered.size(), false);
    std::size_t usedCount = 0;
    while (usedCount < unordered.size()) {
        const auto unused = std::find(used.begin(), used.end(), false);
        const std::size_t first = static_cast<std::size_t>(
            std::distance(used.begin(), unused));
        std::vector<Line2D> ordered{unordered[first]};
        used[first] = true;
        ++usedCount;

        const double startX = unordered[first].x1;
        const double startY = unordered[first].y1;
        double endX = unordered[first].x2;
        double endY = unordered[first].y2;

        while (!samePoint(endX, endY, startX, startY)) {
            bool found = false;
            for (std::size_t index = 0; index < unordered.size(); ++index) {
                if (used[index]) {
                    continue;
                }
                const Line2D& candidate = unordered[index];
                if (samePoint(candidate.x1, candidate.y1, endX, endY)) {
                    ordered.push_back(candidate);
                    endX = candidate.x2;
                    endY = candidate.y2;
                } else if (samePoint(candidate.x2, candidate.y2, endX, endY)) {
                    ordered.push_back(
                        Line2D{candidate.x2, candidate.y2, candidate.x1, candidate.y1});
                    endX = candidate.x1;
                    endY = candidate.y1;
                } else {
                    continue;
                }
                used[index] = true;
                ++usedCount;
                found = true;
                break;
            }
            if (!found) {
                error = "Line entities contain an open or disconnected chain";
                return false;
            }
        }
        if (ordered.size() < 3U) {
            error = "A closed line profile needs at least three segments";
            return false;
        }

        BRepBuilderAPI_MakeWire wireBuilder;
        for (const Line2D& line : ordered) {
            BRepBuilderAPI_MakeEdge edgeBuilder(
                pointOnPlane(frame, line.x1, line.y1),
                pointOnPlane(frame, line.x2, line.y2));
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
        wires.push_back(wireBuilder.Wire());
    }
    return true;
}

bool buildFaceFromWires(
    const std::vector<TopoDS_Wire>& wires,
    TopoDS_Face& output,
    std::string& error) {
    if (wires.empty()) {
        error = "Sketch must contain one closed profile";
        return false;
    }

    std::vector<TopoDS_Face> candidateFaces;
    std::vector<double> areas;
    candidateFaces.reserve(wires.size());
    areas.reserve(wires.size());
    for (const TopoDS_Wire& wire : wires) {
        BRepBuilderAPI_MakeFace candidateBuilder(wire, true);
        if (!candidateBuilder.IsDone()) {
            error = "Sketch wire is not a valid closed planar profile";
            return false;
        }
        const TopoDS_Face candidate = candidateBuilder.Face();
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(candidate, properties);
        candidateFaces.push_back(candidate);
        areas.push_back(std::abs(properties.Mass()));
    }

    const auto outerIterator = std::max_element(areas.begin(), areas.end());
    const std::size_t outerIndex = static_cast<std::size_t>(
        std::distance(areas.begin(), outerIterator));
    const TopoDS_Wire outerWire = BRepTools::OuterWire(candidateFaces[outerIndex]);
    BRepBuilderAPI_MakeFace faceBuilder(outerWire, true);

    for (std::size_t index = 0; index < candidateFaces.size(); ++index) {
        if (index == outerIndex) {
            continue;
        }
        const TopoDS_Wire innerWire = BRepTools::OuterWire(candidateFaces[index]);
        TopExp_Explorer vertexExplorer(innerWire, TopAbs_VERTEX);
        if (!vertexExplorer.More()) {
            error = "Sketch contains an empty profile wire";
            return false;
        }
        const gp_Pnt sample = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
        BRepClass_FaceClassifier classifier(
            candidateFaces[outerIndex], sample, Precision::Confusion());
        if (classifier.State() != TopAbs_IN) {
            error = "Sketch profiles must form one outer loop with inner holes";
            return false;
        }
        TopoDS_Wire hole = innerWire;
        hole.Reverse();
        faceBuilder.Add(hole);
    }

    if (!faceBuilder.IsDone()) {
        error = "OpenCASCADE could not build the multi-loop sketch face";
        return false;
    }
    output = faceBuilder.Face();
    if (!BRepCheck_Analyzer(output).IsValid()) {
        error = "Sketch produced an invalid OpenCASCADE face";
        return false;
    }
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
        std::vector<SketchEntity> lineEntities;
        std::vector<TopoDS_Wire> wires;
        lineEntities.reserve(params.entities.size());
        for (const SketchEntity& entity : params.entities) {
            if (std::holds_alternative<Point2D>(entity.geometry)) {
                continue;
            }
            if (const auto* rectangle = std::get_if<Rectangle2D>(&entity.geometry)) {
                TopoDS_Wire wire;
                if (!buildRectangle(*rectangle, frame, wire, error)) {
                    return false;
                }
                wires.push_back(wire);
            } else if (const auto* circle = std::get_if<Circle2D>(&entity.geometry)) {
                TopoDS_Wire wire;
                if (!buildCircle(*circle, frame, wire, error)) {
                    return false;
                }
                wires.push_back(wire);
            } else {
                lineEntities.push_back(entity);
            }
        }

        if (!buildLineWires(lineEntities, frame, wires, error)) {
            return false;
        }
        return buildFaceFromWires(wires, output, error);
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE sketch failure: ") +
            failureMessage(failure);
        return false;
    }
}

}  // namespace modeling::occt
