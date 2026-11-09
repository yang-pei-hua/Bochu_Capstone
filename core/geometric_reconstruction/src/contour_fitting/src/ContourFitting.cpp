#include "contour_fitting/ContourFitting.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/Stop_above_cost_threshold.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>
#include <CGAL/Polyline_simplification_2/Vertex_base_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace contour_fitting {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;

struct FaceState {
    bool kept = false;
    std::array<bool, 3> visited{};
    bool componentVisited = false;
};

using VertexBase = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel>;
using FaceBase = CGAL::Triangulation_face_base_with_info_2<FaceState, Kernel>;
using Triangulation = CGAL::Delaunay_triangulation_2<Kernel,
    CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>>;
using Edge = Triangulation::Edge;
using Vertex = Triangulation::Vertex_handle;

double estimateRadius(const Triangulation& triangulation)
{
    std::vector<double> nearest(triangulation.number_of_vertices(), std::numeric_limits<double>::infinity());
    for (auto edge = triangulation.finite_edges_begin(); edge != triangulation.finite_edges_end(); ++edge) {
        const auto [face, side] = *edge;
        const auto first = face->vertex(triangulation.ccw(side));
        const auto last = face->vertex(triangulation.cw(side));
        const double distance = CGAL::squared_distance(first->point(), last->point());
        nearest[first->info()] = std::min(nearest[first->info()], distance);
        nearest[last->info()] = std::min(nearest[last->info()], distance);
    }
    const auto percentile = nearest.begin() + (nearest.size() - 1) * 9 / 10;
    std::nth_element(nearest.begin(), percentile, nearest.end());
    const double radius = 6 * std::sqrt(*percentile);

    // Uniform interior samples need a smaller radius than projected data with
    // dense rows. Bound the local estimate using the mean rectangular density.
    const auto bounds = CGAL::bbox_2(triangulation.points_begin(), triangulation.points_end());
    const double meanSpacing = std::sqrt(bounds.xmax() - bounds.xmin())
        * std::sqrt(bounds.ymax() - bounds.ymin()) / std::sqrt(double(nearest.size()));
    return std::min(radius, 2 * meanSpacing);
}

bool kept(const Triangulation& triangulation, Triangulation::Face_handle face)
{
    return !triangulation.is_infinite(face) && face->info().kept;
}

Edge nextBoundaryEdge(const Triangulation& triangulation, Edge edge)
{
    auto [face, side] = edge;
    side = triangulation.ccw(side);
    // Follow the same fan of retained faces around the end vertex. Separate
    // regions that only meet at a vertex cannot redirect the boundary walk.
    while (kept(triangulation, face->neighbor(side))) {
        const auto next = face->neighbor(side);
        side = triangulation.ccw(next->index(face));
        face = next;
    }
    return {face, side};
}

void appendCycles(const std::vector<Vertex>& walk, std::vector<Polygon>& rings)
{
    std::vector<Vertex> path;
    std::unordered_map<std::size_t, std::size_t> positions;
    for (std::size_t i = 0; i <= walk.size(); ++i) {
        const auto vertex = walk[i % walk.size()];
        const auto [entry, inserted] = positions.emplace(vertex->info(), path.size());
        if (inserted) {
            path.push_back(vertex);
            continue;
        }
        const auto start = entry->second;
        if (path.size() - start >= 3) {
            Polygon ring;
            for (std::size_t j = start; j < path.size(); ++j) ring.push_back(path[j]->point());
            if (ring.is_simple() && ring.orientation() != CGAL::COLLINEAR) rings.push_back(std::move(ring));
        }
        for (std::size_t j = start + 1; j < path.size(); ++j) positions.erase(path[j]->info());
        path.resize(start + 1);
    }
}

std::size_t discardWeakRegions(Triangulation& triangulation, std::size_t minimumPoints)
{
    // Connected retained triangles define material. Their distinct vertices
    // support the outer region, with samples inside holes counted separately.
    std::vector<std::size_t> vertexStamp(triangulation.number_of_vertices(), 0);
    std::vector<Triangulation::Face_handle> component;
    std::size_t stamp = 0, discarded = 0;
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        if (!face->info().kept || face->info().componentVisited) continue;
        ++stamp;
        component.clear();
        component.push_back(face);
        face->info().componentVisited = true;
        std::size_t points = 0;
        for (std::size_t i = 0; i < component.size(); ++i) {
            const auto current = component[i];
            for (int side = 0; side < 3; ++side) {
                if (points < minimumPoints) {
                    auto& seen = vertexStamp[current->vertex(side)->info()];
                    if (seen != stamp) {
                        seen = stamp;
                        ++points;
                    }
                }
                const auto neighbor = current->neighbor(side);
                if (!kept(triangulation, neighbor) || neighbor->info().componentVisited) continue;
                neighbor->info().componentVisited = true;
                component.push_back(neighbor);
            }
        }
        if (points >= minimumPoints) continue;
        for (const auto triangle : component) triangle->info().kept = false;
        ++discarded;
    }
    return discarded;
}

std::vector<Polygon> traceRings(Triangulation& triangulation, double radius,
                               std::size_t minimumPoints, std::size_t& discarded)
{
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        face->info().kept = CGAL::squared_radius(face->vertex(0)->point(), face->vertex(1)->point(),
            face->vertex(2)->point()) <= radius * radius;
    }
    if (minimumPoints > 3) discarded = discardWeakRegions(triangulation, minimumPoints);
    std::vector<Polygon> rings;
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        if (!face->info().kept) continue;
        for (int side = 0; side < 3; ++side) {
            if (face->info().visited[side] || kept(triangulation, face->neighbor(side))) continue;
            const Edge start{face, side};
            Edge edge = start;
            std::vector<Vertex> walk;
            do {
                auto [current, index] = edge;
                current->info().visited[index] = true;
                walk.push_back(current->vertex(triangulation.ccw(index)));
                edge = nextBoundaryEdge(triangulation, edge);
            } while (edge != start);
            appendCycles(walk, rings);
        }
    }
    return rings;
}

void simplify(std::vector<Polygon>& rings, double tolerance)
{
    if (rings.empty() || tolerance == 0) return;
    namespace ps = CGAL::Polyline_simplification_2;
    using Data = CGAL::Triangulation_data_structure_2<ps::Vertex_base_2<Kernel>,
        CGAL::Constrained_triangulation_face_base_2<Kernel>>;
    using Cdt = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Data, CGAL::Exact_predicates_tag>;
    CGAL::Constrained_triangulation_plus_2<Cdt> constraints;
    std::vector<decltype(constraints)::Constraint_id> ids;
    for (const auto& ring : rings)
        ids.push_back(constraints.insert_constraint(ring.vertices_begin(), ring.vertices_end(), true));

    ps::simplify(constraints, ps::Squared_distance_cost{}, ps::Stop_above_cost_threshold{tolerance * tolerance});
    std::vector<Polygon> simplified;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        Polygon ring;
        for (const auto vertex : constraints.vertices_in_constraint(ids[i])) ring.push_back(vertex->point());
        if (ring.size() > 1 && ring[0] == ring[ring.size() - 1]) ring.erase(ring.vertices_end() - 1);
        if (ring.size() < 3 || !ring.is_simple() || ring.orientation() != rings[i].orientation()) return;
        // Closed constraints can retain an anchor in the middle of a straight edge.
        Polygon corners;
        for (std::size_t j = 0; j < ring.size(); ++j) {
            const auto& previous = ring[(j + ring.size() - 1) % ring.size()];
            const auto& next = ring[(j + 1) % ring.size()];
            if (!CGAL::collinear(previous, ring[j], next)) corners.push_back(ring[j]);
        }
        simplified.push_back(std::move(corners));
    }
    rings = std::move(simplified);
}

Ring outputRing(const Polygon& polygon)
{
    Ring result;
    auto first = std::min_element(polygon.vertices_begin(), polygon.vertices_end());
    for (auto point = first; point != polygon.vertices_end(); ++point) result.push_back({point->x(), point->y()});
    for (auto point = polygon.vertices_begin(); point != first; ++point) result.push_back({point->x(), point->y()});
    return result;
}

std::vector<Contour> groupRings(std::vector<Polygon> rings)
{
    std::stable_sort(rings.begin(), rings.end(), [](const Polygon& a, const Polygon& b) {
        return std::abs(a.area()) > std::abs(b.area());
    });
    std::vector<Contour> contours;
    std::vector<const Polygon*> outers;
    for (const auto& ring : rings) {
        if (ring.is_counterclockwise_oriented()) {
            outers.push_back(&ring);
            contours.push_back({outputRing(ring), {}});
            continue;
        }
        // Larger rings were processed first. The last containing outer is the
        // smallest one, so a nested island receives its own holes.
        const auto containing = std::find_if(outers.rbegin(), outers.rend(), [&](const Polygon* outer) {
            for (const auto& point : ring) {
                const auto side = outer->bounded_side(point);
                if (side != CGAL::ON_BOUNDARY) return side == CGAL::ON_BOUNDED_SIDE;
            }
            return false;
        });
        if (containing != outers.rend())
            contours[std::size_t(containing.base() - outers.begin() - 1)].holes.push_back(outputRing(ring));
    }
    return contours;
}

} // namespace

Result fitContours(const std::vector<Point2D>& points, const Options& options)
{
    const auto valid = [](double value) { return std::isfinite(value) && value >= 0; };
    if (!valid(options.alphaRadius) || (options.simplificationTolerance && !valid(*options.simplificationTolerance)))
        throw std::invalid_argument("Contour fitting options must be finite and nonnegative");
    if (!valid(options.minimumRegionFraction) || options.minimumRegionFraction > 1)
        throw std::invalid_argument("Minimum region fraction must be in [0, 1]");
    if (!std::isfinite(options.automaticAlphaScale) || options.automaticAlphaScale <= 0)
        throw std::invalid_argument("Automatic alpha scale must be finite and positive");
    std::vector<Point> input;
    input.reserve(points.size());
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            throw std::invalid_argument("Contour fitting requires finite 2D coordinates");
        input.emplace_back(point.x, point.y);
    }
    std::sort(input.begin(), input.end());
    input.erase(std::unique(input.begin(), input.end()), input.end());
    Result result;
    result.uniquePointCount = input.size();
    result.alphaRadius = options.alphaRadius;
    result.simplificationTolerance = options.simplificationTolerance.value_or(options.alphaRadius / 10);
    if (input.size() < 3) return result;
    Triangulation triangulation(input.begin(), input.end());
    if (triangulation.dimension() != 2) return result;

    std::size_t index = 0;
    for (auto vertex = triangulation.finite_vertices_begin(); vertex != triangulation.finite_vertices_end(); ++vertex)
        vertex->info() = index++;
    if (result.alphaRadius == 0) result.alphaRadius = estimateRadius(triangulation)*options.automaticAlphaScale;
    result.simplificationTolerance = options.simplificationTolerance.value_or(result.alphaRadius / 10);
    auto fractionMinimum = std::size_t(options.minimumRegionFraction * result.uniquePointCount);
    if (double(fractionMinimum) / double(result.uniquePointCount) < options.minimumRegionFraction)
        ++fractionMinimum;
    const auto minimumPoints = std::max(options.minimumRegionPoints, fractionMinimum);
    auto rings = traceRings(triangulation, result.alphaRadius, minimumPoints, result.discardedRegionCount);
    simplify(rings, result.simplificationTolerance);
    result.contours = groupRings(std::move(rings));
    return result;
}

} // namespace contour_fitting
