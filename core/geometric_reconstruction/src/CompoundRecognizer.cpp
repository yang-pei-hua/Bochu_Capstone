#include "reconstruction/CompoundRecognizer.h"
#include "Math3.h"
#include "reconstruction/ThroughHoleRecognizer.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace reconstruction {
namespace {
using math3::add;
using math3::dot;
using math3::scale;
using math3::subtract;
using modeling::Vec3;
using Coord = std::array<double, 3>;
double component(const Vec3 &p, int a) {
    return a == 0 ? p.x : a == 1 ? p.y : p.z;
}
struct Node {
    int point = -1, left = -1, right = -1, axis = 0;
};
// Deterministic nearest-neighbour index, independent of RANSAC's random state.
class Nearest {
    const std::vector<PointSample> &points;
    double margin;
    std::vector<Node> nodes;
    int build(std::vector<int> &ids, int lo, int hi, int depth) {
        if (lo >= hi)
            return -1;
        const int mid = (lo + hi) / 2, axis = depth % 3;
        std::nth_element(ids.begin() + lo, ids.begin() + mid, ids.begin() + hi, [&](int a, int b) {
            return component(points[a].position, axis) < component(points[b].position, axis);
        });
        const int id = int(nodes.size());
        nodes.push_back({ids[mid], -1, -1, axis});
        const int left = build(ids, lo, mid, depth + 1), right = build(ids, mid + 1, hi, depth + 1);
        nodes[id].left = left;
        nodes[id].right = right;
        return id;
    }
    void visit(int id, const Vec3 &q, double &best, int &index) const {
        if (id < 0)
            return;
        const auto &n = nodes[id];
        const auto d = subtract(q, points[n.point].position);
        const double distance = dot(d, d);
        if (distance < best) {
            best = distance;
            index = n.point;
        }
        const double delta = component(d, n.axis);
        visit(delta < 0 ? n.left : n.right, q, best, index);
        if (delta * delta < best)
            visit(delta < 0 ? n.right : n.left, q, best, index);
    }
    void orientation(int id, const Vec3 &q, double radius, double &score, double &sign) const {
        if (id < 0)
            return;
        const auto &node = nodes[id];
        const auto &point = points[node.point];
        auto d = subtract(q, point.position);
        const double distance = dot(d, d);
        if (distance <= radius && point.normal) {
            const double projection = dot(d, *point.normal);
            // At a boundary edge the closest sample's tangent plane may extend
            // into empty space. Prefer a nearby face whose normal actually
            // points toward the query, rather than extrapolating that edge.
            const double alignment = std::abs(projection) / std::sqrt(std::max(1e-12, distance));
            if (alignment > score) {
                score = alignment;
                sign = projection;
            }
        }
        const double delta = component(d, node.axis);
        orientation(delta < 0 ? node.left : node.right, q, radius, score, sign);
        if (delta * delta <= radius)
            orientation(delta < 0 ? node.right : node.left, q, radius, score, sign);
    }

  public:
    explicit Nearest(const std::vector<PointSample> &p, double tolerance)
        : points(p), margin(tolerance * 8) {
        std::vector<int> ids(p.size());
        std::iota(ids.begin(), ids.end(), 0);
        nodes.reserve(p.size());
        build(ids, 0, int(ids.size()), 0);
    }
    bool inside(const Vec3 &q) const {
        double best = std::numeric_limits<double>::infinity();
        int index = -1;
        visit(0, q, best, index);
        if (index < 0)
            return false;
        double score = -1, sign = 0;
        orientation(0, q, best * 1.5 + margin * margin, score, sign);
        return sign < 0;
    }
};
struct Direction {
    Vec3 sum{};
    Vec3 axis{};
    std::size_t count = 0;
};
struct Oblique {
    Coord n{};
    double offset = 0;
    int extrusion = 0;
};
struct Cell {
    std::array<int, 3> index{};
    bool full = false;
};
struct Rectangle {
    std::array<int, 3> lo{}, hi{};
    std::vector<int> cells;
    double volume = 0;
};
Vec3 world(const Coord &p, const std::array<Vec3, 3> &axes) {
    return add(add(scale(axes[0], p[0]), scale(axes[1], p[1])), scale(axes[2], p[2]));
}
std::vector<modeling::Point2D> clipPolygon(std::vector<modeling::Point2D> polygon, double a,
                                           double b, double c) {
    std::vector<modeling::Point2D> result;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        auto p = polygon[i], q = polygon[(i + 1) % polygon.size()];
        const double dp = a * p.x + b * p.y + c, dq = a * q.x + b * q.y + c;
        if (dp <= 0)
            result.push_back(p);
        if ((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) {
            const double t = dp / (dp - dq);
            result.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)});
        }
    }
    return result;
}
} // namespace

bool reconstructCompound(const PointStore &store, const CompoundRecognitionOptions &options,
                         CompoundCandidate &output, std::string &error) {
    output = {};
    error.clear();
    if (store.unit() != LengthUnit::Millimeter || store.size() < 30 ||
        !(options.distanceTolerance > 0) ||
        !(options.normalAlignment > 0.9 && options.normalAlignment < 1) ||
        !(options.maximumCellError >= 0 && options.maximumCellError < 0.5)) {
        error = "Compound fitting requires millimetre points, outward normals and valid tolerances";
        return false;
    }
    const auto &points = store.points();
    std::vector<Direction> directions;
    for (const auto &p : points) {
        if (!p.normal || math3::length(*p.normal) < 0.5) {
            error = "Compound fitting requires oriented normals";
            return false;
        }
        const auto normal = math3::canonicalDirection(*p.normal);
        auto found = std::find_if(directions.begin(), directions.end(), [&](const auto &g) {
            return dot(normal, g.axis) > options.normalAlignment;
        });
        if (found == directions.end())
            directions.push_back({normal, normal, 1});
        else {
            found->sum = add(found->sum, normal);
            found->axis = math3::normalized(found->sum);
            ++found->count;
        }
    }
    std::sort(directions.begin(), directions.end(),
              [](auto a, auto b) { return a.count > b.count; });
    const std::size_t minimumSupport = std::max<std::size_t>(20, points.size() / 200);
    auto refineDirection = [&](Vec3 normal) {
        std::vector<std::pair<double, Vec3>> data;
        for (const auto &p : points)
            if (std::abs(dot(math3::normalized(*p.normal), normal)) > options.normalAlignment)
                data.push_back({dot(p.position, normal), p.position});
        std::sort(data.begin(), data.end(), [](auto a, auto b) { return a.first < b.first; });
        std::size_t start = 0, bestStart = 0, bestEnd = 0;
        for (std::size_t end = 0; end < data.size(); ++end) {
            while (data[end].first - data[start].first > options.distanceTolerance * 4)
                ++start;
            if (end + 1 - start > bestEnd - bestStart) {
                bestStart = start;
                bestEnd = end + 1;
            }
        }
        if (bestEnd - bestStart < minimumSupport)
            return normal;
        Vec3 center{};
        for (std::size_t i = bestStart; i < bestEnd; ++i)
            center = add(center, data[i].second);
        center = scale(center, 1.0 / (bestEnd - bestStart));
        std::array<std::array<double, 3>, 3> covariance{};
        for (std::size_t i = bestStart; i < bestEnd; ++i) {
            auto d = subtract(data[i].second, center);
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    covariance[a][b] += component(d, a) * component(d, b);
        }
        return math3::smallestEigenvector(covariance);
    };
    const Vec3 first = refineDirection(directions.front().axis);
    auto second = std::find_if(directions.begin() + 1, directions.end(),
                               [&](auto g) { return std::abs(dot(first, g.axis)) < 0.1; });
    if (second == directions.end()) {
        error = "No supported orthogonal frame";
        return false;
    }
    const auto refinedSecond = refineDirection(second->axis);
    std::array<Vec3, 3> axes{
        {first,
         math3::normalized(subtract(refinedSecond, scale(first, dot(first, refinedSecond)))),
         {}}};
    axes[2] = math3::normalized(math3::cross(axes[0], axes[1]));
    std::array<std::vector<double>, 3> boundaries;
    auto levels = [&](std::vector<double> values) {
        std::vector<double> result;
        std::sort(values.begin(), values.end());
        for (std::size_t i = 0; i < values.size();) {
            std::size_t j = i + 1;
            // A bounded cluster prevents chains of noise/outliers joining two faces.
            while (j < values.size() && values[j] - values[i] < options.distanceTolerance * 4)
                ++j;
            if (j - i >= minimumSupport)
                result.push_back(values[(i + j) / 2]);
            i = j;
        }
        // Merge split tails of the same noisy face, preserving actual thin walls.
        std::vector<double> merged;
        for (double v : result) {
            if (!merged.empty() && v - merged.back() < options.distanceTolerance * 5)
                merged.back() = (v + merged.back()) / 2;
            else
                merged.push_back(v);
        }
        return merged;
    };
    for (int a = 0; a < 3; ++a) {
        std::vector<double> values;
        for (const auto &p : points)
            if (std::abs(dot(math3::normalized(*p.normal), axes[a])) > options.normalAlignment)
                values.push_back(dot(p.position, axes[a]));
        boundaries[a] = levels(std::move(values));
        if (boundaries[a].size() < 2 || boundaries[a].size() > 10) {
            error = "Insufficient or excessive planar boundaries";
            return false;
        }
    }
    std::vector<Oblique> obliques;
    for (const auto &group : directions) {
        if (group.count < minimumSupport)
            continue;
        const auto refinedNormal = refineDirection(group.axis);
        Coord n{{dot(refinedNormal, axes[0]), dot(refinedNormal, axes[1]),
                 dot(refinedNormal, axes[2])}};
        if (std::max({std::abs(n[0]), std::abs(n[1]), std::abs(n[2])}) > options.normalAlignment)
            continue;
        int extrusion =
            int(std::min_element(n.begin(), n.end(),
                                 [](double a, double b) { return std::abs(a) < std::abs(b); }) -
                n.begin());
        if (std::abs(n[extrusion]) > 0.05)
            continue;
        std::vector<double> offsets;
        for (const auto &p : points)
            if (std::abs(dot(math3::normalized(*p.normal), refinedNormal)) >
                options.normalAlignment)
                offsets.push_back(dot(p.position, refinedNormal));
        for (double offset : levels(std::move(offsets)))
            obliques.push_back({n, -offset, extrusion});
    }
    if (obliques.size() > 6) {
        error = "Curved or unsupported surface requires curved-feature recognition";
        return false;
    }
    std::array<int, 3> sizes{{int(boundaries[0].size() - 1), int(boundaries[1].size() - 1),
                              int(boundaries[2].size() - 1)}};
    const std::size_t total = std::size_t(sizes[0]) * sizes[1] * sizes[2];
    if (total > options.maximumGridCells) {
        error = "Planar arrangement exceeds grid limit";
        return false;
    }
    output.gridCells = total;
    Nearest nearest(points, options.distanceTolerance);
    std::vector<Cell> cells;
    double totalError = 0;
    for (int x = 0; x < sizes[0]; ++x)
        for (int y = 0; y < sizes[1]; ++y)
            for (int z = 0; z < sizes[2]; ++z) {
                Coord lo{{boundaries[0][x], boundaries[1][y], boundaries[2][z]}},
                    hi{{boundaries[0][x + 1], boundaries[1][y + 1], boundaries[2][z + 1]}};
                std::vector<std::pair<Coord, bool>> samples;
                int insideCount = 0;
                for (int i = 0; i < 5; ++i)
                    for (int j = 0; j < 5; ++j)
                        for (int k = 0; k < 5; ++k) {
                            Coord q{{lo[0] + (hi[0] - lo[0]) * (i + 0.5) / 5,
                                     lo[1] + (hi[1] - lo[1]) * (j + 0.5) / 5,
                                     lo[2] + (hi[2] - lo[2]) * (k + 0.5) / 5}};
                            const bool inside = nearest.inside(world(q, axes));
                            insideCount += inside;
                            samples.push_back({q, inside});
                        }
                const double ratio = insideCount / 125.0;
                if (ratio >= 1 - options.maximumCellError) {
                    cells.push_back({{{x, y, z}}, true});
                    totalError += 1 - ratio;
                    continue;
                }
                if (ratio <= options.maximumCellError) {
                    cells.push_back({{{x, y, z}}, false});
                    totalError += ratio;
                    continue;
                }
                double bestError = 1;
                const Oblique *best = nullptr;
                double bestSign = 1;
                for (const auto &plane : obliques)
                    for (double sign : {-1.0, 1.0}) {
                        int mismatch = 0;
                        for (const auto &sample : samples) {
                            double d = plane.offset;
                            for (int a = 0; a < 3; ++a)
                                d += plane.n[a] * sample.first[a];
                            mismatch += (sign * d <= 0) != sample.second;
                        }
                        if (mismatch / 125.0 < bestError) {
                            bestError = mismatch / 125.0;
                            best = &plane;
                            bestSign = sign;
                        }
                    }
                if (!best || bestError > options.maximumCellError) {
                    error = "Unexplained material cell " + std::to_string(x) + "," +
                            std::to_string(y) + "," + std::to_string(z) +
                            "; occupancy=" + std::to_string(ratio) +
                            "; best prism error=" + std::to_string(bestError);
                    return false;
                }
                totalError += bestError;
                cells.push_back({{{x, y, z}}, false});
                const int e = best->extrusion, u = (e + 1) % 3, v = (e + 2) % 3;
                auto polygon = clipPolygon(
                    {{lo[u], lo[v]}, {hi[u], lo[v]}, {hi[u], hi[v]}, {lo[u], hi[v]}},
                    bestSign * best->n[u], bestSign * best->n[v], bestSign * best->offset);
                if (polygon.size() < 3) {
                    error = "Degenerate prism intersection";
                    return false;
                }
                // cyclic axes ensure normal cross xDirection equals the polygon Y axis.
                Coord origin{};
                origin[e] = lo[e];
                PrismCandidate prism;
                prism.plane = {world(origin, axes), axes[e], axes[u]};
                prism.depth = hi[e] - lo[e];
                prism.polygon = std::move(polygon);
                output.prisms.push_back(std::move(prism));
            }
    output.occupancyError = totalError / total;
    auto index = [&](int x, int y, int z) { return (x * sizes[1] + y) * sizes[2] + z; };
    std::vector<Rectangle> rectangles;
    for (int x = 0; x < sizes[0]; ++x)
        for (int X = x + 1; X <= sizes[0]; ++X)
            for (int y = 0; y < sizes[1]; ++y)
                for (int Y = y + 1; Y <= sizes[1]; ++Y)
                    for (int z = 0; z < sizes[2]; ++z)
                        for (int Z = z + 1; Z <= sizes[2]; ++Z) {
                            Rectangle r;
                            r.lo = {{x, y, z}};
                            r.hi = {{X, Y, Z}};
                            bool valid = true;
                            for (int i = x; i < X && valid; ++i)
                                for (int j = y; j < Y && valid; ++j)
                                    for (int k = z; k < Z; ++k) {
                                        int id = index(i, j, k);
                                        if (!cells[id].full) {
                                            valid = false;
                                            break;
                                        }
                                        r.cells.push_back(id);
                                    }
                            if (valid) {
                                r.volume = (boundaries[0][X] - boundaries[0][x]) *
                                           (boundaries[1][Y] - boundaries[1][y]) *
                                           (boundaries[2][Z] - boundaries[2][z]);
                                rectangles.push_back(std::move(r));
                            }
                        }
    // Remove contained candidates. With overlap permitted, replacing a box by
    // a larger admissible box never increases the count or includes empty cells.
    std::vector<Rectangle> maximal;
    for (std::size_t i = 0; i < rectangles.size(); ++i) {
        bool contained = false;
        for (std::size_t j = 0; j < rectangles.size() && !contained; ++j)
            if (i != j && rectangles[j].cells.size() > rectangles[i].cells.size()) {
                contained = true;
                for (int a = 0; a < 3; ++a)
                    contained &= rectangles[j].lo[a] <= rectangles[i].lo[a] &&
                                 rectangles[j].hi[a] >= rectangles[i].hi[a];
            }
        if (!contained)
            maximal.push_back(rectangles[i]);
    }
    rectangles = std::move(maximal);
    std::sort(rectangles.begin(), rectangles.end(),
              [](const auto &a, const auto &b) { return a.cells.size() > b.cells.size(); });
    std::vector<std::vector<int>> containing(total);
    for (std::size_t i = 0; i < rectangles.size(); ++i)
        for (int c : rectangles[i].cells)
            containing[c].push_back(int(i));
    std::vector<bool> covered(total, false);
    std::vector<int> chosen, bestCover;
    std::size_t fullCount = 0;
    for (const auto &c : cells)
        fullCount += c.full;
    // A greedy cover bounds the exact branch-and-bound search immediately.
    std::size_t remaining = fullCount;
    while (remaining) {
        int winner = -1;
        std::size_t gain = 0;
        for (std::size_t i = 0; i < rectangles.size(); ++i) {
            std::size_t g = 0;
            for (int c : rectangles[i].cells)
                g += !covered[c];
            if (g > gain) {
                gain = g;
                winner = int(i);
            }
        }
        if (winner < 0) {
            error = "Material cell has no admissible box";
            return false;
        }
        bestCover.push_back(winner);
        for (int c : rectangles[winner].cells)
            if (!covered[c]) {
                covered[c] = true;
                --remaining;
            }
    }
    std::fill(covered.begin(), covered.end(), false);
    bool exhausted = false;
    std::function<void(std::size_t)> search = [&](std::size_t left) {
        if (++output.searchNodes > options.maximumSearchNodes) {
            exhausted = true;
            return;
        }
        if (!left) {
            if (chosen.size() < bestCover.size())
                bestCover = chosen;
            return;
        }
        if (chosen.size() + 1 >= bestCover.size())
            return;
        int pivot = -1;
        std::size_t alternatives = std::numeric_limits<std::size_t>::max();
        for (std::size_t c = 0; c < total; ++c)
            if (cells[c].full && !covered[c] && containing[c].size() < alternatives) {
                pivot = int(c);
                alternatives = containing[c].size();
            }
        if (pivot < 0)
            return;
        for (int r : containing[pivot]) {
            std::vector<int> changed;
            for (int c : rectangles[r].cells)
                if (!covered[c]) {
                    covered[c] = true;
                    changed.push_back(c);
                }
            chosen.push_back(r);
            search(left - changed.size());
            chosen.pop_back();
            for (int c : changed)
                covered[c] = false;
            if (exhausted)
                return;
        }
    };
    search(fullCount);
    output.minimumBoxCountProven = !exhausted;
    for (int r : bestCover) {
        const auto &rect = rectangles[r];
        Coord origin{};
        BoxCandidate box;
        for (int a = 0; a < 3; ++a)
            origin[a] = boundaries[a][rect.lo[a]];
        box.primitive.pose = {world(origin, axes), axes[0], axes[2]};
        box.primitive.sizeX = boundaries[0][rect.hi[0]] - origin[0];
        box.primitive.sizeY = boundaries[1][rect.hi[1]] - origin[1];
        box.primitive.sizeZ = boundaries[2][rect.hi[2]] - origin[2];
        box.confidence = 1 - output.occupancyError;
        output.boxes.push_back(box);
    }
    if (output.boxes.empty() && output.prisms.empty()) {
        error = "No supported material";
        return false;
    }
    auto modelContains = [&](const Vec3 &p) {
        for (const auto &box : output.boxes) {
            auto d = subtract(p, box.primitive.pose.origin);
            Coord t{{dot(d, axes[0]), dot(d, axes[1]), dot(d, axes[2])}};
            if (t[0] >= 0 && t[0] <= box.primitive.sizeX && t[1] >= 0 &&
                t[1] <= box.primitive.sizeY && t[2] >= 0 && t[2] <= box.primitive.sizeZ)
                return true;
        }
        for (const auto &prism : output.prisms) {
            const auto d = subtract(p, prism.plane.origin),
                       v = math3::cross(prism.plane.normal, prism.plane.xDirection);
            const double depth = dot(d, prism.plane.normal), x = dot(d, prism.plane.xDirection),
                         y = dot(d, v);
            if (depth < 0 || depth > prism.depth)
                continue;
            bool in = true;
            for (std::size_t i = 0; i < prism.polygon.size(); ++i) {
                auto a = prism.polygon[i], b = prism.polygon[(i + 1) % prism.polygon.size()];
                if ((b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x) < -1e-9) {
                    in = false;
                    break;
                }
            }
            if (in)
                return true;
        }
        return false;
    };
    std::size_t supported = 0;
    for (const auto &p : points) {
        const auto shift = scale(math3::normalized(*p.normal), options.distanceTolerance * 4);
        supported +=
            modelContains(subtract(p.position, shift)) && !modelContains(add(p.position, shift));
    }
    output.surfaceCoverage = double(supported) / points.size();
    if (output.surfaceCoverage < 0.90) {
        error = "Compound surface coverage too low: " + std::to_string(output.surfaceCoverage);
        return false;
    }
    output.confidence = 1 - output.occupancyError;
    return true;
}

bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error) {
    CompoundRecognitionOptions options;
    options.distanceTolerance = planes.distanceThreshold;
    if (reconstructCompound(points, options, output, error))
        return true;
    const auto compoundError = error;
    BoxWithThroughHolesCandidate legacy;
    if (!reconstructBoxWithThroughHoles(points, planes, cylinders, BoxRecognitionOptions{},
                                        ThroughHoleRecognitionOptions{}, legacy, error)) {
        error = compoundError + "; curved-feature fallback: " + error;
        output = {};
        return false;
    }
    const auto &box = legacy.box.primitive;
    const auto x = math3::normalized(box.pose.xDirection),
               z = math3::normalized(box.pose.zDirection), y = math3::cross(z, x);
    auto contains = [&](const Vec3 &point) {
        auto delta = subtract(point, box.pose.origin);
        const Coord local{{dot(delta, x), dot(delta, y), dot(delta, z)}};
        if (local[0] < 0 || local[0] > box.sizeX || local[1] < 0 || local[1] > box.sizeY ||
            local[2] < 0 || local[2] > box.sizeZ)
            return false;
        for (const auto &hole : legacy.throughHoles) {
            const auto direction = math3::normalized(hole.primitive.axisDirection);
            auto d = subtract(point, hole.primitive.axisPoint);
            auto radial = subtract(d, scale(direction, dot(d, direction)));
            if (dot(radial, radial) < hole.primitive.radius * hole.primitive.radius)
                return false;
        }
        return true;
    };
    std::size_t support = 0;
    for (const auto &p : points.points())
        if (p.normal) {
            const auto shift = scale(math3::normalized(*p.normal), planes.distanceThreshold * 4);
            support += contains(subtract(p.position, shift)) && !contains(add(p.position, shift));
        }
    const double coverage = double(support) / points.size();
    // Curved fused scan normals have larger boundary uncertainty than planar
    // normals. Still require the whole-body hypothesis to explain most points.
    if (coverage < 0.80) {
        error = compoundError + "; single-box fallback rejected: complete surface coverage=" +
                std::to_string(coverage);
        output = {};
        return false;
    }
    output = {};
    output.boxes.push_back(legacy.box);
    output.throughHoles = legacy.throughHoles;
    output.surfaceCoverage = coverage;
    output.confidence = legacy.confidence;
    output.minimumBoxCountProven = true;
    error.clear();
    return true;
}
} // namespace reconstruction
