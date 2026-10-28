#pragma once
#include "CompoundInternal.h"

namespace reconstruction::detail::compound {
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
} // namespace reconstruction::detail::compound
