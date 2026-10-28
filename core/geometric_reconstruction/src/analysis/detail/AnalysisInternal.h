#pragma once
#include "reconstruction/analysis/SurfaceAnalysis.h"
#include "common/Math3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace reconstruction::detail::analysis {
using namespace math3;
using modeling::Vec3;

struct Key {
    std::int64_t x, y, z;
    bool operator==(const Key& b) const { return x == b.x && y == b.y && z == b.z; }
};
struct Hash {
    std::size_t operator()(Key k) const {
        auto h = std::hash<std::int64_t>{}(k.x);
        h ^= std::hash<std::int64_t>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h ^ (std::hash<std::int64_t>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2));
    }
};
// Relative coordinates keep index arithmetic bounded for far-away world frames.
class Grid {
    double cell;
    Vec3 origin;
    std::unordered_map<Key, std::vector<std::size_t>, Hash> bins;
public:
    const std::vector<const PointSample*>& data;
    Grid(const std::vector<const PointSample*>& p, double radius)
        : cell(radius), origin(p.empty() ? Vec3{} : p.front()->position), data(p) {
        for (std::size_t i = 0; i < p.size(); ++i) bins[key(p[i]->position)].push_back(i);
    }
    Key key(Vec3 p) const {
        const auto q = scale(subtract(p, origin), 1 / cell);
        // A single extreme outlier must not overflow an integer grid key.
        auto coord = [](double x) { return static_cast<std::int64_t>(std::clamp(std::floor(x), -1e15, 1e15)); };
        return {coord(q.x), coord(q.y), coord(q.z)};
    }
    template<class F> void near(Vec3 p, F f) const {
        const auto k = key(p);
        for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
            auto it = bins.find({k.x+x, k.y+y, k.z+z});
            if (it == bins.end()) continue;
            for (auto i : it->second) {
                const auto d = subtract(p, data[i]->position);
                if (dot(d, d) <= cell*cell) f(i);
            }
        }
    }
};

const std::vector<PointId>& ids(const PrimitiveEvidence& e);

double confidence(const PrimitiveEvidence& e);

std::unordered_map<PointId, const PointSample*> pointMap(const PointStore& points);

Vec3 axis(const PrimitiveEvidence& e);

Vec3 center(const PrimitiveEvidence& e);

double radius(const PrimitiveEvidence& e);

bool axial(PrimitiveKind k);

bool valid(const SurfaceAnalysisOptions& o);
} // namespace reconstruction::detail::analysis
