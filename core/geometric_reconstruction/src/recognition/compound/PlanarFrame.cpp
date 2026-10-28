#include "detail/CompoundInternal.h"

namespace reconstruction::detail::compound {
bool recoverPlanarFrame(const std::vector<PointSample>& points, const CompoundRecognitionOptions& options,
                        Arrangement& arrangement, std::string& error) {
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
        const PlaneEvidence* best=nullptr;
        for(const auto& e : options.boundaryEvidence)
            if(std::abs(dot(e.plane.normal,normal))>options.normalAlignment &&
               (!best || e.supportPointIds.size()>best->supportPointIds.size())) best=&e;
        if(best) return best->plane.normal;
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
    auto& axes=arrangement.axes;
    axes={
        {first,
         math3::normalized(subtract(refinedSecond, scale(first, dot(first, refinedSecond)))),
         {}}};
    axes[2] = math3::normalized(math3::cross(axes[0], axes[1]));
    if(!options.interfacePlanes.empty()) {
        // Inherited host contact is a shared geometric constraint. Independent
        // noisy local frames must not open a gap between a boss and its host.
        auto n=math3::normalized(options.interfacePlanes.front().normal);
        int fixed=0;
        for(int i=1;i<3;++i) if(std::abs(dot(axes[i],n))>std::abs(dot(axes[fixed],n))) fixed=i;
        if(dot(axes[fixed],n)<0) n=scale(n,-1);
        const int next=(fixed+1)%3,last=(fixed+2)%3;
        axes[fixed]=n;
        axes[next]=math3::normalized(subtract(axes[next],scale(n,dot(axes[next],n))));
        axes[last]=math3::normalized(math3::cross(axes[fixed],axes[next]));
    }
    auto& boundaries=arrangement.boundaries;
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
        for (const auto& plane : options.interfacePlanes) {
            const double alignment=dot(plane.normal,axes[a]);
            if (std::abs(alignment)>options.normalAlignment)
                boundaries[a].push_back(-plane.offset/alignment);
        }
        std::sort(boundaries[a].begin(),boundaries[a].end());
        boundaries[a].erase(std::unique(boundaries[a].begin(),boundaries[a].end(),[&](double a,double b){
            return std::abs(a-b)<options.distanceTolerance*4;
        }),boundaries[a].end());
        if (boundaries[a].size() < 2 || boundaries[a].size() > 10) {
            error = "Insufficient or excessive planar boundaries";
            return false;
        }
    }
    auto& obliques=arrangement.obliques;
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
    return true;
}
} // namespace reconstruction::detail::compound
