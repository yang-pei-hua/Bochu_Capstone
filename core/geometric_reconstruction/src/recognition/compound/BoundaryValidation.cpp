#include "detail/CompoundInternal.h"

namespace reconstruction::detail::compound {
bool validateCompoundBoundary(const std::vector<PointSample>& points, const CompoundRecognitionOptions& options,
                              const std::array<Vec3,3>& axes, CompoundCandidate& output, std::string& error) {
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
} // namespace reconstruction::detail::compound
