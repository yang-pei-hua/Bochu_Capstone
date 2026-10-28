#include "detail/CompoundInternal.h"

namespace reconstruction::detail::compound {
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
} // namespace reconstruction::detail::compound
