#include "detail/CurvedInternal.h"

namespace reconstruction::detail::curved {
std::vector<Cap> caps(const PointStore &points, Vec3 origin, Vec3 axis, double tolerance,
                      const std::function<double(double)> &radius) {
    struct Item {
        double t;
        const PointSample *p;
        int sign;
    };
    std::vector<Item> items;
    for (const auto &p : points.points()) {
        const double projection = dot(normalized(*p.normal), axis);
        if (std::abs(projection) < 0.98)
            continue;
        auto d = subtract(p.position, origin);
        const double t = dot(d, axis);
        if (radialLength(d, axis) > radius(t) + 2 * tolerance)
            continue;
        items.push_back({t, &p, projection > 0 ? 1 : -1});
    }
    std::sort(items.begin(), items.end(), [](auto a, auto b) { return a.t < b.t; });
    std::vector<Cap> result;
    for (std::size_t i = 0; i < items.size();) {
        auto end = i + 1;
        while (end < items.size() && items[end].t - items[i].t <= 4 * tolerance)
            ++end;
        for (int sign : {-1, 1}) {
            Cap cap;
            cap.sign = sign;
            for (auto j = i; j < end; ++j)
                if (items[j].sign == sign)
                    cap.data.push_back(items[j].p);
            if (cap.data.size() >= std::max<std::size_t>(12, points.size() / 1000)) {
                std::sort(cap.data.begin(), cap.data.end(),
                          [](auto a, auto b) { return a->id < b->id; });
                for (auto p : cap.data)
                    cap.t += dot(subtract(p->position, origin), axis);
                cap.t /= cap.data.size();
                result.push_back(std::move(cap));
            }
        }
        i = end;
    }
    return result;
}

bool ends(const std::vector<Cap> &planes, double &start, double &end, bool pointed) {
    const Cap *low = nullptr;
    const Cap *high = nullptr;
    for (const auto &cap : planes) {
        auto *&chosen = cap.sign < 0 ? low : high;
        if (!chosen || cap.data.size() > chosen->data.size())
            chosen = &cap;
    }
    if (!high || (!pointed && !low))
        return false;
    start = low ? low->t : 0;
    end = high->t;
    return end > start;
}
} // namespace reconstruction::detail::curved
