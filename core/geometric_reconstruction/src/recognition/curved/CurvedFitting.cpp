#include "detail/CurvedInternal.h"

namespace reconstruction::detail::curved {
Vec3 vec(const Eigen::VectorXd &x, int i) { return {x[i], x[i + 1], x[i + 2]}; }

double radialLength(Vec3 d, Vec3 axis) { return length(subtract(d, scale(axis, dot(d, axis)))); }

Vec3 transverse(Vec3 axis) {
    return normalized(cross(axis, std::abs(axis.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}));
}

int angleBin(double y, double x, int count) {
    double a = std::atan2(y, x);
    if (a < 0)
        a += 2 * pi;
    return std::clamp(int(a / (2 * pi) * count), 0, count - 1);
}

int unitBin(double value, int count) { return std::clamp(int(value * count), 0, count - 1); }

bool refine(Eigen::VectorXd &x, const std::vector<const PointSample *> &data, double tolerance,
            const std::function<double(const Eigen::VectorXd &, Vec3)> &residual) {
    if (data.size() < std::size_t(x.size() * 4))
        return false;
    auto cost = [&](const Eigen::VectorXd &p) -> double {
        double sum = 0;
        for (auto point : data) {
            const double r = residual(p, point->position);
            if (!std::isfinite(r))
                return INFINITY;
            sum += std::abs(r) <= tolerance ? r * r
                                            : 2 * tolerance * std::abs(r) - tolerance * tolerance;
        }
        return sum;
    };
    for (int iteration = 0; iteration < 15; ++iteration) {
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(x.size(), x.size());
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(x.size());
        for (auto point : data) {
            const double r = residual(x, point->position);
            if (!std::isfinite(r))
                return false;
            Eigen::VectorXd gradient(x.size());
            for (int j = 0; j < x.size(); ++j) {
                const double delta = 1e-5 * std::max(1.0, std::abs(x[j]));
                auto plus = x, minus = x;
                plus[j] += delta;
                minus[j] -= delta;
                gradient[j] = (residual(plus, point->position) - residual(minus, point->position)) /
                              (2 * delta);
            }
            const double weight = std::min(1.0, tolerance / std::max(std::abs(r), 1e-12));
            matrix += weight * gradient * gradient.transpose();
            rhs -= weight * gradient * r;
        }
        matrix.diagonal().array() += 1e-8;
        const Eigen::VectorXd step = matrix.ldlt().solve(rhs);
        if (!step.allFinite())
            return false;
        const double before = cost(x);
        bool moved = false;
        for (double fraction = 1; fraction >= 1.0 / 64; fraction /= 2) {
            const Eigen::VectorXd next = x + fraction * step;
            if (cost(next) < before) {
                x = next;
                moved = true;
                break;
            }
        }
        if (!moved || step.norm() < 1e-9)
            break;
    }
    return x.allFinite();
}

double sphereResidual(const Eigen::VectorXd &x, Vec3 p) {
    return length(subtract(p, vec(x, 0))) - x[3];
}

double coneResidual(const Eigen::VectorXd &x, Vec3 p) {
    if (length(vec(x, 3)) < 1e-12 || x[6] <= 0 || x[6] >= pi / 2)
        return INFINITY;
    auto axis = normalized(vec(x, 3)), d = subtract(p, vec(x, 0));
    return radialLength(d, axis) * std::cos(x[6]) - dot(d, axis) * std::sin(x[6]);
}

double torusResidual(const Eigen::VectorXd &x, Vec3 p) {
    if (length(vec(x, 3)) < 1e-12 || x[7] <= 0 || x[6] <= x[7])
        return INFINITY;
    auto axis = normalized(vec(x, 3)), d = subtract(p, vec(x, 0));
    return std::hypot(radialLength(d, axis) - x[6], dot(d, axis)) - x[7];
}
} // namespace reconstruction::detail::curved
