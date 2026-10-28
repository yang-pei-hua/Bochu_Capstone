#pragma once
#include "reconstruction/recognition/CurvedSolidRecognizer.h"
#include "common/Math3.h"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <set>
#include <type_traits>

namespace reconstruction::detail::curved {
using modeling::Vec3;
using namespace math3;
constexpr double pi = 3.14159265358979323846;
using Params = modeling::CurvedPrimitiveParams;

struct Cap {
    double t = 0;
    int sign = 0;
    std::vector<const PointSample *> data;
};
struct Boundary {
    double distance = INFINITY;
    Vec3 normal{};
};
template <class Evidence>
std::vector<const PointSample *> support(const PointStore &points, const Evidence &e) {
    std::set<PointId> ids(e.supportPointIds.begin(), e.supportPointIds.end());
    std::vector<const PointSample *> data;
    for (const auto &p : points.points())
        if (ids.count(p.id))
            data.push_back(&p);
    std::sort(data.begin(), data.end(), [](auto a, auto b) { return a->id < b->id; });
    return data;
}

Vec3 vec(const Eigen::VectorXd &x, int i);

double radialLength(Vec3 d, Vec3 axis);

Vec3 transverse(Vec3 axis);

int angleBin(double y, double x, int count);

int unitBin(double value, int count);

bool refine(Eigen::VectorXd &x, const std::vector<const PointSample *> &data, double tolerance,
            const std::function<double(const Eigen::VectorXd &, Vec3)> &residual);

double sphereResidual(const Eigen::VectorXd &x, Vec3 p);

double coneResidual(const Eigen::VectorXd &x, Vec3 p);

double torusResidual(const Eigen::VectorXd &x, Vec3 p);

std::vector<Cap> caps(const PointStore &points, Vec3 origin, Vec3 axis, double tolerance,
                      const std::function<double(double)> &radius);

bool ends(const std::vector<Cap> &planes, double &start, double &end, bool pointed);

Boundary boundary(const Params &params, Vec3 p);

bool assess(const PointStore &points, const CurvedSolidRecognitionOptions &options,
            CurvedSolidCandidate &result);
} // namespace reconstruction::detail::curved
