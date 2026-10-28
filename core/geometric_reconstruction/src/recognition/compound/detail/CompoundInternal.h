#pragma once
#include "reconstruction/recognition/CompoundRecognizer.h"
#include "common/Math3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace reconstruction::detail::compound {
using math3::add;
using math3::dot;
using math3::scale;
using math3::subtract;
using modeling::Vec3;
using Coord = std::array<double, 3>;

inline double component(const Vec3 &p, int a) {
    return a == 0 ? p.x : a == 1 ? p.y : p.z;
}
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

struct Arrangement {
    std::array<Vec3,3> axes;
    std::array<std::vector<double>,3> boundaries;
    std::vector<Oblique> obliques;
    std::vector<Cell> cells;
};

Vec3 world(const Coord &p, const std::array<Vec3, 3> &axes);
std::vector<modeling::Point2D> clipPolygon(std::vector<modeling::Point2D> polygon, double a,
                                           double b, double c);

bool recoverPlanarFrame(const std::vector<PointSample>& points, const CompoundRecognitionOptions& options,
                        Arrangement& arrangement, std::string& error);
bool classifyMaterialCells(const PointStore& store, const CompoundRecognitionOptions& options,
                           Arrangement& arrangement, CompoundCandidate& output, std::string& error);
bool selectBoxCover(const Arrangement& arrangement, const CompoundRecognitionOptions& options,
                    CompoundCandidate& output, std::string& error);
bool validateCompoundBoundary(const std::vector<PointSample>& points, const CompoundRecognitionOptions& options,
                              const std::array<Vec3,3>& axes, CompoundCandidate& output, std::string& error);
} // namespace reconstruction::detail::compound
