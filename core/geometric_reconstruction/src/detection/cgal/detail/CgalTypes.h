#pragma once
#include "reconstruction/detection/CgalEfficientRansacDetector.h"

#include "common/Math3.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cone.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cylinder.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Efficient_RANSAC_traits.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Plane.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Sphere.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Torus.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <CGAL/Random.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace reconstruction::detail::cgal {
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint = Kernel::Point_3;
using CgalVector = Kernel::Vector_3;

using InputPoint = std::tuple<CgalPoint, CgalVector, PointId>;
using InputRange = std::vector<InputPoint>;
using InputPointMap = CGAL::Nth_of_tuple_property_map<0, InputPoint>;
using InputNormalMap = CGAL::Nth_of_tuple_property_map<1, InputPoint>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<
    Kernel, InputRange, InputPointMap, InputNormalMap>;
using Ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using CgalPlane = CGAL::Shape_detection::Plane<Traits>;
using CgalCylinder = CGAL::Shape_detection::Cylinder<Traits>;
using CgalSphere = CGAL::Shape_detection::Sphere<Traits>;
using CgalCone = CGAL::Shape_detection::Cone<Traits>;
using CgalTorus = CGAL::Shape_detection::Torus<Traits>;

// A pair of perpendicular normals on a box edge can produce a zero-radius
// cylinder which CGAL still marks valid. Its angular parameterization then
// divides by zero and indexes the connectivity bitmap with non-finite values.
// Reject degenerate candidates before scoring; use CGAL's spatial connectivity
// for cylinders so axial points and the angular seam cannot corrupt the bitmap.
class SafeCylinder final : public CgalCylinder {
  protected:
    void create_shape(const std::vector<std::size_t> &indices) override {
        CgalCylinder::create_shape(indices);
        if (!this->m_is_valid)
            return;
        const double radius = CGAL::to_double(this->radius());
        if (!std::isfinite(radius) || radius <= 1e-12) {
            this->m_is_valid = false;
            return;
        }
        const auto axis = this->axis();
        const auto p = axis.point(0);
        const auto v = axis.to_vector();
        if (!std::isfinite(CGAL::to_double(p.x())) || !std::isfinite(CGAL::to_double(p.y())) ||
            !std::isfinite(CGAL::to_double(p.z())) ||
            !std::isfinite(CGAL::to_double(v.squared_length())) ||
            CGAL::to_double(v.squared_length()) <= 1e-24)
            this->m_is_valid = false;
    }
    std::size_t connected_component(std::vector<std::size_t> &indices,
                                    Traits::FT epsilon) override {
        if (indices.empty())
            return 0;
        return this->connected_component_kdTree(indices, epsilon);
    }
};

class ScopedRandomStream {
    CGAL::Random::State previous;

  public:
    explicit ScopedRandomStream(std::uint32_t seed) {
        auto &stream = CGAL::get_default_random();
        stream.save_state(previous);
        stream = CGAL::Random(seed);
    }
    ~ScopedRandomStream() { CGAL::get_default_random().restore_state(previous); }
    ScopedRandomStream(const ScopedRandomStream &) = delete;
    ScopedRandomStream &operator=(const ScopedRandomStream &) = delete;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
} // namespace reconstruction::detail::cgal
