#pragma once

#include "modeling/Geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace reconstruction::math3 {

using modeling::Vec3;

inline Vec3 add(const Vec3& left, const Vec3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 subtract(const Vec3& left, const Vec3& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 scale(const Vec3& value, double factor) {
    return {value.x * factor, value.y * factor, value.z * factor};
}

inline double dot(const Vec3& left, const Vec3& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline Vec3 cross(const Vec3& left, const Vec3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

inline double length(const Vec3& value) {
    return std::sqrt(dot(value, value));
}

inline Vec3 normalized(const Vec3& value) {
    const double magnitude = length(value);
    if (!std::isfinite(magnitude) || magnitude <= 1.0e-12) {
        throw std::invalid_argument("Cannot normalize a degenerate vector");
    }
    return scale(value, 1.0 / magnitude);
}

inline Vec3 canonicalDirection(Vec3 value) {
    value = normalized(value);
    const std::array<double, 3> absolute{
        std::abs(value.x), std::abs(value.y), std::abs(value.z)};
    const auto dominant = std::max_element(absolute.begin(), absolute.end());
    const std::size_t axis = static_cast<std::size_t>(
        std::distance(absolute.begin(), dominant));
    const double component = axis == 0U ? value.x : (axis == 1U ? value.y : value.z);
    return component < 0.0 ? scale(value, -1.0) : value;
}

inline Vec3 smallestEigenvector(std::array<std::array<double, 3>, 3> matrix) {
    std::array<std::array<double, 3>, 3> vectors{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}},
    }};

    for (int iteration = 0; iteration < 32; ++iteration) {
        int p = 0;
        int q = 1;
        double largest = std::abs(matrix[0][1]);
        for (int row = 0; row < 3; ++row) {
            for (int column = row + 1; column < 3; ++column) {
                const double candidate = std::abs(matrix[row][column]);
                if (candidate > largest) {
                    largest = candidate;
                    p = row;
                    q = column;
                }
            }
        }
        if (largest <= 1.0e-15) {
            break;
        }

        const double app = matrix[p][p];
        const double aqq = matrix[q][q];
        const double apq = matrix[p][q];
        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);

        for (int index = 0; index < 3; ++index) {
            if (index == p || index == q) {
                continue;
            }
            const double aip = matrix[index][p];
            const double aiq = matrix[index][q];
            matrix[index][p] = matrix[p][index] = cosine * aip - sine * aiq;
            matrix[index][q] = matrix[q][index] = sine * aip + cosine * aiq;
        }
        matrix[p][p] = cosine * cosine * app -
            2.0 * sine * cosine * apq + sine * sine * aqq;
        matrix[q][q] = sine * sine * app +
            2.0 * sine * cosine * apq + cosine * cosine * aqq;
        matrix[p][q] = matrix[q][p] = 0.0;

        for (int row = 0; row < 3; ++row) {
            const double vip = vectors[row][p];
            const double viq = vectors[row][q];
            vectors[row][p] = cosine * vip - sine * viq;
            vectors[row][q] = sine * vip + cosine * viq;
        }
    }

    int smallest = 0;
    if (matrix[1][1] < matrix[smallest][smallest]) {
        smallest = 1;
    }
    if (matrix[2][2] < matrix[smallest][smallest]) {
        smallest = 2;
    }
    return canonicalDirection(
        {vectors[0][smallest], vectors[1][smallest], vectors[2][smallest]});
}

}  // namespace reconstruction::math3
