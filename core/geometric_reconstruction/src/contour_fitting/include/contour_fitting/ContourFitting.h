#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace contour_fitting {

struct Point2D {
    double x = 0;
    double y = 0;
};

// Closed implicitly, with no repeated terminal point.
using Ring = std::vector<Point2D>;

struct Contour {
    Ring outer;               // Counterclockwise.
    std::vector<Ring> holes;  // Clockwise.
};

struct Options {
    double alphaRadius = 0; // Zero estimates a scale from the samples.
    std::optional<double> simplificationTolerance; // Zero keeps the original boundary vertices.
    std::size_t minimumRegionPoints = 0;
    double minimumRegionFraction = 0; // Fraction of all distinct projected samples, in [0, 1].
    double automaticAlphaScale = 1;
};

struct Result {
    std::size_t uniquePointCount = 0;
    double alphaRadius = 0;
    double simplificationTolerance = 0;
    std::vector<Contour> contours;
    std::size_t discardedRegionCount = 0;
};

// Fits regions supported by unordered samples in a common 2D coordinate system.
// Duplicate points are merged. Degenerate input yields no contours.
// Non-finite coordinates or invalid options throw std::invalid_argument.
Result fitContours(const std::vector<Point2D>& points, const Options& options = {});

} // namespace contour_fitting
