#include "contour_fitting/ContourFitting.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace contour_fitting;

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

double area(const Ring& ring)
{
    double sum = 0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto& a = ring[i];
        const auto& b = ring[(i + 1) % ring.size()];
        sum += a.x*b.y - a.y*b.x;
    }
    return sum / 2;
}

bool contains(const Ring& ring, Point2D point)
{
    bool inside = false;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto& a = ring[i];
        const auto& b = ring[(i + 1) % ring.size()];
        if ((a.y > point.y) != (b.y > point.y)
            && point.x < a.x + (point.y - a.y)*(b.x - a.x)/(b.y - a.y)) inside = !inside;
    }
    return inside;
}

void fitsConcaveShape()
{
    std::vector<Point2D> points;
    for (int x = 0; x <= 30; ++x) for (int y = 0; y <= 28; ++y) {
        if (x <= 5 || x >= 25 || y <= 5) points.push_back({double(x), double(y)});
    }
    const auto result = fitContours(points);
    require(result.contours.size() == 1 && result.contours.front().holes.empty(), "expected one U-shaped contour");
    const auto& outer = result.contours.front().outer;
    require(area(outer) > 0 && contains(outer, {2,20}) && contains(outer, {15,2})
            && !contains(outer, {15,20}), "contour must leave the open notch empty");

    std::reverse(points.begin(), points.end());
    points.push_back(points.front());
    const auto repeated = fitContours(points);
    require(repeated.uniquePointCount == result.uniquePointCount && repeated.contours.size() == 1,
            "point order and exact duplicates must not affect fitting");
    const auto& repeatedOuter = repeated.contours.front().outer;
    require(outer.size() == repeatedOuter.size() &&
            std::equal(outer.begin(), outer.end(), repeatedOuter.begin(), [](auto a, auto b) {
                return a.x == b.x && a.y == b.y;
            }), "the output ring must be deterministic and ordered");
}

void fitsHoleAndIslands()
{
    std::vector<Point2D> points;
    for (int x = 0; x <= 30; ++x) for (int y = 0; y <= 30; ++y) {
        const bool opening = x > 6 && x < 24 && y > 6 && y < 24;
        const bool island = x >= 13 && x <= 17 && y >= 13 && y <= 17;
        if (!opening || island) points.push_back({double(x), double(y)});
        if (x <= 5 && y <= 5) points.push_back({double(x+45), double(y)});
    }
    const auto result = fitContours(points, {1.5, .4});
    require(result.contours.size() == 3 && result.contours.front().holes.size() == 1,
            "outer shell, interior island and separated island must remain distinct");
    const auto& shell = result.contours.front();
    require(area(shell.outer) > 0 && area(shell.holes.front()) < 0
            && contains(shell.holes.front(), {10,10}) && contains(shell.holes.front(), {15,15}),
            "the hole must remain attached to its containing outer ring with clockwise winding");
    require(std::count_if(result.contours.begin(), result.contours.end(), [](const auto& contour) {
        return contains(contour.outer, {15,15});
    }) == 2, "the island inside the hole must have its own outer ring");

    const auto absolute = fitContours(points, {1.5, .4, 30, 0});
    require(absolute.contours.size() == 2 && absolute.discardedRegionCount == 1,
            "minimum point count rejects the 25-point island and keeps the 36-point island");
    points.insert(points.end(),25,{15,15});
    const auto relative = fitContours(points, {1.5, .4, 30, .05});
    require(relative.contours.size() == 1 && relative.discardedRegionCount == 2
            && relative.contours.front().holes.size() == 1,
            "the fraction uses distinct samples and suppresses outer regions without filling holes");
}

void handlesUnusableSamples()
{
    require(fitContours({{0,0}, {1,1}, {2,2}}).contours.empty(), "collinear samples must not create area");
    require(fitContours({{0,0}, {1,0}, {0,1}}, {.1, 0}).contours.empty(),
            "radius too small to join samples must yield no contour");
    bool rejected = false;
    try {
        fitContours({{std::numeric_limits<double>::quiet_NaN(),0}});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "invalid coordinates must be reported");
}
}

int main()
{
    try {
        fitsConcaveShape();
        fitsHoleAndIslands();
        handlesUnusableSamples();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
