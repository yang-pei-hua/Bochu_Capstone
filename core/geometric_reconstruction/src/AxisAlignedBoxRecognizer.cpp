#include "reconstruction/AxisAlignedBoxRecognizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace reconstruction {
namespace {

double coordinate(const modeling::Vec3& point, int axis) {
    if (axis == 0) {
        return point.x;
    }
    if (axis == 1) {
        return point.y;
    }
    return point.z;
}

}  // namespace

bool recognizeAxisAlignedBox(
    const PointStore& points,
    double distanceTolerance,
    BoxCandidate& output,
    std::string& error) {
    output = {};
    error.clear();
    if (points.empty()) {
        error = "Cannot recognize a box from an empty point store";
        return false;
    }
    if (points.unit() != LengthUnit::Millimeter) {
        error = "Axis-aligned box recognition requires an explicit millimeter scale";
        return false;
    }
    if (!std::isfinite(distanceTolerance) || distanceTolerance <= 0.0) {
        error = "Box recognition distance tolerance must be finite and positive";
        return false;
    }

    std::array<double, 3> minimum{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    };
    std::array<double, 3> maximum{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    for (const PointSample& point : points.points()) {
        for (int axis = 0; axis < 3; ++axis) {
            const double value = coordinate(point.position, axis);
            minimum[axis] = std::min(minimum[axis], value);
            maximum[axis] = std::max(maximum[axis], value);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (maximum[axis] - minimum[axis] <= distanceTolerance) {
            error = "Point store is degenerate and cannot define a box volume";
            return false;
        }
    }

    output.primitive.pose.origin =
        modeling::Vec3{minimum[0], minimum[1], minimum[2]};
    output.primitive.sizeX = maximum[0] - minimum[0];
    output.primitive.sizeY = maximum[1] - minimum[1];
    output.primitive.sizeZ = maximum[2] - minimum[2];

    struct Boundary {
        int axis;
        double coordinate;
        modeling::Vec3 normal;
        double offset;
    };
    const std::array<Boundary, 6> boundaries{{
        {0, minimum[0], {-1.0, 0.0, 0.0}, minimum[0]},
        {0, maximum[0], {1.0, 0.0, 0.0}, -maximum[0]},
        {1, minimum[1], {0.0, -1.0, 0.0}, minimum[1]},
        {1, maximum[1], {0.0, 1.0, 0.0}, -maximum[1]},
        {2, minimum[2], {0.0, 0.0, -1.0}, minimum[2]},
        {2, maximum[2], {0.0, 0.0, 1.0}, -maximum[2]},
    }};

    output.sourceSurfaces.reserve(boundaries.size());
    std::size_t explainedPoints = 0;
    for (std::size_t boundaryIndex = 0;
         boundaryIndex < boundaries.size();
         ++boundaryIndex) {
        const Boundary& boundary = boundaries[boundaryIndex];
        PlaneEvidence evidence;
        evidence.id = static_cast<SurfaceId>(boundaryIndex + 1U);
        evidence.plane = {boundary.normal, boundary.offset};
        double residualSum = 0.0;
        for (const PointSample& point : points.points()) {
            const double residual = std::abs(
                coordinate(point.position, boundary.axis) - boundary.coordinate);
            if (residual <= distanceTolerance) {
                evidence.supportPointIds.push_back(point.id);
                residualSum += residual;
                evidence.maxAbsoluteResidual =
                    std::max(evidence.maxAbsoluteResidual, residual);
            }
        }
        if (evidence.supportPointIds.empty()) {
            error = "A box boundary plane has no supporting points";
            output = {};
            return false;
        }
        evidence.meanAbsoluteResidual = residualSum /
            static_cast<double>(evidence.supportPointIds.size());
        evidence.coverage = static_cast<double>(evidence.supportPointIds.size()) /
            static_cast<double>(points.size());
        evidence.confidence = std::clamp(
            1.0 - evidence.meanAbsoluteResidual / distanceTolerance, 0.0, 1.0);
        output.sourceSurfaces.push_back(std::move(evidence));
    }

    for (const PointSample& point : points.points()) {
        bool explained = false;
        for (const Boundary& boundary : boundaries) {
            if (std::abs(coordinate(point.position, boundary.axis) -
                         boundary.coordinate) <= distanceTolerance) {
                explained = true;
                break;
            }
        }
        if (explained) {
            ++explainedPoints;
        }
    }
    output.confidence = static_cast<double>(explainedPoints) /
        static_cast<double>(points.size());
    return true;
}

}  // namespace reconstruction
