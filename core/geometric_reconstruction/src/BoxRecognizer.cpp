#include "reconstruction/BoxRecognizer.h"

#include "Math3.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace reconstruction {
namespace {

using Pair = std::array<std::size_t, 2>;
using Pairing = std::array<Pair, 3>;

std::vector<Pairing> allPairings() {
    std::vector<Pairing> result;
    for (std::size_t partner = 1; partner < 6; ++partner) {
        std::array<std::size_t, 4> remaining{};
        std::size_t cursor = 0;
        for (std::size_t index = 1; index < 6; ++index) {
            if (index != partner) {
                remaining[cursor++] = index;
            }
        }
        for (std::size_t secondPartner = 1;
             secondPartner < remaining.size();
             ++secondPartner) {
            std::array<std::size_t, 2> last{};
            std::size_t lastCursor = 0;
            for (std::size_t index = 1; index < remaining.size(); ++index) {
                if (index != secondPartner) {
                    last[lastCursor++] = remaining[index];
                }
            }
            result.push_back(Pairing{{
                Pair{{0U, partner}},
                Pair{{remaining[0], remaining[secondPartner]}},
                Pair{{last[0], last[1]}},
            }});
        }
    }
    return result;
}

modeling::Vec3 pairDirection(
    const PlaneEvidence& first,
    const PlaneEvidence& second) {
    modeling::Vec3 secondNormal = second.plane.normal;
    if (math3::dot(first.plane.normal, secondNormal) < 0.0) {
        secondNormal = math3::scale(secondNormal, -1.0);
    }
    return math3::canonicalDirection(
        math3::add(first.plane.normal, secondNormal));
}

double planeCoordinate(
    const PlaneEvidence& plane,
    const modeling::Vec3& axis) {
    const double projection = math3::dot(plane.plane.normal, axis);
    return -plane.plane.offset / projection;
}

std::array<double, 2> pairExtents(
    const Pair& pair,
    const std::vector<PlaneEvidence>& planes,
    const modeling::Vec3& axis) {
    const double first = planeCoordinate(planes[pair[0]], axis);
    const double second = planeCoordinate(planes[pair[1]], axis);
    return {std::min(first, second), std::max(first, second)};
}

}  // namespace

bool recognizeBoxFromPlanes(
    const PointStore& points,
    const std::vector<PlaneEvidence>& planes,
    const BoxRecognitionOptions& options,
    BoxCandidate& output,
    std::string& error) {
    output = {};
    error.clear();
    if (planes.size() != 6U) {
        error = "Box recognition currently requires exactly six plane candidates";
        return false;
    }
    if (!std::isfinite(options.angularToleranceRadians) ||
        options.angularToleranceRadians <= 0.0 ||
        options.angularToleranceRadians >= 0.7853981633974483) {
        error = "Box angular tolerance must be between zero and 45 degrees";
        return false;
    }

    const double minimumParallelDot =
        std::cos(options.angularToleranceRadians);
    const double maximumPerpendicularDot =
        std::sin(options.angularToleranceRadians);
    double bestScore = std::numeric_limits<double>::infinity();
    Pairing bestPairing{};
    std::array<modeling::Vec3, 3> bestDirections{};
    bool found = false;

    for (const Pairing& pairing : allPairings()) {
        std::array<modeling::Vec3, 3> directions{};
        double score = 0.0;
        bool valid = true;
        for (std::size_t pairIndex = 0; pairIndex < 3; ++pairIndex) {
            const Pair& pair = pairing[pairIndex];
            const double parallel = std::abs(math3::dot(
                planes[pair[0]].plane.normal,
                planes[pair[1]].plane.normal));
            if (parallel < minimumParallelDot) {
                valid = false;
                break;
            }
            directions[pairIndex] =
                pairDirection(planes[pair[0]], planes[pair[1]]);
            score += 1.0 - parallel;
        }
        if (!valid) {
            continue;
        }
        for (std::size_t first = 0; first < 3; ++first) {
            for (std::size_t second = first + 1; second < 3; ++second) {
                const double perpendicular = std::abs(
                    math3::dot(directions[first], directions[second]));
                if (perpendicular > maximumPerpendicularDot) {
                    valid = false;
                }
                score += perpendicular;
            }
        }
        if (valid && score < bestScore) {
            found = true;
            bestScore = score;
            bestPairing = pairing;
            bestDirections = directions;
        }
    }

    if (!found) {
        error = "Plane candidates do not form three orthogonal parallel pairs";
        return false;
    }

    const modeling::Vec3 xAxis = bestDirections[0];
    modeling::Vec3 yAxis = math3::normalized(math3::subtract(
        bestDirections[1],
        math3::scale(xAxis, math3::dot(bestDirections[1], xAxis))));
    modeling::Vec3 zAxis = math3::normalized(math3::cross(xAxis, yAxis));
    if (math3::dot(zAxis, bestDirections[2]) < 0.0) {
        zAxis = math3::scale(zAxis, -1.0);
    }
    yAxis = math3::normalized(math3::cross(zAxis, xAxis));

    const std::array<double, 2> xExtents =
        pairExtents(bestPairing[0], planes, xAxis);
    const std::array<double, 2> yExtents =
        pairExtents(bestPairing[1], planes, yAxis);
    const std::array<double, 2> zExtents =
        pairExtents(bestPairing[2], planes, zAxis);
    const double sizeX = xExtents[1] - xExtents[0];
    const double sizeY = yExtents[1] - yExtents[0];
    const double sizeZ = zExtents[1] - zExtents[0];
    if (sizeX <= 0.0 || sizeY <= 0.0 || sizeZ <= 0.0) {
        error = "Paired planes produced a degenerate box";
        return false;
    }

    output.primitive.pose.origin = math3::add(
        math3::add(
            math3::scale(xAxis, xExtents[0]),
            math3::scale(yAxis, yExtents[0])),
        math3::scale(zAxis, zExtents[0]));
    output.primitive.pose.xDirection = xAxis;
    output.primitive.pose.zDirection = zAxis;
    output.primitive.sizeX = sizeX;
    output.primitive.sizeY = sizeY;
    output.primitive.sizeZ = sizeZ;
    output.sourceSurfaces = planes;

    std::unordered_set<PointId> explained;
    double evidenceConfidence = 0.0;
    for (const PlaneEvidence& plane : planes) {
        evidenceConfidence += plane.confidence;
        explained.insert(
            plane.supportPointIds.begin(), plane.supportPointIds.end());
    }
    evidenceConfidence /= static_cast<double>(planes.size());
    const double explainedFraction = points.empty()
        ? 0.0
        : static_cast<double>(explained.size()) /
            static_cast<double>(points.size());
    const double relationConfidence =
        std::clamp(1.0 - bestScore / 6.0, 0.0, 1.0);
    output.confidence =
        evidenceConfidence * explainedFraction * relationConfidence;
    return true;
}

bool reconstructBox(
    const PointStore& points,
    const PlaneDetectionOptions& planeOptions,
    const BoxRecognitionOptions& boxOptions,
    BoxCandidate& output,
    std::string& error) {
    std::vector<PlaneEvidence> planes;
    if (!detectPlanes(points, planeOptions, planes, error)) {
        return false;
    }
    return recognizeBoxFromPlanes(points, planes, boxOptions, output, error);
}

}  // namespace reconstruction
