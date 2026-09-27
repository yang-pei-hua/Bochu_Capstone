#include "reconstruction/PlaneDetector.h"

#include "Math3.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace reconstruction {
namespace {

using modeling::Vec3;

PlaneEquation planeThroughThreePoints(
    const Vec3& first,
    const Vec3& second,
    const Vec3& third) {
    Vec3 normal = math3::cross(
        math3::subtract(second, first), math3::subtract(third, first));
    normal = math3::canonicalDirection(normal);
    return {normal, -math3::dot(normal, first)};
}

PlaneEquation fitPlane(
    const std::vector<PointSample>& points,
    const std::vector<std::size_t>& indices) {
    Vec3 centroid{};
    for (const std::size_t index : indices) {
        centroid = math3::add(centroid, points[index].position);
    }
    centroid = math3::scale(
        centroid, 1.0 / static_cast<double>(indices.size()));

    std::array<std::array<double, 3>, 3> covariance{};
    for (const std::size_t index : indices) {
        const Vec3 delta = math3::subtract(points[index].position, centroid);
        const std::array<double, 3> values{delta.x, delta.y, delta.z};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                covariance[row][column] += values[row] * values[column];
            }
        }
    }
    const Vec3 normal = math3::smallestEigenvector(covariance);
    return {normal, -math3::dot(normal, centroid)};
}

double residual(const PlaneEquation& plane, const Vec3& point) {
    return std::abs(math3::dot(plane.normal, point) + plane.offset);
}

}  // namespace

bool detectPlanes(
    const PointStore& pointStore,
    const PlaneDetectionOptions& options,
    std::vector<PlaneEvidence>& output,
    std::string& error) {
    output.clear();
    error.clear();
    if (pointStore.size() < 3U) {
        error = "Plane detection requires at least three points";
        return false;
    }
    if (!std::isfinite(options.distanceThreshold) ||
        options.distanceThreshold <= 0.0 ||
        options.minimumSupportPoints < 3U || options.maximumPlanes == 0U ||
        options.ransacIterations == 0U) {
        error = "Plane detection options are invalid";
        return false;
    }

    const std::vector<PointSample>& points = pointStore.points();
    std::vector<std::size_t> remaining(points.size());
    for (std::size_t index = 0; index < remaining.size(); ++index) {
        remaining[index] = index;
    }
    std::mt19937 generator(options.randomSeed);

    while (output.size() < options.maximumPlanes &&
           remaining.size() >= options.minimumSupportPoints) {
        std::vector<std::size_t> bestInliers;
        double bestResidual = std::numeric_limits<double>::infinity();
        std::uniform_int_distribution<std::size_t> distribution(
            0U, remaining.size() - 1U);

        for (std::size_t iteration = 0;
             iteration < options.ransacIterations;
             ++iteration) {
            std::size_t first = distribution(generator);
            std::size_t second = distribution(generator);
            std::size_t third = distribution(generator);
            for (int retry = 0;
                 retry < 8 &&
                 (first == second || first == third || second == third);
                 ++retry) {
                second = distribution(generator);
                third = distribution(generator);
            }
            if (first == second || first == third || second == third) {
                continue;
            }

            PlaneEquation plane;
            try {
                plane = planeThroughThreePoints(
                    points[remaining[first]].position,
                    points[remaining[second]].position,
                    points[remaining[third]].position);
            } catch (const std::invalid_argument&) {
                continue;
            }

            std::vector<std::size_t> inliers;
            double residualSum = 0.0;
            for (const std::size_t index : remaining) {
                const double distance = residual(plane, points[index].position);
                if (distance <= options.distanceThreshold) {
                    inliers.push_back(index);
                    residualSum += distance;
                }
            }
            if (inliers.size() > bestInliers.size() ||
                (inliers.size() == bestInliers.size() &&
                 residualSum < bestResidual)) {
                bestInliers = std::move(inliers);
                bestResidual = residualSum;
            }
        }

        if (bestInliers.size() < options.minimumSupportPoints) {
            break;
        }

        PlaneEquation refined = fitPlane(points, bestInliers);
        std::vector<std::size_t> finalInliers;
        for (const std::size_t index : remaining) {
            if (residual(refined, points[index].position) <=
                options.distanceThreshold) {
                finalInliers.push_back(index);
            }
        }
        if (finalInliers.size() < options.minimumSupportPoints) {
            break;
        }
        refined = fitPlane(points, finalInliers);

        PlaneEvidence evidence;
        evidence.id = static_cast<SurfaceId>(output.size() + 1U);
        evidence.plane = refined;
        evidence.supportPointIds.reserve(finalInliers.size());
        double residualSum = 0.0;
        for (const std::size_t index : finalInliers) {
            const double distance = residual(refined, points[index].position);
            evidence.supportPointIds.push_back(points[index].id);
            residualSum += distance;
            evidence.maxAbsoluteResidual =
                std::max(evidence.maxAbsoluteResidual, distance);
        }
        evidence.meanAbsoluteResidual = residualSum /
            static_cast<double>(finalInliers.size());
        evidence.coverage = static_cast<double>(finalInliers.size()) /
            static_cast<double>(points.size());
        evidence.confidence = std::clamp(
            1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
            0.0,
            1.0);
        output.push_back(std::move(evidence));

        std::vector<bool> consumed(points.size(), false);
        for (const std::size_t index : finalInliers) {
            consumed[index] = true;
        }
        remaining.erase(
            std::remove_if(
                remaining.begin(),
                remaining.end(),
                [&consumed](std::size_t index) { return consumed[index]; }),
            remaining.end());
    }

    if (output.empty()) {
        error = "RANSAC did not find a plane with sufficient support";
        return false;
    }
    return true;
}

}  // namespace reconstruction
