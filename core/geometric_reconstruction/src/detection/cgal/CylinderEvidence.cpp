#include "detail/RansacEvidence.h"
#include <Eigen/Dense>

namespace reconstruction::detail::cgal {
double cylinderResidual(
    const CylinderEquation& cylinder,
    const modeling::Vec3& point,
    double* axial,
    modeling::Vec3* radialDirection) {
    const modeling::Vec3 offset = math3::subtract(
        point, cylinder.axisPoint);
    const double coordinate = math3::dot(offset, cylinder.axisDirection);
    const modeling::Vec3 radial = math3::subtract(
        offset, math3::scale(cylinder.axisDirection, coordinate));
    const double radialLength = math3::length(radial);
    if (axial != nullptr) {
        *axial = coordinate;
    }
    if (radialDirection != nullptr && radialLength > 1.0e-12) {
        *radialDirection = math3::scale(radial, 1.0 / radialLength);
    }
    return std::abs(radialLength - cylinder.radius);
}

bool makeCylinderEvidence(
    const InputRange& input,
    const CgalCylinder& shape,
    const PointStore& points,
    const CylinderDetectionOptions& options,
    CylinderEvidence& evidence) {
    const auto axis = shape.axis();
    modeling::Vec3 direction;
    modeling::Vec3 pointOnAxis;
    try {
        direction = math3::canonicalDirection(toVec3(axis.to_vector()));
        pointOnAxis = toVec3(axis.point(0));
    } catch (const std::invalid_argument&) {
        return false;
    }

    CylinderEquation cylinder;
    cylinder.axisDirection = direction;
    cylinder.axisPoint = math3::subtract(
        pointOnAxis,
        math3::scale(direction, math3::dot(pointOnAxis, direction)));
    cylinder.radius = CGAL::to_double(shape.radius());
    if (!std::isfinite(cylinder.radius) ||
        cylinder.radius < options.minimumRadius ||
        cylinder.radius > options.maximumRadius) {
        return false;
    }

    std::vector<std::size_t> support;
    support.reserve(shape.indices_of_assigned_points().size());
    for (const std::size_t index : shape.indices_of_assigned_points()) {
        modeling::Vec3 radialDirection{};
        const double residual = cylinderResidual(
            cylinder,
            toVec3(std::get<0>(input[index])),
            nullptr,
            &radialDirection);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        if (residual <= options.distanceThreshold &&
            std::abs(math3::dot(normal, radialDirection)) >=
                options.minimumNormalAlignment) {
            support.push_back(index);
        }
    }
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    auto refine = [&]() {
        // Normals span the transverse plane: its smallest eigenvector gives
        // the axis without a biased two-point RANSAC sample. Sign is immaterial.
        std::sort(support.begin(), support.end(),
                  [&](auto a, auto b) { return std::get<2>(input[a]) < std::get<2>(input[b]); });
        std::array<std::array<double, 3>, 3> covariance{};
        for (auto index : support) {
            const auto n = toVec3(std::get<1>(input[index]));
            const std::array<double, 3> v{{n.x, n.y, n.z}};
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    covariance[a][b] += v[a] * v[b];
        }
        const auto fit = math3::symmetricEigenDecomposition(covariance);
        if (fit.values[1] < support.size() * 0.01)
            return false; // unidentifiable line/very short arc
        const auto axis = fit.vectors[0];
        if (std::abs(math3::dot(axis, cylinder.axisDirection)) < 0.95)
            return false;
        const auto center = centroidOf(input, support);
        const auto u = math3::normalized(math3::cross(
            axis, std::abs(axis.z) < 0.9 ? modeling::Vec3{0, 0, 1} : modeling::Vec3{1, 0, 0}));
        const auto v = math3::cross(axis, u);
        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
        Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
        for (auto index : support) {
            const auto d = math3::subtract(toVec3(std::get<0>(input[index])), center);
            const double x = math3::dot(d, u), y = math3::dot(d, v);
            const Eigen::Vector3d row(x, y, 1);
            matrix += row * row.transpose();
            rhs -= row * (x * x + y * y);
        }
        const auto solver = matrix.fullPivLu();
        if (!solver.isInvertible())
            return false;
        const Eigen::Vector3d solution = solver.solve(rhs);
        const double r2 = (solution[0] * solution[0] + solution[1] * solution[1]) / 4 - solution[2];
        if (!solution.allFinite() || !(r2 > 0))
            return false;
        const auto point = math3::add(center, math3::add(math3::scale(u, -solution[0] / 2),
                                                         math3::scale(v, -solution[1] / 2)));
        cylinder.axisDirection = axis;
        cylinder.axisPoint = math3::subtract(point, math3::scale(axis, math3::dot(point, axis)));
        cylinder.radius = std::sqrt(r2);
        return true;
    };
    if (!refine())
        return false;
    // Re-establish complete support with the fitted geometry, then refine once
    // more. Assigned points alone vary with random sampling and other shapes.
    support.clear();
    for (std::size_t index = 0; index < input.size(); ++index) {
        modeling::Vec3 radial{};
        if (cylinderResidual(cylinder, toVec3(std::get<0>(input[index])), nullptr, &radial) <=
                options.distanceThreshold &&
            std::abs(math3::dot(toVec3(std::get<1>(input[index])), radial)) >=
                options.minimumNormalAlignment)
            support.push_back(index);
    }
    if (support.size() < options.minimumSupportPoints || !refine())
        return false;
    // Radius is the mean radial distance after the axis and centre fit.
    double radiusSum = 0.0;
    for (const std::size_t index : support) {
        const modeling::Vec3 offset =
            math3::subtract(toVec3(std::get<0>(input[index])), cylinder.axisPoint);
        const double axial = math3::dot(offset, cylinder.axisDirection);
        radiusSum +=
            math3::length(math3::subtract(offset, math3::scale(cylinder.axisDirection, axial)));
    }
    cylinder.radius = radiusSum / static_cast<double>(support.size());
    if (cylinder.radius < options.minimumRadius || cylinder.radius > options.maximumRadius) {
        return false;
    }

    modeling::Vec3 basisU;
    try {
        basisU = math3::normalized(math3::cross(
            cylinder.axisDirection,
            std::abs(cylinder.axisDirection.z) < 0.9
                ? modeling::Vec3{0.0, 0.0, 1.0}
                : modeling::Vec3{1.0, 0.0, 0.0}));
    } catch (const std::invalid_argument&) {
        return false;
    }
    const modeling::Vec3 basisV = math3::normalized(
        math3::cross(cylinder.axisDirection, basisU));

    evidence = {};
    evidence.cylinder = cylinder;
    evidence.axialMinimum = std::numeric_limits<double>::infinity();
    evidence.axialMaximum = -std::numeric_limits<double>::infinity();
    evidence.supportPointIds.reserve(support.size());
    std::vector<double> angles;
    angles.reserve(support.size());
    double residualSum = 0.0;
    double alignmentSum = 0.0;
    for (const std::size_t index : support) {
        double axial = 0.0;
        modeling::Vec3 radialDirection{};
        const double residual = cylinderResidual(
            cylinder,
            toVec3(std::get<0>(input[index])),
            &axial,
            &radialDirection);
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        evidence.axialMinimum = std::min(evidence.axialMinimum, axial);
        evidence.axialMaximum = std::max(evidence.axialMaximum, axial);
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
        residualSum += residual;
        alignmentSum += std::abs(math3::dot(normal, radialDirection));
        double angle = std::atan2(
            math3::dot(radialDirection, basisV),
            math3::dot(radialDirection, basisU));
        if (angle < 0.0) {
            angle += kTwoPi;
        }
        angles.push_back(angle);
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(support.size());
    evidence.angularCoverage = angularCoverage(std::move(angles));
    evidence.coverage = static_cast<double>(support.size()) /
        static_cast<double>(points.size());
    const double residualConfidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0);
    const double normalConfidence = alignmentSum /
        static_cast<double>(support.size());
    evidence.confidence = residualConfidence * normalConfidence *
        std::clamp(evidence.angularCoverage / 0.5, 0.0, 1.0);
    return true;
}
} // namespace reconstruction::detail::cgal
