#include "detail/RansacEvidence.h"

namespace reconstruction {
using namespace detail::cgal;

bool detectWithCgalEfficientRansac(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error) {
    output = {};
    error.clear();
    if (!options.detectPlanes && !options.detectCylinders &&
        !options.detectSpheres && !options.detectCones &&
        !options.detectTori) {
        error = "At least one CGAL primitive type must be enabled";
        return false;
    }
    if ((options.detectPlanes && !validPlaneOptions(options.plane)) ||
        (options.detectCylinders &&
         !validCylinderOptions(options.cylinder)) ||
        (options.detectSpheres && !validSphereOptions(options.sphere)) ||
        (options.detectCones && !validConeOptions(options.cone)) ||
        (options.detectTori && !validTorusOptions(options.torus))) {
        error = "CGAL Efficient RANSAC options are invalid";
        return false;
    }

    InputRange input;
    input.reserve(points.size());
    for (const PointSample& point : points.points()) {
        if (!point.normal.has_value()) {
            continue;
        }
        try {
            const modeling::Vec3 normal = math3::normalized(*point.normal);
            input.push_back({
                CgalPoint(point.position.x, point.position.y, point.position.z),
                CgalVector(normal.x, normal.y, normal.z),
                point.id,
            });
        } catch (const std::invalid_argument&) {
            // A zero or non-finite normal is not a valid CGAL observation.
        }
    }

    std::size_t minimumSupport = std::numeric_limits<std::size_t>::max();
    double probability = 1.0;
    double epsilon = 0.0;
    double normalThreshold = 1.0;
    double explicitCluster = 0.0;
    const auto contribute = [&](
                                bool enabled,
                                std::size_t support,
                                double shapeProbability,
                                double shapeEpsilon,
                                double shapeNormalThreshold,
                                double shapeCluster) {
        if (!enabled) {
            return;
        }
        minimumSupport = std::min(minimumSupport, support);
        probability = std::min(probability, shapeProbability);
        epsilon = std::max(epsilon, shapeEpsilon);
        normalThreshold = std::min(
            normalThreshold, shapeNormalThreshold);
        explicitCluster = std::max(explicitCluster, shapeCluster);
    };
    contribute(
        options.detectPlanes,
        options.plane.minimumSupportPoints,
        options.plane.probability,
        options.plane.distanceThreshold,
        options.plane.minimumNormalAlignment,
        options.plane.clusterEpsilon);
    contribute(
        options.detectCylinders,
        options.cylinder.minimumSupportPoints,
        options.cylinder.probability,
        options.cylinder.distanceThreshold,
        options.cylinder.minimumNormalAlignment,
        options.cylinder.clusterEpsilon);
    contribute(
        options.detectSpheres,
        options.sphere.minimumSupportPoints,
        options.sphere.probability,
        options.sphere.distanceThreshold,
        options.sphere.minimumNormalAlignment,
        options.sphere.clusterEpsilon);
    contribute(
        options.detectCones,
        options.cone.minimumSupportPoints,
        options.cone.probability,
        options.cone.distanceThreshold,
        options.cone.minimumNormalAlignment,
        options.cone.clusterEpsilon);
    contribute(
        options.detectTori,
        options.torus.minimumSupportPoints,
        options.torus.probability,
        options.torus.distanceThreshold,
        options.torus.minimumNormalAlignment,
        options.torus.clusterEpsilon);
    if (input.size() < minimumSupport) {
        error = "CGAL Efficient RANSAC requires more points with valid normals";
        return false;
    }

    const double diagonal = boundingBoxDiagonal(points);
    Ransac::Parameters parameters;
    parameters.probability = probability;
    parameters.min_points = minimumSupport;
    parameters.epsilon = epsilon;
    parameters.normal_threshold = normalThreshold;
    // Ten percent also connects deliberately sparse mechanical-surface grids.
    // Shape distance is still governed by epsilon, so this does not merge
    // parallel depth layers. Callers with dense scenes can override it.
    parameters.cluster_epsilon = explicitCluster > 0.0
        ? explicitCluster
        : std::max(parameters.epsilon * 3.0, diagonal * 0.1);

    Ransac ransac;
    ScopedRandomStream randomStream(options.randomSeed);
    ransac.set_input(input, InputPointMap{}, InputNormalMap{});
    if (options.detectPlanes) {
        ransac.add_shape_factory<CgalPlane>();
    }
    if (options.detectCylinders) {
        ransac.add_shape_factory<SafeCylinder>();
    }
    if (options.detectSpheres) {
        ransac.add_shape_factory<CgalSphere>();
    }
    if (options.detectCones) {
        ransac.add_shape_factory<CgalCone>();
    }
    if (options.detectTori) {
        ransac.add_shape_factory<CgalTorus>();
    }
    if (!ransac.detect(parameters)) {
        error = "CGAL Efficient RANSAC rejected the input or shape factories";
        return false;
    }

    std::size_t planeCount = 0U;
    std::size_t cylinderCount = 0U;
    std::size_t sphereCount = 0U;
    std::size_t coneCount = 0U;
    std::size_t torusCount = 0U;
    std::vector<PlaneEquation> acceptedPlanes;
    std::vector<modeling::Vec3> planeCentroids;
    std::vector<std::size_t> planeOutputIndices;
    std::unordered_map<PointId, const PointSample*> observations;
    for(const auto& p:points.points()) observations.emplace(p.id,&p);
    for (const std::shared_ptr<Ransac::Shape>& shape : ransac.shapes()) {
        if (const auto plane = std::dynamic_pointer_cast<CgalPlane>(shape)) {
            PlaneEvidence evidence;
            modeling::Vec3 centroid{};
            if (!makePlaneEvidence(
                    input, *plane, points, options.plane, evidence, centroid)) {
                continue;
            }
            bool duplicate = false;
            for (std::size_t index = 0; index < planeCount; ++index) {
                if (duplicatePlane(
                        evidence.plane,
                        centroid,
                        acceptedPlanes[index],
                        planeCentroids[index],
                        options.plane)) {
                    // One equation may describe several disconnected observed
                    // patches. Retain compatible support instead of dropping
                    // a distant coplanar face as a duplicate. The analysis layer
                    // subsequently splits these observations by connectivity.
                    auto& accepted=std::get<PlaneEvidence>(output.evidence[planeOutputIndices[index]]);
                    std::unordered_set<PointId> known(accepted.supportPointIds.begin(),accepted.supportPointIds.end());
                    for(auto id:evidence.supportPointIds) {
                        const auto* p=observations.at(id);
                        if(planeResidual(accepted.plane,p->position)<=options.plane.distanceThreshold && p->normal &&
                           std::abs(math3::dot(math3::normalized(*p->normal),accepted.plane.normal))>=options.plane.minimumNormalAlignment &&
                           known.insert(id).second) accepted.supportPointIds.push_back(id);
                    }
                    double residual=0;modeling::Vec3 center{};accepted.maxAbsoluteResidual=0;
                    for(auto id:accepted.supportPointIds) {
                        const auto* p=observations.at(id);
                        const double distance=planeResidual(accepted.plane,p->position);
                        residual+=distance;center=math3::add(center,p->position);
                        accepted.maxAbsoluteResidual=std::max(accepted.maxAbsoluteResidual,distance);
                    }
                    const double count=static_cast<double>(accepted.supportPointIds.size());
                    accepted.meanAbsoluteResidual=residual/count;
                    accepted.coverage=count/points.size();
                    accepted.confidence=std::clamp(1-accepted.meanAbsoluteResidual/options.plane.distanceThreshold,0.0,1.0);
                    planeCentroids[index]=math3::scale(center,1/count);
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate && planeCount < options.plane.maximumPlanes) {
                planeOutputIndices.push_back(output.evidence.size());
                output.evidence.emplace_back(std::move(evidence));
                acceptedPlanes.push_back(
                    std::get<PlaneEvidence>(output.evidence.back()).plane);
                planeCentroids.push_back(centroid);
                ++planeCount;
            }
            continue;
        }

        if (const auto cylinder =
                std::dynamic_pointer_cast<CgalCylinder>(shape)) {
            if (cylinderCount >= options.cylinder.maximumCylinders) {
                continue;
            }
            CylinderEvidence evidence;
            if (makeCylinderEvidence(
                    input,
                    *cylinder,
                    points,
                    options.cylinder,
                    evidence)) {
                bool duplicate = false;
                for (auto& accepted : output.evidence) {
                    auto* previous = std::get_if<CylinderEvidence>(&accepted);
                    if (!previous || !duplicateCylinder(*previous, evidence, options.cylinder)) {
                        continue;
                    }
                    if (evidence.supportPointIds.size() > previous->supportPointIds.size() ||
                        (evidence.supportPointIds.size() == previous->supportPointIds.size() &&
                         evidence.confidence > previous->confidence)) {
                        *previous = std::move(evidence);
                    }
                    duplicate = true;
                    break;
                }
                if (duplicate) {
                    continue;
                }
                output.evidence.emplace_back(std::move(evidence));
                ++cylinderCount;
            }
            continue;
        }

        if (const auto sphere = std::dynamic_pointer_cast<CgalSphere>(shape)) {
            if (sphereCount >= options.sphere.maximumSpheres) {
                continue;
            }
            SphereEvidence evidence;
            if (makeSphereEvidence(
                    input, *sphere, points, options.sphere, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++sphereCount;
            }
            continue;
        }

        if (const auto cone = std::dynamic_pointer_cast<CgalCone>(shape)) {
            if (coneCount >= options.cone.maximumCones) {
                continue;
            }
            ConeEvidence evidence;
            if (makeConeEvidence(
                    input, *cone, points, options.cone, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++coneCount;
            }
            continue;
        }

        if (const auto torus = std::dynamic_pointer_cast<CgalTorus>(shape)) {
            if (torusCount >= options.torus.maximumTori) {
                continue;
            }
            TorusEvidence evidence;
            if (makeTorusEvidence(
                    input, *torus, points, options.torus, evidence)) {
                output.evidence.emplace_back(std::move(evidence));
                ++torusCount;
            }
        }
    }

    SurfaceId nextId = 1;
    std::unordered_set<PointId> assigned;
    for (PrimitiveEvidence& evidence : output.evidence) {
        assignId(evidence, nextId++);
        addSupportIds(evidence, assigned);
    }
    for (const PointSample& point : points.points()) {
        if (assigned.find(point.id) == assigned.end()) {
            output.unassignedPointIds.push_back(point.id);
        }
    }

    if (output.evidence.empty()) {
        error = "CGAL Efficient RANSAC found no supported primitive";
        return false;
    }
    return true;
}
} // namespace reconstruction
