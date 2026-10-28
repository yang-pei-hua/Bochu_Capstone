#pragma once
#include "CgalTypes.h"

namespace reconstruction::detail::cgal {
bool validPlaneOptions(const PlaneDetectionOptions& options);

bool validCylinderOptions(const CylinderDetectionOptions& options);

bool validSphereOptions(const SphereDetectionOptions& options);

bool validConeOptions(const ConeDetectionOptions& options);

bool validTorusOptions(const TorusDetectionOptions& options);

bool duplicateCylinder(const CylinderEvidence& left, const CylinderEvidence& right,
                       const CylinderDetectionOptions& options);

modeling::Vec3 toVec3(const CgalPoint &point);

modeling::Vec3 toVec3(const CgalVector& vector);

double boundingBoxDiagonal(const PointStore& points);

double planeResidual(
    const PlaneEquation& plane,
    const modeling::Vec3& point);

double angularCoverage(std::vector<double> angles);

void assignId(PrimitiveEvidence& evidence, SurfaceId id);

void addSupportIds(
    const PrimitiveEvidence& evidence,
    std::unordered_set<PointId>& assigned);

PlaneEquation fitPlane(
    const InputRange& input,
    const std::vector<std::size_t>& indices);

modeling::Vec3 centroidOf(
    const InputRange& input,
    const std::vector<std::size_t>& indices);

bool duplicatePlane(
    const PlaneEquation& candidate,
    const modeling::Vec3& candidateCentroid,
    const PlaneEquation& accepted,
    const modeling::Vec3& acceptedCentroid,
    const PlaneDetectionOptions& options);

std::vector<std::size_t> filteredPlaneSupport(
    const InputRange& input,
    const std::vector<std::size_t>& candidateIndices,
    const PlaneEquation& plane,
    const PlaneDetectionOptions& options);

bool makePlaneEvidence(
    const InputRange& input,
    const CgalPlane& shape,
    const PointStore& points,
    const PlaneDetectionOptions& options,
    PlaneEvidence& evidence,
    modeling::Vec3& centroid);

double cylinderResidual(
    const CylinderEquation& cylinder,
    const modeling::Vec3& point,
    double* axial = nullptr,
    modeling::Vec3* radialDirection = nullptr);

bool makeCylinderEvidence(
    const InputRange& input,
    const CgalCylinder& shape,
    const PointStore& points,
    const CylinderDetectionOptions& options,
    CylinderEvidence& evidence);

bool makeSphereEvidence(
    const InputRange& input,
    const CgalSphere& shape,
    const PointStore& points,
    const SphereDetectionOptions& options,
    SphereEvidence& evidence);

bool makeConeEvidence(
    const InputRange& input,
    const CgalCone& shape,
    const PointStore& points,
    const ConeDetectionOptions& options,
    ConeEvidence& evidence);

bool makeTorusEvidence(
    const InputRange& input,
    const CgalTorus& shape,
    const PointStore& points,
    const TorusDetectionOptions& options,
    TorusEvidence& evidence);
} // namespace reconstruction::detail::cgal
