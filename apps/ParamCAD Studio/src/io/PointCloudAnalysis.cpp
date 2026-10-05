#include "io/PointCloudAnalysis.h"
#include "io/PointCloudAdapter.h"

#include <QJsonArray>
#include <QThread>
#include <reconstruction/PrimitiveDetector.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace {
using Vec = modeling::Vec3;
Vec minus(const Vec& a, const Vec& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
double dot(const Vec& a, const Vec& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double length(const Vec& a) { return std::sqrt(dot(a,a)); }
QJsonArray array(const Vec& p) { return {p.x, p.y, p.z}; }
double residual(const reconstruction::PrimitiveEvidence& evidence, const Vec& p)
{
    return std::visit([&](const auto& surface) {
        using T = std::decay_t<decltype(surface)>;
        if constexpr (std::is_same_v<T, reconstruction::PlaneEvidence>) {
            return std::abs(dot(surface.plane.normal, p)+surface.plane.offset);
        } else if constexpr (std::is_same_v<T, reconstruction::SphereEvidence>) {
            return std::abs(length(minus(p, surface.sphere.center))-surface.sphere.radius);
        } else if constexpr (std::is_same_v<T, reconstruction::CylinderEvidence>) {
            const auto delta = minus(p, surface.cylinder.axisPoint);
            const double axial = dot(delta, surface.cylinder.axisDirection);
            return std::abs(std::sqrt(std::max(0.0, dot(delta,delta)-axial*axial))-surface.cylinder.radius);
        } else if constexpr (std::is_same_v<T, reconstruction::ConeEvidence>) {
            const auto delta = minus(p, surface.cone.apex);
            const double axial = dot(delta, surface.cone.axisDirection);
            if (axial < 0) return std::numeric_limits<double>::infinity();
            const double radial = std::sqrt(std::max(0.0, dot(delta,delta)-axial*axial));
            return std::abs(radial*std::cos(surface.cone.openingAngleRadians)-axial*std::sin(surface.cone.openingAngleRadians));
        } else {
            const auto delta = minus(p, surface.torus.center);
            const double axial = dot(delta, surface.torus.axisDirection);
            const double radial = std::sqrt(std::max(0.0, dot(delta,delta)-axial*axial));
            return std::abs(std::hypot(radial-surface.torus.majorRadius, axial)-surface.torus.minorRadius);
        }
    }, evidence);
}
PointCloudRegion describe(const reconstruction::PrimitiveEvidence& evidence, int index)
{
    PointCloudRegion region;
    region.source = "detected";
    std::visit([&](const auto& surface) {
        using T = std::decay_t<decltype(surface)>;
        region.residual = surface.meanAbsoluteResidual;
        region.confidence = std::clamp(surface.confidence, 0.0, 1.0);
        if constexpr (std::is_same_v<T, reconstruction::PlaneEvidence>) {
            region.geometry = "plane"; region.name = QStringLiteral("平面 %1").arg(index);
            region.parameters = {{"normal", array(surface.plane.normal)}, {"offset", surface.plane.offset}};
        } else if constexpr (std::is_same_v<T, reconstruction::SphereEvidence>) {
            region.geometry = "sphere"; region.name = QStringLiteral("球面 %1").arg(index);
            region.parameters = {{"center", array(surface.sphere.center)}, {"radius", surface.sphere.radius}};
        } else if constexpr (std::is_same_v<T, reconstruction::CylinderEvidence>) {
            region.geometry = "cylinder"; region.name = QStringLiteral("圆柱面 %1").arg(index);
            region.parameters = {{"center", array(surface.cylinder.axisPoint)}, {"axis", array(surface.cylinder.axisDirection)},
                {"radius", surface.cylinder.radius}, {"axial_min", surface.axialMinimum}, {"axial_max", surface.axialMaximum}};
        } else if constexpr (std::is_same_v<T, reconstruction::ConeEvidence>) {
            region.geometry = "cone"; region.name = QStringLiteral("圆锥面 %1").arg(index);
            region.parameters = {{"center", array(surface.cone.apex)}, {"axis", array(surface.cone.axisDirection)},
                {"angle", surface.cone.openingAngleRadians}, {"axial_min", surface.axialMinimum}, {"axial_max", surface.axialMaximum}};
        } else {
            region.geometry = "ring"; region.name = QStringLiteral("环面 %1").arg(index);
            region.parameters = {{"center", array(surface.torus.center)}, {"axis", array(surface.torus.axisDirection)},
                {"major_radius", surface.torus.majorRadius}, {"minor_radius", surface.torus.minorRadius}};
        }
    }, evidence);
    return region;
}
}

PointCloudAnalysisResult analysePointCloud(const PlyCloud& cloud,
    const std::vector<vtkIdType>& originalIds, double distanceTolerance, const QString& geometry)
{
    PointCloudAnalysisResult result;
    if (cloud.positions.size() < 20 || cloud.positions.size() != originalIds.size()) {
        result.error = QStringLiteral("至少需要 20 个有效点进行表面识别。"); return result;
    }
    if (!std::isfinite(distanceTolerance) || distanceTolerance <= 0) {
        result.error = QStringLiteral("表面识别距离容差必须大于零。"); return result;
    }
    const auto source = PointCloudAdapter::toPointStore(cloud, reconstruction::LengthUnit::Arbitrary);
    auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud);
    preprocessing.convertKnownUnitsToMillimeters = false;
    if (cloud.positions.size() < 50000) preprocessing.voxelSize = 0;
    reconstruction::PointStore points({}, reconstruction::LengthUnit::Arbitrary);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    if (!reconstruction::preprocessPointCloud(source, preprocessing, points, report, error)) {
        result.error = QString::fromStdString(error); return result;
    }
    result.processedPoints = points.size();
    if (QThread::currentThread()->isInterruptionRequested()) { result.error = "识别已取消。"; return result; }
    reconstruction::PrimitiveDetectionOptions options;
    options.plane = PointCloudAdapter::planeOptionsFor(points);
    options.cylinder = PointCloudAdapter::cylinderOptionsFor(points);
    const size_t support = std::max<size_t>(20, std::min<size_t>(200, points.size()/200));
    options.plane.distanceThreshold = distanceTolerance;
    options.plane.maximumPlanes = geometry.isEmpty() ? 128 : 1;
    options.cylinder.distanceThreshold = distanceTolerance;
    options.cylinder.minimumSupportPoints = support;
    options.cylinder.maximumCylinders = geometry.isEmpty() ? 32 : 1;
    options.sphere.distanceThreshold = distanceTolerance; options.sphere.minimumSupportPoints = support;
    options.sphere.maximumSpheres = geometry.isEmpty() ? 32 : 1;
    options.cone.distanceThreshold = distanceTolerance; options.cone.minimumSupportPoints = support;
    options.cone.maximumCones = geometry.isEmpty() ? 32 : 1;
    options.torus.distanceThreshold = distanceTolerance; options.torus.minimumSupportPoints = support;
    options.torus.maximumTori = geometry.isEmpty() ? 32 : 1;
    if (!geometry.isEmpty()) {
        options.detectPlanes = geometry == "plane"; options.detectCylinders = geometry == "cylinder";
        options.detectSpheres = geometry == "sphere"; options.detectCones = geometry == "cone";
        options.detectTori = geometry == "ring";
    }
    reconstruction::PrimitiveDetectionResult detection;
    if (!reconstruction::detectPrimitiveEvidence(points, options, detection, error)) {
        result.error = QString::fromStdString(error); return result;
    }
    std::vector<vtkSmartPointer<vtkStaticPointLocator>> supports;
    std::vector<std::vector<vtkIdType>> owners(detection.evidence.size());
    for (const auto& evidence : detection.evidence) {
        vtkNew<vtkPoints> coordinates;
        std::visit([&](const auto& surface) {
            for (auto id : surface.supportPointIds) if (id > 0 && id <= cloud.positions.size())
                coordinates->InsertNextPoint(cloud.positions[static_cast<size_t>(id-1)].data());
        }, evidence);
        vtkNew<vtkPolyData> data; data->SetPoints(coordinates);
        auto locator = vtkSmartPointer<vtkStaticPointLocator>::New();
        locator->SetDataSet(data);
        if (coordinates->GetNumberOfPoints()) locator->BuildLocator();
        supports.push_back(locator);
    }
    // Recover every eligible original observation, constrained to the observed
    // patch so distant coplanar surfaces aren't swallowed by an infinite plane.
    const double supportRadius = std::max(preprocessing.voxelSize*3.0, distanceTolerance*4.0);
    for (size_t i = 0; i < cloud.positions.size(); ++i) {
        if (i % 4096 == 0 && QThread::currentThread()->isInterruptionRequested()) {
            result.error = "识别已取消。"; return result;
        }
        const auto& coordinate = cloud.positions[i];
        Vec point{coordinate[0], coordinate[1], coordinate[2]};
        double best = distanceTolerance*1.5;
        size_t chosen = owners.size();
        for (size_t j = 0; j < detection.evidence.size(); ++j) {
            const double distance = residual(detection.evidence[j], point);
            if (distance > best || supports[j]->GetDataSet()->GetNumberOfPoints() == 0) continue;
            double p[3]{point.x,point.y,point.z}, q[3];
            const auto nearest = supports[j]->FindClosestPoint(p);
            supports[j]->GetDataSet()->GetPoint(nearest, q);
            if (std::hypot(std::hypot(p[0]-q[0],p[1]-q[1]),p[2]-q[2]) > supportRadius) continue;
            best = distance; chosen = j;
        }
        if (chosen != owners.size()) { owners[chosen].push_back(originalIds[i]); ++result.assignedPoints; }
    }
    for (size_t j = 0; j < owners.size(); ++j) {
        if (!owners[j].empty()) result.surfaces.push_back({describe(detection.evidence[j], static_cast<int>(j+1)), std::move(owners[j])});
    }
    return result;
}

PointCloudRegion describeTemplateFit(const reconstruction::TemplateFit& fit, double scale)
{
    PointCloudRegion region;
    if (fit.surface) region=describe(*fit.surface,fit.regionId);
    if (fit.box) {
        const auto& p=fit.box->primitive;
        region.geometry="cuboid"; region.confidence=fit.box->confidence;
        region.parameters={{"origin",array(p.pose.origin)},{"x_direction",array(p.pose.xDirection)},
            {"z_direction",array(p.pose.zDirection)},{"size_x",p.sizeX},{"size_y",p.sizeY},{"size_z",p.sizeZ}};
    }
    // Fits run in CAD millimetres; annotation equations stay in PLY units.
    for (const auto& key : {"origin","center"}) if(region.parameters.contains(key)) {
        auto values=region.parameters[key].toArray();
        for(int i=0;i<values.size();++i) values[i]=values[i].toDouble()/scale;
        region.parameters[key]=values;
    }
    for (const auto& key : {"offset","radius","major_radius","minor_radius","axial_min","axial_max",
                           "size_x","size_y","size_z"}) if(region.parameters.contains(key))
        region.parameters[key]=region.parameters[key].toDouble()/scale;
    region.residual/=scale;
    return region;
}
