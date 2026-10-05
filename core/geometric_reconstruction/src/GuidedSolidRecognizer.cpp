#include "reconstruction/GuidedSolidRecognizer.h"
#include "reconstruction/BoxRecognizer.h"
#include "reconstruction/ModelingAdapter.h"
#include "Math3.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace reconstruction {
namespace {
PrimitiveDetectionOptions detectionFor(TemplateKind kind, double tolerance)
{
    PrimitiveDetectionOptions o;
    o.detectPlanes = kind == TemplateKind::Plane;
    o.detectCylinders = kind == TemplateKind::Cylinder;
    o.detectSpheres = kind == TemplateKind::Sphere;
    o.detectCones = kind == TemplateKind::Cone;
    o.detectTori = kind == TemplateKind::Torus;
    o.plane.distanceThreshold = o.cylinder.distanceThreshold = o.sphere.distanceThreshold =
        o.cone.distanceThreshold = o.torus.distanceThreshold = tolerance;
    o.plane.minimumSupportPoints = o.cylinder.minimumSupportPoints = o.sphere.minimumSupportPoints =
        o.cone.minimumSupportPoints = o.torus.minimumSupportPoints = 20;
    o.plane.maximumPlanes = o.cylinder.maximumCylinders = o.sphere.maximumSpheres =
        o.cone.maximumCones = o.torus.maximumTori = 1;
    return o;
}
std::size_t supportSize(const PrimitiveEvidence& e)
{
    return std::visit([](const auto& surface) { return surface.supportPointIds.size(); }, e);
}

// Recover the rest of a manually identified component from its observed
// equation and outward normals. Capped families also admit observed disks
// near the lateral extent; the complete-solid recognizer still verifies caps
// and angular coverage before accepting any material.
std::vector<PointSample> expandCurvedSupport(const PrimitiveEvidence& evidence,
                                             const std::vector<PointSample>& samples,
                                             double tolerance)
{
    using namespace math3;
    return std::visit(
        [&](const auto& surface) {
            using S = std::decay_t<decltype(surface)>;
            std::vector<PointSample> expanded;
            if constexpr (std::is_same_v<S, SphereEvidence>) {
                modeling::CurvedPrimitiveParams params =
                    modeling::SpherePrimitiveParams{surface.sphere.center, surface.sphere.radius};
                for (const auto& p : samples)
                    if (matchesCurvedBoundary(params, p, tolerance * 3))
                        expanded.push_back(p);
            } else if constexpr (std::is_same_v<S, TorusEvidence>) {
                modeling::CurvedPrimitiveParams params = modeling::TorusPrimitiveParams{
                    surface.torus.center, surface.torus.axisDirection, surface.torus.majorRadius,
                    surface.torus.minorRadius};
                for (const auto& p : samples)
                    if (matchesCurvedBoundary(params, p, tolerance * 3))
                        expanded.push_back(p);
            } else if constexpr (std::is_same_v<S, CylinderEvidence> ||
                                 std::is_same_v<S, ConeEvidence>) {
                modeling::Vec3 center, axis;
                double radius, slope;
                if constexpr (std::is_same_v<S, CylinderEvidence>) {
                    center = surface.cylinder.axisPoint;
                    axis = normalized(surface.cylinder.axisDirection);
                    radius = surface.cylinder.radius;
                    slope = 0;
                } else {
                    center = surface.cone.apex;
                    axis = normalized(surface.cone.axisDirection);
                    radius = 0;
                    slope = std::tan(surface.cone.openingAngleRadians);
                }
                double minimum = INFINITY, maximum = -INFINITY;
                std::vector<std::pair<PointSample, double>> disks;
                for (const auto& p : samples) {
                    if (!p.normal)
                        continue;
                    const auto delta = subtract(p.position, center);
                    const double axial = dot(delta, axis), expectedRadius = radius + axial * slope;
                    const auto radial = subtract(delta, scale(axis, axial));
                    const double rho = length(radial);
                    if (expectedRadius < 0)
                        continue;
                    const auto normal = normalized(*p.normal);
                    if (std::abs(dot(normal, axis)) > 0.98 && rho <= expectedRadius + tolerance * 3)
                        disks.push_back({p, axial});
                    if (rho <= 1e-12)
                        continue;
                    const auto outward =
                        normalized(subtract(scale(radial, 1 / rho), scale(axis, slope)));
                    if (std::abs(rho - expectedRadius) / std::sqrt(1 + slope * slope) >
                            tolerance * 3 ||
                        dot(normal, outward) < 0.8)
                        continue;
                    expanded.push_back(p);
                    minimum = std::min(minimum, axial);
                    maximum = std::max(maximum, axial);
                }
                if (!expanded.empty()) {
                    const double margin = (maximum - minimum) * 0.1 + tolerance * 3;
                    for (const auto& disk : disks)
                        if (disk.second >= minimum - margin && disk.second <= maximum + margin)
                            expanded.push_back(disk.first);
                }
            }
            return expanded;
        },
        evidence);
}
GeomAbs_SurfaceType faceKind(TemplateKind kind)
{
    switch (kind) {
    case TemplateKind::Plane:
    case TemplateKind::Box:
        return GeomAbs_Plane;
    case TemplateKind::Cylinder:
        return GeomAbs_Cylinder;
    case TemplateKind::Sphere:
        return GeomAbs_Sphere;
    case TemplateKind::Cone:
        return GeomAbs_Cone;
    case TemplateKind::Torus:
        return GeomAbs_Torus;
    }
    return GeomAbs_OtherSurface;
}
bool onBox(const modeling::BoxPrimitiveParams& box, const PointSample& p, double tolerance)
{
    using namespace math3;
    const auto x = normalized(box.pose.xDirection), z = normalized(box.pose.zDirection),
               y = cross(z, x);
    const auto d = subtract(p.position, box.pose.origin);
    const std::array<modeling::Vec3, 3> axes{x, y, z};
    const std::array<double, 3> sizes{box.sizeX, box.sizeY, box.sizeZ};
    bool boundary = false;
    for (int i = 0; i < 3; ++i) {
        const double c = dot(d, axes[i]);
        if (c < -tolerance || c > sizes[i] + tolerance)
            return false;
        boundary |= std::abs(c) <= tolerance || std::abs(c - sizes[i]) <= tolerance;
    }
    return boundary;
}
bool verifyHints(const PointStore& points, const std::vector<TemplateHint>& hints,
                 const CompoundCandidate& candidate, const std::vector<TemplateFit>& fits,
                 double tolerance, int& failed, std::string& error,
                 const std::function<bool()>& cancelled)
{
    modeling::ModelingCore model;
    const auto built = commitCompoundCandidate(candidate, model);
    if (!built.success) {
        error = built.error;
        return false;
    }
    std::map<GeomAbs_SurfaceType, std::vector<TopoDS_Face>> faces;
    for (TopExp_Explorer e(model.bodyShape(), TopAbs_FACE); e.More(); e.Next()) {
        const auto face = TopoDS::Face(e.Current());
        faces[BRepAdaptor_Surface(face).GetType()].push_back(face);
    }
    for (std::size_t h = 0; h < hints.size(); ++h) {
        if (cancelled && cancelled()) {
            error = "Cancelled";
            return false;
        }
        std::set<PointId> selected(hints[h].points.begin(), hints[h].points.end());
        std::size_t total = 0, matched = 0;
        for (const auto& point : points.points())
            if (selected.count(point.id)) {
                if (cancelled && cancelled()) {
                    error = "Cancelled";
                    return false;
                }
                ++total;
                if (fits[h].solid &&
                    matchesCurvedBoundary(fits[h].solid->primitive, point, tolerance * 3)) {
                    ++matched;
                    continue;
                }
                const auto vertex =
                    BRepBuilderAPI_MakeVertex(
                        gp_Pnt(point.position.x, point.position.y, point.position.z))
                        .Vertex();
                for (const auto& face : faces[faceKind(hints[h].kind)]) {
                    BRepExtrema_DistShapeShape distance(vertex, face);
                    if (distance.IsDone() && distance.Value() <= tolerance * 3) {
                        ++matched;
                        break;
                    }
                }
            }
        if (!total || matched < total * 0.9) {
            failed = hints[h].regionId;
            error = "The complete CAD boundary conflicts with the selected template (support " +
                    std::to_string(matched) + "/" + std::to_string(total) + ")";
            return false;
        }
    }
    return true;
}
} // namespace

bool reconstructGuidedSolid(const PointStore& points, const PlaneDetectionOptions& planes,
                            const CylinderDetectionOptions& cylinders,
                            const std::vector<TemplateHint>& hints, GuidedSolidResult& output,
                            std::string& error, const std::function<bool()>& cancelled)
{
    output = {};
    if (hints.empty())
        return reconstructParametricSolid(points, planes, cylinders, output.candidate, error);
    auto samples = points.points();
    std::map<PointId, std::size_t> indices;
    for (std::size_t i = 0; i < samples.size(); ++i)
        indices[samples[i].id] = i;
    std::set<PointId> assigned;
    std::vector<PrimitiveEvidence> seeds;
    std::vector<CurvedSolidCandidate> closed;
    for (const auto& hint : hints) {
        if (cancelled && cancelled()) {
            error = "Cancelled";
            return false;
        }
        output.failedRegionId = hint.regionId;
        std::vector<PointSample> selected;
        for (auto id : hint.points) {
            if (!assigned.insert(id).second) {
                error = "Overlapping template groups";
                return false;
            }
            const auto found = indices.find(id);
            if (found != indices.end())
                selected.push_back(samples[found->second]);
        }
        if (selected.size() < 20) {
            error = "At least 20 valid retained points are required for a template";
            return false;
        }
        const PointStore subset(selected, points.unit());
        TemplateFit fit;
        fit.regionId = hint.regionId;
        if (hint.kind == TemplateKind::Box) {
            auto settings = planes;
            settings.minimumSupportPoints = 20;
            settings.maximumPlanes = 6;
            BoxCandidate box;
            if (!reconstructBox(subset, settings, {}, box, error)) {
                // A local patch cannot determine all dimensions by itself.
                // Complete it only from actual observations of the whole cloud.
                CompoundCandidate context;
                if (!reconstructParametricSolid(points, planes, cylinders, context, error))
                    return false;
                const auto found =
                    std::find_if(context.boxes.begin(), context.boxes.end(), [&](const auto& c) {
                        return std::count_if(selected.begin(), selected.end(), [&](const auto& p) {
                                   return onBox(c.primitive, p, planes.distanceThreshold * 3);
                               }) >= selected.size() * 0.9;
                    });
                if (found == context.boxes.end()) {
                    error = "No observed box boundary supports this template patch";
                    return false;
                }
                box = *found;
            }
            fit.box = box;
            for (const auto& plane : box.sourceSurfaces)
                seeds.push_back(plane);
        } else {
            PrimitiveDetectionResult proposals;
            if (!detectPrimitiveEvidence(subset, detectionFor(hint.kind, planes.distanceThreshold),
                                         proposals, error))
                return false;
            if (!proposals.evidence.empty()) {
                fit.surface = *std::max_element(
                    proposals.evidence.begin(), proposals.evidence.end(),
                    [](const auto& a, const auto& b) { return supportSize(a) < supportSize(b); });
            }
            if (hint.kind != TemplateKind::Plane) {
                CurvedSolidRecognitionOptions settings;
                settings.distanceTolerance = planes.distanceThreshold;
                settings.requiredKind = static_cast<PrimitiveKind>(hint.kind);
                if (fit.surface)
                    settings.seeds.push_back(*fit.surface);
                CurvedSolidCandidate solid;
                std::string incomplete;
                bool complete = reconstructCurvedSolid(subset, settings, solid, incomplete);
                if (!complete && fit.surface) {
                    const auto expanded =
                        expandCurvedSupport(*fit.surface, samples, planes.distanceThreshold);
                    if (expanded.size() >= 20)
                        complete = reconstructCurvedSolid(PointStore(expanded, points.unit()),
                                                          settings, solid, incomplete);
                }
                if (complete) {
                    const auto same =
                        std::find_if(closed.begin(), closed.end(), [&](const auto& known) {
                            return std::count_if(
                                       selected.begin(), selected.end(), [&](const auto& p) {
                                           return matchesCurvedBoundary(
                                               known.primitive, p, planes.distanceThreshold * 3);
                                       }) >= selected.size() * 0.9;
                        });
                    if (same != closed.end())
                        solid = *same;
                    else
                        closed.push_back(solid);
                    fit.solid = solid;
                }
            }
            if (!fit.solid && (!fit.surface || supportSize(*fit.surface) < selected.size() * 0.7)) {
                error = "Selected points do not support the requested template; correct membership "
                        "or add observations";
                return false;
            }
            if (fit.surface)
                seeds.push_back(*fit.surface);
        }
        // The user's planar equation guides normal/frame recovery. Preserve
        // outward orientation; the original document's observations stay intact.
        std::vector<PlaneEvidence> fixed;
        if (fit.box)
            fixed = fit.box->sourceSurfaces;
        if (fit.surface)
            if (const auto* p = std::get_if<PlaneEvidence>(&*fit.surface))
                fixed.push_back(*p);
        for (const auto& p : fixed)
            for (auto id : p.supportPointIds) {
                auto& sample = samples[indices.at(id)];
                auto normal = p.plane.normal;
                if (sample.normal && math3::dot(normal, *sample.normal) < 0)
                    normal = math3::scale(normal, -1);
                sample.normal = normal;
                const double offset = math3::dot(p.plane.normal, sample.position) + p.plane.offset;
                sample.position =
                    math3::subtract(sample.position, math3::scale(p.plane.normal, offset));
            }
        output.fits.push_back(std::move(fit));
    }
    output.failedRegionId = 0;
    if (cancelled && cancelled()) {
        error = "Cancelled";
        return false;
    }
    // A complete manually identified curved component is fixed first. Its
    // remaining observed boundary is consumed, and automatic reconstruction
    // operates on the remaining material rather than refitting that component.
    std::vector<PointSample> remaining;
    for (const auto& p : samples) {
        const bool consumed = std::any_of(closed.begin(), closed.end(), [&](const auto& solid) {
            return matchesCurvedBoundary(solid.primitive, p, planes.distanceThreshold * 3);
        });
        if (!consumed)
            remaining.push_back(p);
    }
    if (!remaining.empty()) {
        std::set<PointId> remainingIds;
        for (const auto& p : remaining)
            remainingIds.insert(p.id);
        seeds.erase(
            std::remove_if(seeds.begin(), seeds.end(),
                           [&](auto& seed) {
                               return std::visit(
                                   [&](auto& evidence) {
                                       auto& ids = evidence.supportPointIds;
                                       ids.erase(std::remove_if(ids.begin(), ids.end(),
                                                                [&](auto id) {
                                                                    return !remainingIds.count(id);
                                                                }),
                                                 ids.end());
                                       return ids.size() < 20;
                                   },
                                   seed);
                           }),
            seeds.end());
        if (!reconstructParametricSolid(PointStore(remaining, points.unit()), planes, cylinders,
                                        output.candidate, error, seeds))
            return false;
    }
    output.candidate.curvedSolids.insert(output.candidate.curvedSolids.end(), closed.begin(),
                                         closed.end());
    if (output.candidate.boxes.empty() && output.candidate.prisms.empty() &&
        output.candidate.curvedSolids.empty()) {
        error = "No supported material";
        return false;
    }
    if (!verifyHints(points, hints, output.candidate, output.fits, planes.distanceThreshold,
                     output.failedRegionId, error, cancelled))
        return false;
    output.candidate.surfaceCoverage =
        (remaining.size() * output.candidate.surfaceCoverage + (points.size() - remaining.size())) /
        points.size();
    output.candidate.confidence = output.candidate.surfaceCoverage;
    error.clear();
    return true;
}
} // namespace reconstruction
