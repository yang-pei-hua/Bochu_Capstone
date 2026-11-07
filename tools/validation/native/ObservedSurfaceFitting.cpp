#include "SurfaceCoverageValidation.h"
#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"
#include "preprocessing/detail/KdTree.h"

#include <Eigen/QR>

#include <modeling/StepExporter.h>
#include <reconstruction/analysis/SurfaceAnalysis.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Reader.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Torus.hxx>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QSaveFile>

#include <algorithm>
#include <bitset>
#include <cmath>
#include <filesystem>
#include <random>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace {
using PointIds = std::unordered_set<reconstruction::PointId>;
constexpr int maximumAngularBins = 24;
constexpr int maximumAxialBins = 4;
constexpr double pi = 3.14159265358979323846;
constexpr double fullTurn = 2 * pi;

struct AxialFit {
    gp_Ax3 frame;
    double radius = 0;
    double angle = 0;
    double low = 0;
    double high = 0;
    double startAngle = 0;
    double angularSpan = fullTurn;
    double coverage = 0;
    double occupied = 0;
    double meanResidual = 0;
    int normalDirection = 0;
    bool gridSupported = false;
    bool periodicGridSupported = false;
};

std::optional<AxialFit> measure(const reconstruction::PointStore& points,
                                const reconstruction::PrimitiveEvidence& surface, double tolerance,
                                std::size_t minimumSamples = 80, bool preciseCylinder = false) {
    AxialFit fit;
    if (const auto* cylinder = std::get_if<reconstruction::CylinderEvidence>(&surface)) {
        const auto& [p, axis, radius] = cylinder->cylinder;
        fit.frame = gp_Ax3(gp_Pnt(p.x, p.y, p.z), gp_Dir(axis.x, axis.y, axis.z));
        fit.radius = radius;
    } else if (const auto* cone = std::get_if<reconstruction::ConeEvidence>(&surface)) {
        const auto& [p, axis, angle] = cone->cone;
        fit.frame = gp_Ax3(gp_Pnt(p.x, p.y, p.z), gp_Dir(axis.x, axis.y, axis.z));
        fit.angle = angle;
    } else return std::nullopt;

    struct Sample { double axial, angular, residual; int direction; };
    std::vector<Sample> matches;
    matches.reserve(points.size());
    const gp_Vec axis(fit.frame.Direction());
    const gp_Vec xDirection(fit.frame.XDirection()), yDirection(fit.frame.YDirection());
    for (const auto& point : points.points()) {
        const double residual = reconstruction::surfaceResidual(surface, point.position);
        if (residual > tolerance || !point.normal ||
            reconstruction::surfaceNormalAlignment(surface, point.position, *point.normal) < .95)
            continue;
        const auto& p = point.position;
        const gp_Vec local(fit.frame.Location(), gp_Pnt(p.x, p.y, p.z));
        const double axial = local.Dot(axis);
        const double azimuth = std::atan2(local.Dot(yDirection), local.Dot(xDirection));
        int direction = 0;
        if (point.normalOrientationKnown) {
            const auto& n = *point.normal;
            const gp_Vec radial = local - axis * axial;
            const gp_Vec outward = radial - axis * (radial.Magnitude() * std::tan(fit.angle));
            direction = outward.Dot(gp_Vec(n.x, n.y, n.z)) > 0 ? 1 : -1;
        }
        matches.push_back({axial, azimuth < 0 ? azimuth + fullTurn : azimuth, residual, direction});
    }
    if (matches.size() < minimumSamples || matches.size() < points.size() * .98) return std::nullopt;
    std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) { return a.axial < b.axial; });
    const auto tail = matches.size() / 1000;
    fit.low = matches[tail].axial;
    fit.high = matches[matches.size() - 1 - tail].axial;
    if (fit.high - fit.low <= 4 * tolerance) return std::nullopt;

    std::vector<double> angles;
    angles.reserve(matches.size());
    for (const auto& sample : matches)
        if (sample.axial >= fit.low - tolerance && sample.axial <= fit.high + tolerance)
            angles.push_back(sample.angular);
    std::sort(angles.begin(), angles.end());
    double largestGap = angles.front() + fullTurn - angles.back();
    double firstAngleAfterGap = angles.front();
    for (std::size_t i = 1; i < angles.size(); ++i) {
        if (angles[i] - angles[i - 1] <= largestGap) continue;
        largestGap = angles[i] - angles[i - 1];
        firstAngleAfterGap = angles[i];
    }
    // Keep a missing angular sector outside the trim. The occupancy test
    // below rejects disconnected bands inside the remaining interval.
    if (largestGap > fullTurn / 72) {
        fit.startAngle = firstAngleAfterGap;
        fit.angularSpan = fullTurn - largestGap;
    }
    const double minimumSpan = preciseCylinder ? std::max(.04,4*tolerance/fit.radius) : fullTurn/maximumAngularBins;
    if (fit.angularSpan <= minimumSpan) return std::nullopt;

    // Use enough cells to check the domain without requiring near-apex
    // sectors to contain as many observations as much larger rim sectors.
    const int axialBins = matches.size() >= 256 ? maximumAxialBins : 3;
    const int angularBins = std::clamp(int(matches.size() / (axialBins * 8)), 6, maximumAngularBins);
    std::bitset<maximumAngularBins * maximumAxialBins> bins;
    const double height = fit.high - fit.low;
    std::size_t supported = 0;
    std::size_t outward = 0, inward = 0;
    double residualSum = 0;
    for (const auto& sample : matches) {
        if (sample.axial < fit.low - tolerance || sample.axial > fit.high + tolerance) continue;
        const double angle = sample.angular >= fit.startAngle ? sample.angular - fit.startAngle
                                                              : sample.angular + fullTurn - fit.startAngle;
        const auto u = std::min(angularBins - 1, int(angle * angularBins / fit.angularSpan));
        double fraction = (sample.axial - fit.low) / height;
        if (fit.angle > 0) fraction *= (sample.axial + fit.low) / (fit.high + fit.low);
        const auto v = std::clamp(int(fraction * axialBins), 0, axialBins - 1);
        bins.set(u * axialBins + v);
        ++supported;
        outward += sample.direction > 0;
        inward += sample.direction < 0;
        residualSum += sample.residual;
    }
    fit.coverage = double(supported) / points.size();
    fit.occupied = double(bins.count()) / (angularBins * axialBins);
    if (fit.coverage < .98) return std::nullopt;
    fit.gridSupported = fit.occupied >= .95;
    fit.periodicGridSupported = fit.angle==0 && fit.angularSpan>=.98*fullTurn && fit.occupied>=.92;
    for (int u = 0; u < angularBins; ++u) {
        int layers = 0;
        for (int v = 0; v < axialBins; ++v) layers += bins.test(u * axialBins + v);
        fit.gridSupported &= layers>0;
        fit.periodicGridSupported &= layers>=axialBins-1;
    }
    fit.meanResidual = residualSum / supported;
    if (outward >= supported * .98) fit.normalDirection = 1;
    if (inward >= supported * .98) fit.normalDirection = -1;
    return fit;
}

int validFaceCount(const TopoDS_Shape& shape) {
    if (shape.IsNull() || TopExp_Explorer(shape, TopAbs_SOLID).More() || !BRepCheck_Analyzer(shape).IsValid())
        return 0;
    int count = 0;
    for (TopExp_Explorer faces(shape,TopAbs_FACE); faces.More(); faces.Next()) ++count;
    return count;
}

bool oneValidFace(const TopoDS_Shape& shape) { return validFaceCount(shape) == 1; }

std::vector<reconstruction::FiniteSurfacePatch> splitPlaneIslands(const reconstruction::FiniteSurfacePatch& patch) {
    const auto& n = patch.normal;
    const auto& x = patch.xDirection;
    const gp_Vec normal(n.x,n.y,n.z), u(x.x,x.y,x.z), v=normal.Crossed(u);
    const gp_Pnt origin(patch.origin.x,patch.origin.y,patch.origin.z);
    const auto toChart = [&](const modeling::Vec3& p) {
        const gp_Vec d(origin,gp_Pnt(p.x,p.y,p.z));
        return gp_Pnt2d(d.Dot(u),d.Dot(v));
    };
    std::vector<std::size_t> outer, holes;
    for (std::size_t k=0; k<patch.loops.size(); ++k) {
        const auto& loop = patch.loops[k];
        double area = 0;
        for (std::size_t i=0; i<loop.size(); ++i) {
            const auto a=toChart(loop[i]), b=toChart(loop[(i+1)%loop.size()]);
            area += a.X()*b.Y()-a.Y()*b.X();
        }
        if (area>0) outer.push_back(k);
        if (area<0) holes.push_back(k);
    }
    if (outer.size()<2 || outer.size()>16) return {};
    const auto contains = [&](const auto& loop, const gp_Pnt2d& p) {
        bool inside = false;
        for (std::size_t i=0; i<loop.size(); ++i) {
            const auto a=toChart(loop[i]), b=toChart(loop[(i+1)%loop.size()]);
            if ((a.Y()>p.Y())==(b.Y()>p.Y())) continue;
            if (p.X()<a.X()+(p.Y()-a.Y())*(b.X()-a.X())/(b.Y()-a.Y())) inside=!inside;
        }
        return inside;
    };
    std::vector<reconstruction::FiniteSurfacePatch> islands;
    for (auto i : outer) {
        reconstruction::FiniteSurfacePatch island;
        island.origin=patch.origin;
        island.normal=patch.normal;
        island.spacing=patch.spacing;
        island.loops.push_back(patch.loops[i]);
        for (auto h : holes) {
            if (contains(patch.loops[i],toChart(patch.loops[h].front()))) island.loops.push_back(patch.loops[h]);
        }
        islands.push_back(std::move(island));
    }
    return islands;
}

template<class Parametrize>
TopoDS_Face boundedParametricFace(const Handle(Geom_Surface)& surface,
                                 const std::vector<std::vector<modeling::Vec3>>& loops,
                                 Parametrize uv) {
    if (loops.empty()) return {};
    BRepBuilderAPI_MakeFace face;
    face.Init(surface,false,1e-7);
    bool valid = true;
    for (std::size_t i=0; i<loops.size(); ++i) {
        const auto& loop = loops[i];
        if (loop.size()<3 || loop.size()>512) { valid = false; break; }
        double area = 0;
        BRepBuilderAPI_MakeWire wire;
        for (std::size_t j=0; j<loop.size(); ++j) {
            const auto& a = loop[j];
            const auto& b = loop[(j+1)%loop.size()];
            area += a.x*b.y-a.y*b.x;
            const auto start = uv(a), end = uv(b);
            if (start.Distance(end)<=1e-10) { valid = false; break; }
            const auto segment = GCE2d_MakeSegment(start,end).Value();
            BRepBuilderAPI_MakeEdge edge(segment,surface);
            if (!edge.IsDone()) { valid = false; break; }
            wire.Add(edge.Edge());
        }
        if (!valid || !wire.IsDone() || (i==0 ? area<=0 : area>=0)) { valid = false; break; }
        face.Add(wire.Wire());
    }
    if (!valid || !face.IsDone() || !BRepLib::BuildCurves3d(face.Face()) || !oneValidFace(face.Face())) return {};
    return face.Face();
}

struct ObservedPatch {
    TopoDS_Face face;
    QJsonObject report;
    PointIds points;
};

std::vector<ObservedPatch> trimAxialSurfaces(const reconstruction::PointStore& points,
                                        const reconstruction::PrimitiveEvidence& proposal, const AxialFit& fit,
                                        double tolerance, double minimumCoverage, std::size_t minimumSamples) {
    if (fit.angularSpan >= .9*fullTurn || fit.normalDirection == 0) return {};
    const bool cone = fit.angle>0;
    const double angularScale = cone ? (fit.low+fit.high)*.5*std::tan(fit.angle) : fit.radius;
    const double axialScale = std::cos(fit.angle);
    if (angularScale<=tolerance) return {};
    const gp_Vec axis(fit.frame.Direction()), x(fit.frame.XDirection()), y(fit.frame.YDirection());
    std::vector<reconstruction::PointSample> chart;
    chart.reserve(points.size());
    for (const auto& point : points.points()) {
        if (reconstruction::surfaceResidual(proposal,point.position)>tolerance || !point.normal ||
            reconstruction::surfaceNormalAlignment(proposal,point.position,*point.normal)<.95) continue;
        const auto& p = point.position;
        const gp_Vec d(fit.frame.Location(),gp_Pnt(p.x,p.y,p.z));
        const double angle = std::atan2(d.Dot(y),d.Dot(x));
        const double u = std::remainder(angle-fit.startAngle-fit.angularSpan*.5,fullTurn)+fit.angularSpan*.5;
        chart.push_back({point.id,{u*angularScale,(d.Dot(axis)-fit.low)/axialScale,0},modeling::Vec3{0,0,1},1,std::nullopt,true});
    }
    reconstruction::AssemblyOptions options;
    options.buildSolids = false;
    options.distanceTolerance = tolerance;
    reconstruction::AssemblyReconstructionResult boundary;
    const reconstruction::PointStore samples(std::move(chart));
    std::string error;
    if (!reconstruction::reconstructAssembly(samples,options,boundary,error)) return {};
    Handle(Geom_Surface) surface;
    if (cone) surface = new Geom_ConicalSurface(fit.frame,fit.angle,0);
    else surface = new Geom_CylindricalSurface(fit.frame,fit.radius);
    std::vector<ObservedPatch> patches;
    PointIds covered;
    const auto trimPatch = [&](const reconstruction::FiniteSurfacePatch& patch, const auto& loops) {
        if (loops.empty()) return;
        auto uv = [&](const modeling::Vec3& p) { return gp_Pnt2d(fit.startAngle+p.x/angularScale,fit.low/axialScale+p.y); };
        const auto face = boundedParametricFace(surface,loops,uv);
        if (face.IsNull()) return;

        BRepClass_FaceClassifier classifier;
        const double trimTolerance = std::max(tolerance,patch.spacing*.5);
        const PointIds patchIds(patch.pointIds.begin(),patch.pointIds.end());
        PointIds supported;
        for (const auto& sample : samples.points()) {
            if (!patchIds.count(sample.id)) continue;
            const auto q = uv(sample.position);
            classifier.Perform(face,surface->Value(q.X(),q.Y()),trimTolerance);
            if (classifier.State()==TopAbs_IN || classifier.State()==TopAbs_ON) supported.insert(sample.id);
        }
        if (supported.size()<minimumSamples) return;
        covered.insert(supported.begin(),supported.end());
        BRepAdaptor_Surface trimmed(face);
        QJsonObject report{{"trim_kind","observed_parameter_boundary"},{"trim_tolerance_mm",trimTolerance},
            {"trim_coverage",double(supported.size())/points.size()},{"inner_loops",double(loops.size()-1)},
            {"angular_span_rad",trimmed.LastUParameter()-trimmed.FirstUParameter()},
            {"axial_bounds_mm",QJsonArray{trimmed.FirstVParameter()*axialScale,trimmed.LastVParameter()*axialScale}}};
        patches.push_back({face,std::move(report),std::move(supported)});
    };
    for (const auto& patch : boundary.patches) {
        if (!patch.planar || patch.pointIds.size()<minimumSamples) continue;
        const auto islands = splitPlaneIslands(patch);
        if (islands.empty()) trimPatch(patch,patch.loops);
        else for (const auto& island : islands) trimPatch(patch,island.loops);
        if (patches.size()>128) return {};
    }
    return covered.size()>=points.size()*minimumCoverage ? std::move(patches) : std::vector<ObservedPatch>{};
}

TopoDS_Face planePatch(const reconstruction::FiniteSurfacePatch& patch,
                        const std::vector<const reconstruction::PointSample*>& points,
                        double tolerance, QJsonObject& report, PointIds* assigned = nullptr,
                        double minimumCoverage = .98) {
    if (points.empty() || patch.loops.empty()) return {};
    const auto& [x, y, z] = patch.origin;
    const auto& n = patch.normal;
    const gp_Pln plane(gp_Pnt(x, y, z), gp_Dir(n.x, n.y, n.z));
    const gp_Vec normal(plane.Axis().Direction());
    const auto build = [&](const auto& loops) -> TopoDS_Face {
        std::optional<BRepBuilderAPI_MakeFace> face;
        for (std::size_t i = 0; i < loops.size(); ++i) {
            const auto& loop = loops[i];
            if (loop.size() < 3) return {};
            BRepBuilderAPI_MakePolygon polygon;
            double area = 0;
            for (std::size_t j = 0; j < loop.size(); ++j) {
                const auto& a = loop[j];
                const auto& b = loop[(j + 1) % loop.size()];
                const gp_Vec p(plane.Location(), gp_Pnt(a.x, a.y, a.z));
                const gp_Vec q(plane.Location(), gp_Pnt(b.x, b.y, b.z));
                area += p.Crossed(q).Dot(normal);
                polygon.Add(plane.Location().Translated(p - normal * p.Dot(normal)));
            }
            polygon.Close();
            if (!polygon.IsDone() || (i == 0 ? area <= 0 : area >= 0)) return {};
            if (i == 0) face.emplace(plane, polygon.Wire(), true);
            else if (face->IsDone()) face->Add(polygon.Wire());
        }
        return face->IsDone() && oneValidFace(face->Face()) ? face->Face() : TopoDS_Face{};
    };
    auto face = build(patch.loops);
    if (face.IsNull() && patch.loops.size()>1) {
        // Alpha boundaries can share a sample at a pinched hole. Separate
        // only that vertex, within the physical tolerance, and verify again.
        auto separated = patch.loops;
        int moved = 0;
        for (std::size_t i=1; i<separated.size(); ++i) {
            const auto& loop = patch.loops[i];
            if (loop.size()<3) continue;
            for (std::size_t j=0; j<loop.size(); ++j) {
                const auto& p = loop[j];
                bool touching = false;
                for (std::size_t k=0; k<i && !touching; ++k) {
                    touching = std::any_of(separated[k].begin(),separated[k].end(),[&](const auto& q) {
                        return std::hypot(q.x-p.x,q.y-p.y,q.z-p.z)<=1e-7;
                    });
                }
                if (!touching) continue;
                const auto& a=loop[(j+loop.size()-1)%loop.size()];
                const auto& b=loop[(j+1)%loop.size()];
                const gp_Vec before(a.x-p.x,a.y-p.y,a.z-p.z), after(b.x-p.x,b.y-p.y,b.z-p.z);
                const double edge = std::min(before.Magnitude(),after.Magnitude());
                if (edge<=1e-7) continue;
                auto inward = before.Normalized()+after.Normalized();
                if (inward.SquareMagnitude()<=1e-12) continue;
                const double sign = before.Crossed(after).Dot(normal)>0 ? 1 : -1;
                inward *= sign*std::min(tolerance*.5,edge*.05)/inward.Magnitude();
                separated[i][j] = {p.x+inward.X(),p.y+inward.Y(),p.z+inward.Z()};
                ++moved;
            }
        }
        if (moved) {
            face = build(separated);
            if (!face.IsNull()) report["separated_trim_vertices"] = moved;
        }
    }
    if (face.IsNull()) return {};
    BRepClass_FaceClassifier classifier;
    PointIds supported;
    double residualSum = 0;
    // Loop simplification is tied to sample spacing. Keep the physical
    // plane tolerance independent from that in-plane trim uncertainty.
    const double trimTolerance = std::max(tolerance, patch.spacing * .5);
    const reconstruction::PrimitiveEvidence fitted = reconstruction::PlaneEvidence{
        0, {patch.normal, -(x * n.x + y * n.y + z * n.z)}};
    for (const auto* sample : points) {
        const auto& point = *sample;
        const double residual = reconstruction::surfaceResidual(fitted, point.position);
        if (residual > tolerance || !point.normal ||
            reconstruction::surfaceNormalAlignment(fitted, point.position, *point.normal) < .95) continue;
        const auto& p = point.position;
        classifier.Perform(face, gp_Pnt(p.x, p.y, p.z), trimTolerance);
        if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON) continue;
        supported.insert(point.id);
        residualSum += residual;
    }
    report["planar_trim_coverage"] = double(supported.size()) / points.size();
    if (supported.empty() || supported.size() < points.size() * minimumCoverage) return {};
    report["kind"] = "plane";
    report["normal"] = QJsonArray{n.x, n.y, n.z};
    report["origin_mm"] = QJsonArray{x, y, z};
    report["inner_loops"] = double(patch.loops.size() - 1);
    report["trim_tolerance_mm"] = trimTolerance;
    report["surface_coverage"] = double(supported.size()) / points.size();
    report["mean_residual_mm"] = residualSum / supported.size();
    if (assigned) assigned->insert(supported.begin(),supported.end());
    return face;
}

TopoDS_Face fitPlanarFace(const reconstruction::PointStore& points,
                         reconstruction::PrimitiveDetectionOptions options, double tolerance,
                         QJsonObject& report) {
    options.detectPlanes = true;
    options.detectCylinders = options.detectCones = false;
    options.plane.distanceThreshold = tolerance;
    reconstruction::PrimitiveDetectionResult proposals;
    std::string error;
    if (!reconstruction::detectPrimitiveEvidence(points, options, proposals, error)) return {};
    std::size_t bestPlaneSupport = 0;
    for (const auto& evidence : proposals.evidence)
        bestPlaneSupport = std::max(bestPlaneSupport, std::get<reconstruction::PlaneEvidence>(evidence).supportPointIds.size());
    report["planar_proposal_coverage"] = double(bestPlaneSupport) / points.size();
    if (bestPlaneSupport < points.size() * .98) return {};

    reconstruction::AssemblyOptions boundaryOptions;
    boundaryOptions.buildSolids = false;
    boundaryOptions.distanceTolerance = tolerance;
    reconstruction::AssemblyReconstructionResult boundary;
    if (!reconstruction::reconstructAssembly(points, boundaryOptions, boundary, error)) return {};
    std::vector<const reconstruction::PointSample*> samples;
    samples.reserve(points.size());
    for (const auto& p : points.points()) samples.push_back(&p);
    for (const auto& patch : boundary.patches) {
        if (!patch.planar || patch.pointIds.size() < points.size() * .98) continue;
        const auto face = planePatch(patch,samples,tolerance,report);
        if (!face.IsNull()) return face;
    }
    return {};
}

struct RingSample {
    reconstruction::PointId id;
    double height, radius, angle;
};

TopoDS_Face annularFace(const gp_Ax3& frame, double height, double inner, double outer, int sign) {
    const auto center = frame.Location().Translated(gp_Vec(frame.Direction())*height);
    const gp_Dir normal = sign>0 ? frame.Direction() : frame.Direction().Reversed();
    const gp_Ax2 circleFrame(center,normal,frame.XDirection());
    const auto rim = [&](double radius) { return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(circleFrame,radius))).Wire(); };
    BRepBuilderAPI_MakeFace face(gp_Pln(center,normal),rim(outer),true);
    if (!face.IsDone()) return {};
    if (inner>0) face.Add(TopoDS::Wire(rim(inner).Reversed()));
    return face.IsDone() && oneValidFace(face.Face()) ? face.Face() : TopoDS_Face{};
}

bool coaxial(const reconstruction::CylinderEquation& a, const reconstruction::CylinderEquation& b, double tolerance) {
    const gp_Vec x(a.axisDirection.x,a.axisDirection.y,a.axisDirection.z);
    const gp_Vec y(b.axisDirection.x,b.axisDirection.y,b.axisDirection.z);
    const gp_Vec d(a.axisPoint.x-b.axisPoint.x,a.axisPoint.y-b.axisPoint.y,a.axisPoint.z-b.axisPoint.z);
    return std::abs(x.Dot(y))>.99999 && d.Crossed(x).Magnitude()<=tolerance;
}

TopoDS_Shape fitCoaxialFaces(const reconstruction::PointStore& points,
                            const std::vector<reconstruction::CylinderEquation>& proposals,
                            double tolerance, QJsonObject& report) {
    std::vector<reconstruction::CylinderEquation> tried;
    for (const auto& seed : proposals) {
        if (std::any_of(tried.begin(),tried.end(),[&](const auto& old) { return coaxial(old,seed,tolerance); })) continue;
        tried.push_back(seed);
        std::vector<double> radii;
        for (const auto& candidate : proposals) {
            if (!coaxial(seed,candidate,tolerance) || std::any_of(radii.begin(),radii.end(),[&](double r) {
                    return std::abs(r-candidate.radius)<=2*tolerance;
                })) continue;
            radii.push_back(candidate.radius);
        }
        if (radii.size()<2) continue;

        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        QJsonArray faces;
        std::unordered_set<reconstruction::PointId> assigned;
        std::vector<AxialFit> walls;
        for (double radius : radii) {
            const reconstruction::PrimitiveEvidence fitted = reconstruction::CylinderEvidence{
                0,{seed.axisPoint,seed.axisDirection,radius}};
            std::vector<reconstruction::PointSample> samples;
            for (const auto& p : points.points()) {
                if (p.normal && p.normalOrientationKnown && reconstruction::surfaceResidual(fitted,p.position)<=tolerance &&
                    reconstruction::surfaceNormalAlignment(fitted,p.position,*p.normal)>=.95) samples.push_back(p);
            }
            const reconstruction::PointStore cloud(std::move(samples));
            const auto wall = measure(cloud,fitted,tolerance);
            if (!wall || !wall->gridSupported || !wall->normalDirection || wall->angularSpan<fullTurn*.98) continue;
            BRepBuilderAPI_MakeFace face(gp_Cylinder(wall->frame,wall->radius),wall->startAngle,
                wall->startAngle+wall->angularSpan,wall->low,wall->high);
            if (!face.IsDone() || !oneValidFace(face.Face())) continue;
            auto surface = face.Face();
            if (wall->normalDirection<0) surface.Reverse();
            builder.Add(compound,surface);
            for (const auto& p : cloud.points()) {
                const auto& v = p.position;
                const double z = gp_Vec(wall->frame.Location(),gp_Pnt(v.x,v.y,v.z)).Dot(gp_Vec(wall->frame.Direction()));
                if (z>=wall->low-tolerance && z<=wall->high+tolerance) assigned.insert(p.id);
            }
            faces.append(QJsonObject{{"kind","cylinder"},{"radius_mm",radius},
                {"axial_bounds_mm",QJsonArray{wall->low,wall->high}}, {"normal_sign",wall->normalDirection}});
            walls.push_back(*wall);
        }
        if (walls.size()<2) continue;

        const auto& frame = walls.front().frame;
        const gp_Vec axis(frame.Direction()), x(frame.XDirection()), y(frame.YDirection());
        std::vector<RingSample> axial[2];
        for (const auto& p : points.points()) {
            if (!p.normal || !p.normalOrientationKnown || assigned.count(p.id)) continue;
            const double alignment = gp_Vec(p.normal->x,p.normal->y,p.normal->z).Dot(axis);
            if (std::abs(alignment)<.98) continue;
            const auto& v = p.position;
            const gp_Vec d(frame.Location(),gp_Pnt(v.x,v.y,v.z));
            const double u = d.Dot(x), w = d.Dot(y);
            axial[alignment>0].push_back({p.id,d.Dot(axis),std::hypot(u,w),std::atan2(w,u)});
        }
        for (int sign : {-1,1}) {
            auto& rows = axial[sign>0];
            std::sort(rows.begin(),rows.end(),[](auto a, auto b) { return a.height<b.height; });
            for (auto first=rows.begin(); first!=rows.end();) {
                const auto next = std::upper_bound(first,rows.end(),first->height+2*tolerance,
                    [](double height,const auto& p) { return height<p.height; });
                if (next-first>=96) {
                    // Each interval must independently form a supported annulus.
                    // A radial gap can separate two disjoint rims at one level.
                    double height = 0;
                    for (auto p=first; p!=next; ++p) height += p->height;
                    height /= next-first;
                    std::sort(first,next,[](auto a, auto b) { return a.radius<b.radius; });
                    const double gap = std::max(4*tolerance,((next-1)->radius-first->radius)*.08);
                    for (auto inner=first; inner!=next;) {
                        auto outer=inner+1;
                        while (outer!=next && outer->radius-(outer-1)->radius<=gap) ++outer;
                        double lo=inner->radius, hi=(outer-1)->radius;
                        if (outer-inner>=96 && lo>2*tolerance && hi-lo>4*tolerance) {
                            const double reach = std::max(2*tolerance,(hi-lo)/24);
                            std::optional<double> innerRim, outerRim;
                            for (const auto& wall : walls) {
                                if (std::min(std::abs(wall.low-height),std::abs(wall.high-height))>
                                    std::max(2*tolerance,(wall.high-wall.low)*.01)) continue;
                                if (std::abs(lo-wall.radius)<=reach && (!innerRim ||
                                    std::abs(lo-wall.radius)<std::abs(lo-*innerRim))) innerRim=wall.radius;
                                if (std::abs(hi-wall.radius)<=reach && (!outerRim ||
                                    std::abs(hi-wall.radius)<std::abs(hi-*outerRim))) outerRim=wall.radius;
                            }
                            lo = innerRim.value_or(lo);
                            hi = outerRim.value_or(hi);
                            constexpr int sectors=16, layers=4;
                            std::bitset<sectors*layers> bins;
                            std::bitset<sectors> around;
                            const auto accepted = [&](const RingSample& p) {
                                return std::abs(p.height-height)<=tolerance && p.radius>=lo-tolerance && p.radius<=hi+tolerance;
                            };
                            if ((innerRim || outerRim) && hi-lo>4*tolerance) for (auto p=inner; p!=outer; ++p) {
                                if (!accepted(*p)) continue;
                                const int u = std::min(sectors-1,int((p->angle+pi)*sectors/fullTurn));
                                const double fraction = (p->radius-lo)/(hi-lo)*(p->radius+lo)/(hi+lo);
                                const int v = std::clamp(int(fraction*layers),0,layers-1);
                                bins.set(u*layers+v);
                                around.set(u);
                            }
                            if (around.all() && bins.count()>=sectors*layers*.92) {
                                const auto face = annularFace(frame,height,lo,hi,sign);
                                if (!face.IsNull()) {
                                    builder.Add(compound,face);
                                    for (auto p=inner; p!=outer; ++p) if (accepted(*p)) assigned.insert(p->id);
                                    faces.append(QJsonObject{{"kind","annulus"},{"height_mm",height},
                                        {"inner_radius_mm",lo},{"outer_radius_mm",hi},{"normal_sign",sign}});
                                }
                            }
                        }
                        inner=outer;
                    }
                }
                first=next;
            }
        }
        if (faces.size()<=qsizetype(walls.size()) || assigned.size()<points.size()*.98 || validFaceCount(compound)!=faces.size()) continue;
        const auto& p = frame.Location();
        report["kind"] = "coaxial";
        report["axis_origin_mm"] = QJsonArray{p.X(),p.Y(),p.Z()};
        report["axis_direction"] = QJsonArray{axis.X(),axis.Y(),axis.Z()};
        report["faces"] = faces;
        report["surface_coverage"] = double(assigned.size())/points.size();
        report["unassigned_points"] = double(points.size()-assigned.size());
        return compound;
    }
    return {};
}

void describeObservedPatch(QJsonObject& evidence, const AxialFit& fit) {
    const auto& origin = fit.frame.Location();
    const auto& axis = fit.frame.Direction();
    evidence["kind"] = fit.angle>0 ? "cone" : "cylinder";
    evidence["axis_origin_mm"] = QJsonArray{origin.X(),origin.Y(),origin.Z()};
    evidence["axis_direction"] = QJsonArray{axis.X(),axis.Y(),axis.Z()};
    evidence["normal_sign"] = fit.normalDirection;
    if (!evidence.contains("axial_bounds_mm")) evidence["axial_bounds_mm"] = QJsonArray{fit.low,fit.high};
    if (!evidence.contains("angular_span_rad")) evidence["angular_span_rad"] = fit.angularSpan;
    if (fit.angle>0) evidence["semi_angle_rad"] = fit.angle;
    else evidence["radius_mm"] = fit.radius;
}

std::vector<ObservedPatch> fitAxialBand(const reconstruction::PointStore& samples,
                                    const reconstruction::PrimitiveEvidence& proposal,
                                    double tolerance, std::size_t minimumSamples, bool preciseCylinder,
                                    bool refineTransitions, bool& preciseEvidence) {
    const auto fit = measure(samples,proposal,tolerance,minimumSamples,preciseCylinder);
    if (!fit || !fit->normalDirection) return {};
    const bool cylinder = std::holds_alternative<reconstruction::CylinderEvidence>(proposal);
    if (cylinder && fit->angularSpan<pi/4 && fit->high-fit->low<fit->radius) {
        // On a short arc, proximity alone can make a freeform transition
        // look cylindrical. The observed normals must also share an axis.
        const gp_Vec axis(fit->frame.Direction());
        const auto transverse = std::count_if(samples.points().begin(),samples.points().end(),[&](const auto& point) {
            const auto& n = *point.normal;
            return std::abs(axis.Dot(gp_Vec(n.x,n.y,n.z)))<=.01;
        });
        if (transverse<samples.size()*.85) return {};
    }
    const gp_Vec candidateAxis(fit->frame.Direction());
    const double expectedNormal = -std::sin(fit->angle)*fit->normalDirection;
    const auto exactSupport = std::count_if(samples.points().begin(),samples.points().end(),[&](const auto& point) {
        const auto& n = *point.normal;
        return std::abs(candidateAxis.Dot(gp_Vec(n.x,n.y,n.z))-expectedNormal)<=.00015 &&
            reconstruction::surfaceResidual(proposal,point.position)<=tolerance*.08;
    });
    preciseEvidence = exactSupport>=samples.size()*.85;
    if (refineTransitions && !preciseEvidence) return {};
    std::vector<ObservedPatch> patches;
    const bool useGrid = !refineTransitions || fit->angularSpan>=.9*fullTurn;
    if (useGrid && (fit->gridSupported || fit->periodicGridSupported)) {
        const auto end = fit->startAngle+fit->angularSpan;
        auto bounded = cylinder
            ? BRepBuilderAPI_MakeFace(gp_Cylinder(fit->frame,fit->radius),fit->startAngle,end,fit->low,fit->high)
            : BRepBuilderAPI_MakeFace(gp_Cone(fit->frame,fit->angle,0),fit->startAngle,end,
                fit->low/std::cos(fit->angle),fit->high/std::cos(fit->angle));
        if (!bounded.IsDone() || !oneValidFace(bounded.Face())) return {};
        ObservedPatch patch{bounded.Face(),{}, {}};
        for (const auto& p : samples.points()) {
            const auto& q = p.position;
            const double z = gp_Vec(fit->frame.Location(),gp_Pnt(q.x,q.y,q.z)).Dot(gp_Vec(fit->frame.Direction()));
            if (z>=fit->low-tolerance && z<=fit->high+tolerance) patch.points.insert(p.id);
        }
        patches.push_back(std::move(patch));
    } else {
        patches = trimAxialSurfaces(samples,proposal,*fit,tolerance,.90,minimumSamples);
    }
    for (auto& patch : patches) {
        if (fit->normalDirection<0) patch.face.Reverse();
        describeObservedPatch(patch.report,*fit);
    }
    return patches;
}

void verifyRemainingObservations(const TopoDS_Shape& shape, const reconstruction::PointStore& points,
                                 double tolerance, PointIds& assigned) {
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
        const auto face = TopoDS::Face(it.Current());
        const BRepAdaptor_Surface surface(face);
        const auto kind = surface.GetType();
        const bool planar = kind==GeomAbs_Plane;
        const bool cylinder = kind==GeomAbs_Cylinder;
        const gp_Pln plane = planar ? surface.Plane() : gp_Pln{};
        const gp_Cylinder tube = cylinder ? surface.Cylinder() : gp_Cylinder{};
        const gp_Cone cone = kind==GeomAbs_Cone ? surface.Cone() : gp_Cone{};
        if (!planar && !cylinder && kind!=GeomAbs_Cone) continue;
        const auto origin = planar ? plane.Location() : (cylinder ? tube.Location() : cone.Apex());
        const gp_Vec axis(planar ? plane.Axis().Direction() : (cylinder ? tube.Axis().Direction() : cone.Axis().Direction()));
        const double angle = kind==GeomAbs_Cone ? cone.SemiAngle() : 0;
        const double sign = face.Orientation()==TopAbs_REVERSED ? -1 : 1;
        BRepClass_FaceClassifier classifier;
        for (const auto& p : points.points()) {
            if (assigned.count(p.id) || !p.normal || !p.normalOrientationKnown) continue;
            const gp_Pnt position(p.position.x,p.position.y,p.position.z);
            const gp_Vec direction(p.normal->x,p.normal->y,p.normal->z);
            const gp_Vec d(origin,position);
            const double axial = d.Dot(axis);
            double residual = std::abs(axial);
            gp_Vec normal = axis;
            if (!planar) {
                const gp_Vec radial = d-axis*axial;
                const double r = radial.Magnitude();
                if (r<=1e-12) continue;
                if (cylinder) {
                    residual = std::abs(r-tube.Radius());
                    normal = radial/r;
                } else {
                    if (axial*angle<=0) continue;
                    residual = std::abs(r*std::cos(angle)-axial*std::sin(angle));
                    normal = (radial/r)*std::cos(angle)-axis*std::sin(angle);
                }
            }
            if (residual>tolerance || direction.Dot(normal)*sign<.999) continue;
            classifier.Perform(face,position,tolerance);
            if (classifier.State()==TopAbs_IN || classifier.State()==TopAbs_ON) assigned.insert(p.id);
        }
    }
}

std::vector<reconstruction::PrimitiveEvidence> signedCylinderProposals(const reconstruction::PointStore& points,
        double tolerance, std::size_t minimumSamples, bool precise = false) {
    struct Observation { reconstruction::PointId id; gp_Vec position, normal; };
    struct Cylinder { gp_Vec center, axis; double radius; };
    std::vector<Observation> remaining;
    for (const auto& p : points.points()) {
        if (p.normal && p.normalOrientationKnown)
            remaining.push_back({p.id,{p.position.x,p.position.y,p.position.z},{p.normal->x,p.normal->y,p.normal->z}});
    }
    const auto matches = [&](const Cylinder& c, const Observation& p) {
        const auto d = p.position-c.center;
        const auto radial = d-c.axis*d.Dot(c.axis);
        const double r = radial.Magnitude();
        return r>tolerance && std::abs(r-std::abs(c.radius))<=(precise ? tolerance*.12 : tolerance) &&
            radial.Dot(p.normal)*std::copysign(1.0,c.radius)>=(precise ? .99999 : .999)*r;
    };
    std::mt19937 random(4500);
    std::vector<reconstruction::PrimitiveEvidence> candidates;
    while (remaining.size()>=minimumSamples && candidates.size()<12) {
        const auto stride = std::max<std::size_t>(1,(remaining.size()+4095)/4096);
        std::uniform_int_distribution<std::size_t> choose(0,remaining.size()-1);
        std::size_t bestVotes = std::max<std::size_t>(8,minimumSamples/stride)-1;
        std::optional<Cylinder> best;
        for (int trial=0; trial<(precise ? 512 : 192); ++trial) {
            const auto& a = remaining[choose(random)];
            const auto& b = remaining[choose(random)];
            auto axis = a.normal.Crossed(b.normal);
            if (axis.SquareMagnitude()<(precise ? .004 : .0225)) continue;
            axis.Normalize();
            const auto deltaNormal = a.normal-b.normal;
            const auto deltaPosition = a.position-b.position;
            // Signed normals of one cylinder differ only in its transverse
            // plane. The axial separation of the samples is irrelevant.
            const double radius = deltaPosition.Dot(deltaNormal)/deltaNormal.SquareMagnitude();
            if (!std::isfinite(radius) || std::abs(radius)<=4*tolerance) continue;
            const auto center = a.position-a.normal*radius;
            const Cylinder candidate{center-axis*center.Dot(axis),axis,radius};
            std::size_t votes=0;
            for (std::size_t i=0; i<remaining.size(); i+=stride) votes += matches(candidate,remaining[i]);
            if (votes>bestVotes) { bestVotes=votes; best=candidate; }
        }
        if (!best) break;
        reconstruction::CylinderEvidence evidence;
        const auto& p = best->center;
        const auto& axis = best->axis;
        evidence.cylinder = {{p.X(),p.Y(),p.Z()},{axis.X(),axis.Y(),axis.Z()},std::abs(best->radius)};
        auto end = std::remove_if(remaining.begin(),remaining.end(),[&](const auto& sample) {
            if (!matches(*best,sample)) return false;
            evidence.supportPointIds.push_back(sample.id);
            return true;
        });
        if (evidence.supportPointIds.size()<minimumSamples) break;
        remaining.erase(end,remaining.end());
        candidates.push_back(std::move(evidence));
    }
    return candidates;
}

struct AngularInterval {
    double start, span;

    double fraction(double angle) const {
        return (angle>=start ? angle-start : angle+fullTurn-start)/span;
    }
};

AngularInterval observedAngularInterval(std::vector<double> angles) {
    std::sort(angles.begin(),angles.end());
    double largest = angles.front()+fullTurn-angles.back();
    double start = angles.front();
    for (std::size_t i=1; i<angles.size(); ++i) {
        const double gap = angles[i]-angles[i-1];
        if (gap<=largest) continue;
        largest = gap;
        start = angles[i];
    }
    return largest>fullTurn/72 ? AngularInterval{start,fullTurn-largest} : AngularInterval{0,fullTurn};
}

TopoDS_Face fitToroidalPatch(const reconstruction::TorusEquation& fit, const reconstruction::PointStore& residual,
                            double tolerance, PointIds& assigned, QJsonObject& report) {
    if (fit.minorRadius<=4*tolerance || fit.majorRadius<=fit.minorRadius+tolerance) return {};
    const auto& c=fit.center;
    const auto& a=fit.axisDirection;
    const gp_Ax3 frame(gp_Pnt(c.x,c.y,c.z),gp_Dir(a.x,a.y,a.z));
    const gp_Vec axis(frame.Direction()), x(frame.XDirection()), y(frame.YDirection());
    struct Hit { reconstruction::PointId id; double u, v; int direction; };
    std::vector<Hit> hits;
    std::vector<double> anglesU, anglesV;
    std::size_t outward = 0;
    for (const auto& p : residual.points()) {
        if (assigned.count(p.id) || !p.normal || !p.normalOrientationKnown) continue;
        const auto& q=p.position;
        const gp_Vec d(frame.Location(),gp_Pnt(q.x,q.y,q.z));
        const double z=d.Dot(axis);
        const auto radial=d-axis*z;
        const double r=radial.Magnitude();
        if (r<=tolerance) continue;
        const auto tube=(radial/r)*(r-fit.majorRadius)+axis*z;
        const double radius=tube.Magnitude();
        if (std::abs(radius-fit.minorRadius)>tolerance || radius<=tolerance) continue;
        const double alignment=tube.Dot(gp_Vec(p.normal->x,p.normal->y,p.normal->z))/radius;
        if (std::abs(alignment)<.999) continue;
        double u=std::atan2(d.Dot(y),d.Dot(x)), v=std::atan2(z,r-fit.majorRadius);
        if (u<0) u+=fullTurn;
        if (v<0) v+=fullTurn;
        hits.push_back({p.id,u,v,alignment>0 ? 1 : -1});
        anglesU.push_back(u);
        anglesV.push_back(v);
        outward+=alignment>0;
    }
    if (hits.size()<48 || std::max(outward,hits.size()-outward)<hits.size()*.98) return {};
    const auto u=observedAngularInterval(std::move(anglesU)), v=observedAngularInterval(std::move(anglesV));
    if (u.span*(fit.majorRadius-fit.minorRadius)<=4*tolerance || v.span*fit.minorRadius<=4*tolerance) return {};
    const int columns=hits.size()>=160 ? 8 : 6, rows=hits.size()>=160 ? 5 : 3;
    std::bitset<40> bins;
    for (const auto& hit : hits) {
        const auto i=std::clamp(int(u.fraction(hit.u)*columns),0,columns-1);
        const auto j=std::clamp(int(v.fraction(hit.v)*rows),0,rows-1);
        bins.set(i*rows+j);
    }
    if (bins.count()<columns*rows*.92) return {};
    BRepBuilderAPI_MakeFace built(gp_Torus(frame,fit.majorRadius,fit.minorRadius),
                                 u.start,u.start+u.span,v.start,v.start+v.span);
    if (!built.IsDone() || !oneValidFace(built.Face())) return {};
    auto face=built.Face();
    const int sign=outward>=hits.size()*.98 ? 1 : -1;
    if (sign<0) face.Reverse();
    for (const auto& hit : hits) if (hit.direction==sign) assigned.insert(hit.id);
    report = QJsonObject{{"kind","torus"},{"center_mm",QJsonArray{c.x,c.y,c.z}},
        {"axis_direction",QJsonArray{a.x,a.y,a.z}},{"major_radius_mm",fit.majorRadius},{"minor_radius_mm",fit.minorRadius},
        {"u_span_rad",u.span},{"v_span_rad",v.span},{"normal_sign",sign}};
    return face;
}

bool normalsVaryWithPosition(const reconstruction::FiniteSurfacePatch& patch,
                             const std::vector<const reconstruction::PointSample*>& samples) {
    const auto& origin = patch.origin;
    const auto& normal = patch.normal;
    const gp_Ax3 frame(gp_Pnt(origin.x,origin.y,origin.z),gp_Dir(normal.x,normal.y,normal.z));
    const gp_Vec x(frame.XDirection()), y(frame.YDirection());
    double sumX = 0, sumY = 0, xx = 0, xy = 0, yy = 0, normalSquares = 0;
    gp_Vec sumNormal, xNormal, yNormal;
    for (const auto* point : samples) {
        const auto& p = point->position;
        const auto& n = *point->normal;
        const gp_Vec offset(frame.Location(),gp_Pnt(p.x,p.y,p.z));
        const gp_Vec direction(n.x-normal.x,n.y-normal.y,n.z-normal.z);
        const double u = offset.Dot(x), v = offset.Dot(y);
        sumX += u;
        sumY += v;
        xx += u*u;
        xy += u*v;
        yy += v*v;
        sumNormal += direction;
        xNormal += direction*u;
        yNormal += direction*v;
        normalSquares += direction.SquareMagnitude();
    }
    const double count = double(samples.size());
    xx -= sumX*sumX/count;
    xy -= sumX*sumY/count;
    yy -= sumY*sumY/count;
    xNormal -= sumNormal*(sumX/count);
    yNormal -= sumNormal*(sumY/count);
    const double variance = normalSquares-sumNormal.SquareMagnitude()/count;
    const double determinant = xx*yy-xy*xy;
    if (variance<=count*.008*.008 || determinant<=1e-8*xx*yy) return false;
    const double explained = (yy*xNormal.SquareMagnitude()-2*xy*xNormal.Dot(yNormal)+
                              xx*yNormal.SquareMagnitude())/determinant;
    return explained>=.85*variance && explained>=count*.008*.008;
}

struct SplineBasis {
    Eigen::VectorXd value, derivative;
};

SplineBasis bernsteinBasis(int degree, double parameter) {
    Eigen::VectorXd values = Eigen::VectorXd::Zero(degree+1);
    values[0] = 1;
    for (int order=1; order<degree; ++order)
        for (int i=order; i>=0; --i)
            values[i] = (1-parameter)*values[i]+(i ? parameter*values[i-1] : 0);
    Eigen::VectorXd derivative(degree+1);
    for (int i=0; i<=degree; ++i)
        derivative[i] = degree*((i ? values[i-1] : 0)-values[i]);
    for (int i=degree; i>=0; --i)
        values[i] = (1-parameter)*values[i]+(i ? parameter*values[i-1] : 0);
    return {std::move(values),std::move(derivative)};
}

struct SplineObservation {
    reconstruction::PointId id;
    double x, y, height;
    gp_Vec normal;
    bool heldOut;
};

std::vector<ObservedPatch> fitSplineCluster(const std::vector<const reconstruction::PointSample*>& cluster,
                                        double spacing, double tolerance, bool refineTransitions) {
    constexpr std::size_t minimumSamples = 64;
    gp_XYZ mean;
    gp_Vec averageNormal;
    for (const auto* point : cluster) {
        const auto& p=point->position;
        const auto& n=*point->normal;
        mean += gp_XYZ(p.x,p.y,p.z);
        averageNormal += gp_Vec(n.x,n.y,n.z);
    }
    if (averageNormal.Magnitude()<cluster.size()*.8) return {};
    const gp_Ax3 frame(gp_Pnt(mean/double(cluster.size())),gp_Dir(averageNormal));
    const gp_Vec x(frame.XDirection()), y(frame.YDirection()), z(frame.Direction());
    std::vector<SplineObservation> samples;
    double xmin=0, xmax=0, ymin=0, ymax=0, maxHeight=0, normalVariance=0;
    for (const auto* point : cluster) {
        const auto& p=point->position;
        const auto& n=*point->normal;
        const gp_Vec offset(frame.Location(),gp_Pnt(p.x,p.y,p.z));
        const gp_Vec normal(n.x,n.y,n.z);
        const gp_Vec local(normal.Dot(x),normal.Dot(y),normal.Dot(z));
        if (local.Z()<.8) return {};
        const double u=offset.Dot(x), v=offset.Dot(y), h=offset.Dot(z);
        xmin=std::min(xmin,u); xmax=std::max(xmax,u);
        ymin=std::min(ymin,v); ymax=std::max(ymax,v);
        maxHeight=std::max(maxHeight,std::abs(h));
        normalVariance += local.X()*local.X()+local.Y()*local.Y();
        const auto hash=std::uint64_t(point->id)*11400714819323198485ull;
        samples.push_back({point->id,u,v,h,local,(hash>>32)%5==0});
    }
    const double width=xmax-xmin, height=ymax-ymin;
    if (std::min(width,height)<(refineTransitions ? 2.5 : 4)*spacing || maxHeight<tolerance ||
        normalVariance<samples.size()*.003*.003) return {};

    Eigen::MatrixXd poles;
    std::vector<reconstruction::PointSample> chart;
    double holdoutCoverage=0, maximumResidual=0;
    for (int degree=2; degree<=5; ++degree) {
        const int order=degree+1, terms=order*order;
        const auto heldOut=std::count_if(samples.begin(),samples.end(),[](const auto& p) { return p.heldOut; });
        if (heldOut<10 || samples.size()-heldOut<std::size_t(terms*2)) break;
        const int training=std::min<int>(4096,int(samples.size()-heldOut));
        Eigen::MatrixXd design(training*3,terms);
        Eigen::VectorXd target(training*3);
        const double normalWeight=std::min(spacing,std::min(width,height)*.1);
        int row=0;
        for (const auto& point : samples) {
            if (point.heldOut || row==training*3) continue;
            const auto u=bernsteinBasis(degree,(point.x-xmin)/width);
            const auto v=bernsteinBasis(degree,(point.y-ymin)/height);
            for (int i=0; i<order; ++i) for (int j=0; j<order; ++j) {
                design(row,i*order+j) = u.value[i]*v.value[j];
                design(row+1,i*order+j) = normalWeight*u.derivative[i]*v.value[j]/width;
                design(row+2,i*order+j) = normalWeight*u.value[i]*v.derivative[j]/height;
            }
            target[row++] = point.height;
            target[row++] = -normalWeight*point.normal.X()/point.normal.Z();
            target[row++] = -normalWeight*point.normal.Y()/point.normal.Z();
        }
        auto solver=design.colPivHouseholderQr();
        solver.setThreshold(1e-8);
        if (solver.rank()!=terms) continue;
        const Eigen::VectorXd coefficients=solver.solve(target);
        if (!coefficients.allFinite() || coefficients.cwiseAbs().maxCoeff()>8*maxHeight) continue;
        const Eigen::MatrixXd heights=Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>(
            coefficients.data(),order,order);
        std::vector<reconstruction::PointSample> inliers;
        std::size_t holdoutInliers=0;
        double residual=0;
        for (const auto& point : samples) {
            const auto u=bernsteinBasis(degree,(point.x-xmin)/width);
            const auto v=bernsteinBasis(degree,(point.y-ymin)/height);
            const double predicted=u.value.dot(heights*v.value);
            const double dx=u.derivative.dot(heights*v.value)/width;
            const double dy=u.value.dot(heights*v.derivative)/height;
            const gp_Vec normal(-dx,-dy,1);
            const double error=std::abs(predicted-point.height);
            if (error>tolerance*.7 || normal.Dot(point.normal)<.999*normal.Magnitude()) continue;
            holdoutInliers += point.heldOut;
            residual=std::max(residual,error);
            inliers.push_back({point.id,{point.x-xmin,point.y-ymin,0},modeling::Vec3{0,0,1},1,std::nullopt,true});
        }
        if (holdoutInliers<heldOut*.95 || inliers.size()<samples.size()*.95) continue;
        if (chart.empty() || (inliers.size()>=chart.size()*.98 && residual<maximumResidual)) {
            chart=std::move(inliers);
            poles=heights;
            holdoutCoverage=double(holdoutInliers)/heldOut;
            maximumResidual=residual;
        }
        if (maximumResidual<=tolerance*.5) break;
    }
    if (chart.empty()) return {};

    const int order=int(poles.rows());
    NCollection_Array2<gp_Pnt> control(1,order,1,order);
    for (int i=0; i<order; ++i) for (int j=0; j<order; ++j)
        control.SetValue(i+1,j+1,frame.Location().Translated(x*(xmin+width*i/(order-1))+
            y*(ymin+height*j/(order-1))+z*poles(i,j)));
    NCollection_Array1<double> knots(1,2);
    knots.SetValue(1,0); knots.SetValue(2,1);
    NCollection_Array1<int> multiplicity(1,2);
    multiplicity.Init(order);
    const Handle(Geom_Surface) surface = new Geom_BSplineSurface(control,knots,knots,multiplicity,multiplicity,
                                                                  order-1,order-1);
    reconstruction::AssemblyOptions options;
    options.buildSolids = false;
    options.distanceTolerance = tolerance;
    options.alphaFactor = 2.1;
    reconstruction::AssemblyReconstructionResult boundary;
    const reconstruction::PointStore chartPoints(std::move(chart));
    std::string error;
    if (!reconstruction::reconstructAssembly(chartPoints,options,boundary,error)) return {};
    const auto uv = [&](const modeling::Vec3& point) { return gp_Pnt2d(point.x/width,point.y/height); };
    std::vector<ObservedPatch> fitted;
    for (const auto& patch : boundary.patches) {
        if (!patch.planar || patch.pointIds.size()<minimumSamples) continue;
        const PointIds ids(patch.pointIds.begin(),patch.pointIds.end());
        const auto add = [&](const auto& loops) {
            const auto face=boundedParametricFace(surface,loops,uv);
            if (face.IsNull()) return;
            PointIds supported;
            BRepClass_FaceClassifier classifier;
            const double trimTolerance=std::max(tolerance,patch.spacing*.5)/std::max(width,height);
            for (const auto& point : chartPoints.points()) {
                if (!ids.count(point.id)) continue;
                classifier.Perform(face,uv(point.position),trimTolerance);
                if (classifier.State()==TopAbs_IN || classifier.State()==TopAbs_ON) supported.insert(point.id);
            }
            if (supported.size()<minimumSamples) return;
            QJsonObject report{{"kind","bspline"},{"degree",order-1},{"support_points",double(supported.size())},
                {"holdout_coverage",holdoutCoverage},{"maximum_height_error_mm",maximumResidual},
                {"inner_loops",double(loops.size()-1)}};
            fitted.push_back({face,std::move(report),std::move(supported)});
        };
        const auto islands=splitPlaneIslands(patch);
        if (islands.empty()) add(patch.loops);
        else for (const auto& island : islands) add(island.loops);
    }
    return fitted;
}

std::vector<ObservedPatch> fitResidualSplines(const std::vector<reconstruction::PointSample>& samples,
                                           double tolerance, bool refineTransitions) {
    constexpr std::size_t minimumSamples = 64;
    if (samples.size()<minimumSamples) return {};
    const reconstruction::detail::preprocessing::KdTree tree(samples);
    std::vector<double> spacings;
    spacings.reserve(samples.size());
    for (std::size_t i=0; i<samples.size(); ++i)
        spacings.push_back(std::sqrt(tree.nearest(i,2).back().first));
    const auto middle=spacings.begin()+spacings.size()/2;
    std::nth_element(spacings.begin(),middle,spacings.end());
    const double spacing=std::max(tolerance,*middle);
    const double connection=20.25*spacing*spacing;
    const auto support=[](const std::vector<ObservedPatch>& faces) {
        PointIds ids;
        for (const auto& face : faces) ids.insert(face.points.begin(),face.points.end());
        return ids.size();
    };
    const auto fitRegion=[&](auto&& fit, std::vector<const reconstruction::PointSample*> cluster,
                             int depth) -> std::vector<ObservedPatch> {
        auto whole=fitSplineCluster(cluster,spacing,tolerance,refineTransitions);
        const auto wholeSupport=support(whole);
        if (!depth || cluster.size()<2*minimumSamples || wholeSupport>=cluster.size()*.95) return whole;
        const auto coordinates=[](const auto* point) {
            const auto& p=point->position;
            return std::array<double,3>{p.x,p.y,p.z};
        };
        auto low=coordinates(cluster.front()), high=low;
        for (const auto* point : cluster) {
            const auto value=coordinates(point);
            for (int i=0; i<3; ++i) {
                low[i]=std::min(low[i],value[i]);
                high[i]=std::max(high[i],value[i]);
            }
        }
        int axis=0;
        for (int i=1; i<3; ++i) if (high[i]-low[i]>high[axis]-low[axis]) axis=i;
        const auto median=cluster.begin()+cluster.size()/2;
        std::nth_element(cluster.begin(),median,cluster.end(),[&](const auto* a,const auto* b) {
            return std::make_pair(coordinates(a)[axis],a->id)<std::make_pair(coordinates(b)[axis],b->id);
        });
        auto first=fit(fit,{cluster.begin(),median},depth-1);
        auto second=fit(fit,{median,cluster.end()},depth-1);
        if (support(first)+support(second)<=wholeSupport) return whole;
        first.insert(first.end(),std::make_move_iterator(second.begin()),std::make_move_iterator(second.end()));
        return first;
    };
    std::vector<bool> visited(samples.size());
    std::vector<ObservedPatch> patches;
    for (std::size_t i=0; i<samples.size(); ++i) {
        if (visited[i]) continue;
        visited[i]=true;
        std::vector<std::size_t> group{i};
        for (std::size_t j=0; j<group.size(); ++j) {
            const auto& a=*samples[group[j]].normal;
            for (const auto& [distance,k] : tree.nearest(group[j],32)) {
                if (distance>connection) break;
                const auto& b=*samples[k].normal;
                if (visited[k] || a.x*b.x+a.y*b.y+a.z*b.z<.9) continue;
                visited[k]=true;
                group.push_back(k);
            }
        }
        if (group.size()<minimumSamples) continue;
        std::vector<const reconstruction::PointSample*> cluster;
        for (auto index : group) cluster.push_back(&samples[index]);
        auto fitted=fitRegion(fitRegion,std::move(cluster),refineTransitions ? 4 : 2);
        patches.insert(patches.end(),std::make_move_iterator(fitted.begin()),std::make_move_iterator(fitted.end()));
    }
    return patches;
}


TopoDS_Shape buildMixedFaces(const reconstruction::PointStore& points,
                             reconstruction::PrimitiveDetectionOptions options, double tolerance,
                             bool refineTransitions, QJsonObject& report) {
    if (points.size()<400) return {};
    constexpr int maximumFaces = 128;
    PointIds assigned;
    int weakAnalyticFaces = 0;
    QJsonArray faces;
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    std::unordered_map<reconstruction::PointId,const reconstruction::PointSample*> byId;
    byId.reserve(points.size());
    for (const auto& p : points.points()) byId.emplace(p.id,&p);
    std::vector<reconstruction::PrimitiveEvidence> observedCylinders;
    const auto appendObservedPatches = [&](const reconstruction::PointStore& residual,
                                        const reconstruction::PrimitiveEvidence& proposal,
                                        std::size_t minimumSamples, std::size_t minimumBandSamples,
                                        bool preciseCylinder = false) {
        const bool cylinders = std::holds_alternative<reconstruction::CylinderEvidence>(proposal);
        const auto axis = cylinders ? std::get<reconstruction::CylinderEvidence>(proposal).cylinder.axisDirection
                                    : std::get<reconstruction::ConeEvidence>(proposal).cone.axisDirection;
        const auto height = [&](const auto& p) { return p.position.x*axis.x+p.position.y*axis.y+p.position.z*axis.z; };
        std::vector<reconstruction::PointSample> subset;
        for (const auto& p : residual.points()) {
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id) &&
                reconstruction::surfaceResidual(proposal,p.position)<=(preciseCylinder ? tolerance*.12 : tolerance) &&
                reconstruction::surfaceNormalAlignment(proposal,p.position,*p.normal)>=(preciseCylinder ? .99999 : .999))
                subset.push_back(p);
        }
        if (subset.size()<minimumSamples) return true;
        bool fitted = false;
        std::sort(subset.begin(),subset.end(),[&](const auto& a, const auto& b) { return height(a)<height(b); });
        // Identical infinite surfaces can support several separate walls.
        // Keep observed axial gaps as gaps in the exported geometry.
        std::vector<double> spacing;
        spacing.reserve(subset.size()-1);
        for (std::size_t i=1; i<subset.size(); ++i) spacing.push_back(height(subset[i])-height(subset[i-1]));
        const auto median = spacing.begin()+spacing.size()/2;
        std::nth_element(spacing.begin(),median,spacing.end());
        const double gap = std::max({8*tolerance,(height(subset.back())-height(subset.front()))*.015,8*(*median)});
        for (auto first=subset.begin(); first!=subset.end();) {
            auto next = first+1;
            while (next!=subset.end() && height(*next)-height(*(next-1))<=gap) ++next;
            const auto start = first;
            first = next;
            const auto sampleCount = std::size_t(next-start);
            if (sampleCount<minimumBandSamples) continue;
            if (sampleCount<minimumSamples && !std::all_of(start,next,[&](const auto& point) {
                return reconstruction::surfaceResidual(proposal,point.position)<=tolerance*.25 &&
                    reconstruction::surfaceNormalAlignment(proposal,point.position,*point.normal)>=.9999;
            })) continue;
            const reconstruction::PointStore samples(std::vector<reconstruction::PointSample>(start,next));
            bool preciseEvidence = false;
            for (const auto& patch : fitAxialBand(samples,proposal,tolerance,minimumBandSamples,preciseCylinder,
                                                  refineTransitions,preciseEvidence)) {
                weakAnalyticFaces += !preciseEvidence;
                fitted = true;
                assigned.insert(patch.points.begin(),patch.points.end());
                builder.Add(compound,patch.face);
                faces.append(patch.report);
                if (faces.size()>maximumFaces) return false;
            }
        }
        if (fitted && cylinders && !preciseCylinder) {
            reconstruction::CylinderEvidence observed;
            observed.cylinder = std::get<reconstruction::CylinderEvidence>(proposal).cylinder;
            observedCylinders.emplace_back(std::move(observed));
        }
        return true;
    };
    const auto appendPlanarPatch = [&](const reconstruction::FiniteSurfacePatch& patch, std::size_t minimumSamples) {
        if (!patch.planar || !patch.orientationKnown || patch.pointIds.size()<minimumSamples) return true;
        std::vector<const reconstruction::PointSample*> samples;
        std::size_t signedSamples = 0;
        for (const auto id : patch.pointIds) {
            const auto* p = byId.at(id);
            if (!p->normal || !p->normalOrientationKnown || assigned.count(id)) continue;
            ++signedSamples;
            const auto& n = *p->normal;
            if (n.x*patch.normal.x+n.y*patch.normal.y+n.z*patch.normal.z>=.9995) samples.push_back(p);
        }
        // A narrow strip can lie within a plane's distance tolerance even
        // when its normals rotate along a curved wall.
        if (samples.size()<minimumSamples || samples.size()<signedSamples*.90 ||
            normalsVaryWithPosition(patch,samples)) return true;
        const auto exactSupport = std::count_if(samples.begin(),samples.end(),[&](const auto* point) {
            const auto& n=*point->normal;
            const auto& p=point->position;
            const auto& a=patch.origin;
            const auto& v=patch.normal;
            return std::abs((p.x-a.x)*v.x+(p.y-a.y)*v.y+(p.z-a.z)*v.z)<=tolerance*.06 &&
                std::hypot(n.x-v.x,n.y-v.y,n.z-v.z)<.0008;
        });
        const bool preciseEvidence = exactSupport>=samples.size()*.9;
        if (refineTransitions && !preciseEvidence) return true;
        const auto priorFaces = faces.size();
        QJsonObject evidence;
        const auto face = planePatch(patch,samples,tolerance,evidence,&assigned,.90);
        if (!face.IsNull()) {
            builder.Add(compound,face);
            faces.append(evidence);
        } else {
            for (const auto& island : splitPlaneIslands(patch)) {
                PointIds supported;
                QJsonObject details;
                const auto piece = planePatch(island,samples,tolerance,details,&supported,0);
                for (auto it=supported.begin(); it!=supported.end();) {
                    if (assigned.count(*it)) it=supported.erase(it); else ++it;
                }
                if (piece.IsNull() || supported.size()<minimumSamples) continue;
                assigned.insert(supported.begin(),supported.end());
                builder.Add(compound,piece);
                faces.append(details);
            }
        }
        if (!preciseEvidence) weakAnalyticFaces += int(faces.size()-priorFaces);
        return faces.size()<=maximumFaces;
    };
    for (int pass : {0,1}) {
        const std::size_t minimumSamples = pass==0 ? 80 : 48;
        std::vector<reconstruction::PointSample> remaining;
        if (pass!=0) {
            if (assigned.size()>=points.size()*.98) break;
            for (const auto& p : points.points())
                if (!assigned.count(p.id)) remaining.push_back(p);
            if (remaining.size()<minimumSamples) break;
        }
        const reconstruction::PointStore leftover(std::move(remaining));
        const auto& working = pass==0 ? points : leftover;
        reconstruction::AssemblyOptions boundaryOptions;
        boundaryOptions.buildSolids = false;
        boundaryOptions.distanceTolerance = tolerance;
        reconstruction::AssemblyReconstructionResult boundaries;
        std::string error;
        boundaryOptions.alphaFactor = pass==0 ? 2.5 : 3.2;
        reconstruction::reconstructAssembly(working,boundaryOptions,boundaries,error);

        for (const auto& patch : boundaries.patches)
            if (!appendPlanarPatch(patch,minimumSamples)) return {};
        if (pass==0) report["mixed_planar_coverage"] = double(assigned.size())/points.size();

        options.detectPlanes = options.detectSpheres = options.detectTori = false;
        options.cylinder.maximumCylinders = options.cone.maximumCones = 24;
        options.cylinder.minimumSupportPoints = options.cone.minimumSupportPoints = minimumSamples;
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : working.points()) {
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id)) unmatched.push_back(p);
        }
        const reconstruction::PointStore residual(std::move(unmatched));
        std::vector<reconstruction::PrimitiveEvidence> curved;
        for (const bool cylinders : {true,false}) {
            if (residual.size()<minimumSamples) break;
            options.detectCylinders = cylinders;
            options.detectCones = !cylinders;
            reconstruction::PrimitiveDetectionResult proposals;
            if (!reconstruction::detectPrimitiveEvidence(residual,options,proposals,error)) {
                if (pass!=0 && cylinders) {
                    auto supplemental = signedCylinderProposals(residual,tolerance,minimumSamples);
                    curved.insert(curved.end(),std::make_move_iterator(supplemental.begin()),std::make_move_iterator(supplemental.end()));
                }
                continue;
            }
            for (auto& proposal : proposals.evidence) curved.push_back(std::move(proposal));
        }
        const auto support = [](const auto& proposal) {
            return std::visit([](const auto& e) { return e.supportPointIds.size(); },proposal);
        };
        // A broadly supported cone or cylinder claims its points before a
        // different primitive that happens to approximate a small strip of it.
        std::stable_sort(curved.begin(),curved.end(),[&](const auto& a, const auto& b) { return support(a)>support(b); });
        for (const auto& proposal : curved) {
            if (!appendObservedPatches(residual,proposal,minimumSamples,minimumSamples)) return {};
        }
    }
    if (!refineTransitions && assigned.size()<points.size()*.98) {
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : points.points()) if (!assigned.count(p.id)) unmatched.push_back(p);
        if (unmatched.size()>=48) {
            const reconstruction::PointStore residual(std::move(unmatched));
            reconstruction::PrimitiveDetectionOptions torus;
            torus.detectPlanes = torus.detectCylinders = torus.detectSpheres = torus.detectCones = false;
            torus.torus.minimumSupportPoints = 48;
            torus.torus.maximumTori = 16;
            torus.torus.minimumNormalAlignment = .98;
            torus.torus.distanceThreshold = tolerance;
            torus.torus.probability = .001;
            reconstruction::PrimitiveDetectionResult proposals;
            std::string error;
            if (reconstruction::detectPrimitiveEvidence(residual,torus,proposals,error)) {
                for (const auto& proposal : proposals.evidence) {
                    QJsonObject evidence;
                    const auto face=fitToroidalPatch(std::get<reconstruction::TorusEvidence>(proposal).torus,
                                                    residual,tolerance,assigned,evidence);
                    if (face.IsNull()) continue;
                    builder.Add(compound,face);
                    faces.append(evidence);
                    if (faces.size()>maximumFaces) return {};
                }
            }
        }
    }
    report["mixed_local_coverage"] = double(assigned.size())/points.size();
    if (assigned.size()<points.size()*.98) verifyRemainingObservations(compound,points,tolerance,assigned);
    // A detector can return strong candidates while missing a thin or
    // disconnected wall. Try signed-normal proposals on only the observations
    // that still lack a bounded face.
    if (assigned.size()<points.size()*.98) {
        constexpr std::size_t minimumSamples = 48;
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : points.points())
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id)) unmatched.push_back(p);
        const reconstruction::PointStore residual(std::move(unmatched));
        const auto previousFaces = faces.size();
        for (const auto& proposal : signedCylinderProposals(residual,tolerance,minimumSamples))
            if (!appendObservedPatches(residual,proposal,minimumSamples,24)) return {};
        report["mixed_residual_cylinder_faces"] = faces.size()-previousFaces;
        if (faces.size()>previousFaces && assigned.size()<points.size()*.98)
            verifyRemainingObservations(compound,points,tolerance,assigned);
    }
    if (assigned.size()<points.size()*.98) {
        constexpr std::size_t minimumSamples = 48;
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : points.points())
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id)) unmatched.push_back(p);
        const reconstruction::PointStore residual(std::move(unmatched));
        reconstruction::PrimitiveDetectionOptions planes;
        planes.detectCylinders = planes.detectCones = planes.detectSpheres = planes.detectTori = false;
        planes.plane.minimumSupportPoints = minimumSamples;
        planes.plane.maximumPlanes = 16;
        planes.plane.distanceThreshold = tolerance;
        planes.plane.minimumNormalAlignment = .9995;
        reconstruction::PrimitiveDetectionResult proposals;
        std::string error;
        const auto previousFaces = faces.size();
        if (reconstruction::detectPrimitiveEvidence(residual,planes,proposals,error)) {
            for (const auto& proposal : proposals.evidence) {
                const auto& normal = std::get<reconstruction::PlaneEvidence>(proposal).plane.normal;
                // Determine density and boundaries from each oriented plane on its own.
                for (const double sign : {-1.,1.}) {
                    std::vector<reconstruction::PointSample> coplanar;
                    for (const auto& p : residual.points()) {
                        if (assigned.count(p.id) || reconstruction::surfaceResidual(proposal,p.position)>tolerance) continue;
                        const auto& n = *p.normal;
                        if (sign*(n.x*normal.x+n.y*normal.y+n.z*normal.z)>=.9995) coplanar.push_back(p);
                    }
                    if (coplanar.size()<minimumSamples) continue;
                    reconstruction::AssemblyOptions boundaryOptions;
                    boundaryOptions.buildSolids = false;
                    boundaryOptions.distanceTolerance = tolerance;
                    reconstruction::AssemblyReconstructionResult boundaries;
                    reconstruction::reconstructAssembly(reconstruction::PointStore(std::move(coplanar)),
                                                         boundaryOptions,boundaries,error);
                    for (const auto& patch : boundaries.patches)
                        if (!appendPlanarPatch(patch,minimumSamples)) return {};
                }
            }
        }
        report["mixed_residual_plane_faces"] = faces.size()-previousFaces;
        if (faces.size()>previousFaces && assigned.size()<points.size()*.98)
            verifyRemainingObservations(compound,points,tolerance,assigned);
    }
    if (assigned.size()<points.size()*.98) {
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : points.points())
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id)) unmatched.push_back(p);
        const reconstruction::PointStore residual(std::move(unmatched));
        const auto previousFaces=faces.size();
        for (const auto& proposal : observedCylinders)
            if (!appendObservedPatches(residual,proposal,20,20,true)) return {};
        for (const auto& proposal : signedCylinderProposals(residual,tolerance,28,true))
            if (!appendObservedPatches(residual,proposal,28,20,true)) return {};
        report["mixed_precise_cylinder_faces"]=faces.size()-previousFaces;
        if (faces.size()>previousFaces && assigned.size()<points.size()*.98)
            verifyRemainingObservations(compound,points,tolerance,assigned);
    }
    const int splinePasses = refineTransitions ? 2 : 1;
    int splineFaces = 0;
    for (int splinePass=0; splinePass<splinePasses && assigned.size()<points.size()*.98; ++splinePass) {
        std::vector<reconstruction::PointSample> unmatched;
        for (const auto& p : points.points())
            if (p.normal && p.normalOrientationKnown && !assigned.count(p.id)) unmatched.push_back(p);
        const auto previousFaces = faces.size();
        for (const auto& patch : fitResidualSplines(unmatched,tolerance,refineTransitions)) {
            assigned.insert(patch.points.begin(),patch.points.end());
            builder.Add(compound,patch.face);
            faces.append(patch.report);
            if (faces.size()>maximumFaces) return {};
        }
        if (faces.size()==previousFaces) break;
        splineFaces += int(faces.size()-previousFaces);
    }
    report["mixed_residual_spline_faces"] = splineFaces;
    report["mixed_weak_analytic_faces"] = weakAnalyticFaces;
    report["mixed_proposal_coverage"] = double(assigned.size())/points.size();
    report["mixed_proposal_faces"] = faces.size();
    if (faces.size()<2 || assigned.size()<points.size()*.98 || validFaceCount(compound)!=faces.size()) return {};
    report["kind"] = "mixed";
    report["faces"] = faces;
    report["surface_coverage"] = double(assigned.size())/points.size();
    report["unassigned_points"] = double(points.size()-assigned.size());
    return compound;
}
TopoDS_Shape fitMixedFaces(const reconstruction::PointStore& points,
                           const reconstruction::PrimitiveDetectionOptions& options, double tolerance,
                           QJsonObject& report) {
    QJsonObject ordinary;
    auto shape = buildMixedFaces(points,options,tolerance,false,ordinary);
    const auto faceCount = ordinary["mixed_proposal_faces"].toInt();
    const auto weakFaces = ordinary["mixed_weak_analytic_faces"].toInt();
    const bool fragmented = ordinary["mixed_residual_spline_faces"].toInt()>0 &&
                            weakFaces>=std::max(8,faceCount/5);
    if (fragmented) {
        // Retry when many weak primitives coexist with spline transitions.
        // Keep the new fit only if it preserves coverage with fewer faces.
        QJsonObject alternative;
        const auto refined = buildMixedFaces(points,options,tolerance,true,alternative);
        if (!refined.IsNull() && (shape.IsNull() ||
            (alternative["mixed_proposal_faces"].toInt()<faceCount &&
             alternative["surface_coverage"].toDouble()>=ordinary["surface_coverage"].toDouble()-.01))) {
            ordinary = std::move(alternative);
            ordinary["mixed_refined_transitions"] = true;
            shape = refined;
        }
    }
    for (auto it=ordinary.constBegin(); it!=ordinary.constEnd(); ++it) report[it.key()] = it.value();
    return shape;
}

}

void fitObservedSurface(const QStringList& args, QJsonObject& report) {
    if (args.size() != 6)
        throw std::runtime_error("Usage: fit-observed-surface cloud.ply report.json output.step tolerance-mm");
    bool parsed = false;
    const double tolerance = args[5].toDouble(&parsed);
    if (!parsed || !std::isfinite(tolerance) || tolerance <= 0)
        throw std::runtime_error("Expected a positive tolerance in millimetres");
    const std::filesystem::path path(args[4].toStdWString());
    if (QFileInfo(args[3]).absoluteFilePath() == QFileInfo(args[4]).absoluteFilePath())
        throw std::runtime_error("Report and fitted STEP require distinct output paths");
    if (std::filesystem::exists(path)) throw std::runtime_error("Fitted surface output path already exists");
    const auto unit = PointCloudAdapter::lengthUnitFor(args[2]);
    if (!reconstruction::millimetersPerUnit(unit)) throw std::runtime_error("Expected metric cloud metadata");
    PlyCloud cloud;
    QString readError;
    if (!PlyReader::read(args[2], cloud, readError)) throw std::runtime_error(readError.toStdString());
    auto preprocessing = reconstruction::PointCloudPreprocessingOptions{};
    preprocessing.removeStatisticalOutliers = false;
    reconstruction::PointStore points({});
    reconstruction::PointCloudPreprocessingReport prep;
    std::string error;
    if (!reconstruction::preprocessPointCloud(PointCloudAdapter::toPointStore(cloud, unit), preprocessing,
                                              points, prep, error)) throw std::runtime_error(error);
    report["surface_fitted"] = false;
    report["cad_constructed"] = false;
    report["input_points"] = double(points.size());
    report["tolerance_mm"] = tolerance;
    report["status"] = "surface_not_recognized";

    reconstruction::PrimitiveDetectionOptions options;
    options.detectPlanes = options.detectSpheres = options.detectTori = false;
    options.cylinder.minimumSupportPoints = std::clamp<std::size_t>(points.size()/2,40,80);
    options.cone.minimumSupportPoints = 80;
    options.cylinder.distanceThreshold = options.cone.distanceThreshold = tolerance;
    options.cylinder.probability = options.cone.probability = .001;
    options.cylinder.maximumCylinders = 16;
    std::vector<reconstruction::CylinderEquation> cylinders;
    std::optional<AxialFit> best;
    std::optional<std::pair<AxialFit,reconstruction::PrimitiveEvidence>> partial;
    for (const bool cylinder : {true, false}) {
        options.detectCylinders = cylinder;
        options.detectCones = !cylinder;
        reconstruction::PrimitiveDetectionResult proposals;
        if (!reconstruction::detectPrimitiveEvidence(points, options, proposals, error)) continue;
        for (const auto& proposal : proposals.evidence) {
            if (cylinder) cylinders.push_back(std::get<reconstruction::CylinderEvidence>(proposal).cylinder);
            const auto fit = measure(points, proposal, tolerance);
            if (!fit) continue;
            if (fit->gridSupported && (!best || fit->meanResidual < best->meanResidual)) best = fit;
            else if (!fit->gridSupported && (!partial || fit->meanResidual < partial->first.meanResidual))
                partial.emplace(*fit,proposal);
        }
    }
    TopoDS_Face face;
    TopoDS_Shape shape;
    if (!best && partial) {
        const auto& fit = partial->first;
        auto patches = trimAxialSurfaces(points,partial->second,fit,tolerance,.98,40);
        if (patches.size()==1) {
            face = patches.front().face;
            for (auto it=patches.front().report.begin(); it!=patches.front().report.end(); ++it)
                report[it.key()] = it.value();
            best = fit;
        } else if (!patches.empty()) {
            BRep_Builder builder;
            TopoDS_Compound compound;
            builder.MakeCompound(compound);
            QJsonArray details;
            PointIds assigned;
            for (auto& patch : patches) {
                if (fit.normalDirection<0) patch.face.Reverse();
                describeObservedPatch(patch.report,fit);
                builder.Add(compound,patch.face);
                details.append(patch.report);
                assigned.insert(patch.points.begin(),patch.points.end());
            }
            shape = compound;
            report["kind"] = "axial";
            report["faces"] = details;
            report["surface_coverage"] = double(assigned.size())/points.size();
            report["unassigned_points"] = double(points.size()-assigned.size());
        }
    }
    if (best) {
        const auto& fit = *best;
        const bool conical = fit.angle > 0;
        const double endAngle = fit.startAngle + fit.angularSpan;
        if (face.IsNull()) {
            auto builder = conical
                ? BRepBuilderAPI_MakeFace(gp_Cone(fit.frame, fit.angle, 0), fit.startAngle, endAngle,
                                          fit.low / std::cos(fit.angle), fit.high / std::cos(fit.angle))
                : BRepBuilderAPI_MakeFace(gp_Cylinder(fit.frame, fit.radius), fit.startAngle, endAngle, fit.low, fit.high);
            if (!builder.IsDone()) throw std::runtime_error("Invalid fitted face");
            face = builder.Face();
        }
        if (fit.normalDirection < 0) face.Reverse();
        const auto& p = fit.frame.Location();
        const auto& axis = fit.frame.Direction();
        report["kind"] = conical ? "cone" : "cylinder";
        report["axis_origin_mm"] = QJsonArray{p.X(), p.Y(), p.Z()};
        report["axis_direction"] = QJsonArray{axis.X(), axis.Y(), axis.Z()};
        report["axial_bounds_mm"] = QJsonArray{fit.low, fit.high};
        report["angular_span_rad"] = fit.angularSpan;
        report["radial_normal_direction"] = fit.normalDirection < 0 ? "inward" :
            fit.normalDirection > 0 ? "outward" : "undetermined";
        if (conical) report["semi_angle_rad"] = fit.angle;
        else report["radius_mm"] = fit.radius;
        report["surface_coverage"] = report.contains("trim_coverage") ? report["trim_coverage"].toDouble() : fit.coverage;
        report["parameter_bin_coverage"] = fit.occupied;
        report["mean_residual_mm"] = fit.meanResidual;
    } else if (shape.IsNull()) {
        face = fitPlanarFace(points, options, tolerance, report);
    }
    if (shape.IsNull()) shape = face;
    if (shape.IsNull()) shape = fitCoaxialFaces(points,cylinders,tolerance,report);
    if (shape.IsNull()) shape = fitMixedFaces(points,options,tolerance,report);
    if (shape.IsNull()) {
        report["error"] = "No supported bounded analytic surface set explains the observations";
        return;
    }
    const int faceCount = validFaceCount(shape);
    if (!faceCount) throw std::runtime_error("Invalid fitted surface geometry");
    std::stringstream step;
    if (!modeling::exportStep(shape, step, &error)) throw std::runtime_error(error);
    step.seekg(0);
    STEPControl_Reader reader;
    if (reader.ReadStream("observed-surface.step", step) != IFSelect_RetDone ||
        reader.TransferRoots() <= 0 || validFaceCount(reader.OneShape()) != faceCount)
        throw std::runtime_error("Fitted surfaces failed STEP round-trip validation");
    if (faceCount>1) {
        auto faceMetrics = [](const TopoDS_Shape& model) {
            std::vector<std::tuple<int,double,double,double>> metrics;
            for (TopExp_Explorer it(model,TopAbs_FACE); it.More(); it.Next()) {
                const auto face = TopoDS::Face(it.Current());
                const BRepAdaptor_Surface surface(face);
                GProp_GProps area;
                BRepGProp::SurfaceProperties(face,area);
                double firstParameter = 0, secondParameter = 0;
                switch (surface.GetType()) {
                case GeomAbs_Cylinder: firstParameter = surface.Cylinder().Radius(); break;
                case GeomAbs_Cone: firstParameter = std::abs(surface.Cone().SemiAngle()); break;
                case GeomAbs_Torus: {
                    const auto torus = surface.Torus();
                    firstParameter = torus.MajorRadius();
                    secondParameter = torus.MinorRadius();
                    break;
                }
                default: break;
                }
                metrics.emplace_back(surface.GetType(),firstParameter,secondParameter,area.Mass());
            }
            std::sort(metrics.begin(),metrics.end());
            return metrics;
        };
        const auto expected = faceMetrics(shape), restored = faceMetrics(reader.OneShape());
        for (std::size_t i=0; i<expected.size(); ++i) {
            const auto [kind,firstParameter,secondParameter,area] = expected[i];
            const auto [actualKind,actualFirstParameter,actualSecondParameter,actualArea] = restored[i];
            if (kind!=actualKind || std::abs(firstParameter-actualFirstParameter)>1e-6 ||
                std::abs(secondParameter-actualSecondParameter)>1e-6 || std::abs(area-actualArea)>area*1e-5)
                throw std::runtime_error("Surface geometry changed during STEP round trip");
        }
    }
    if (report.contains("trim_coverage")) {
        const TopExp_Explorer faces(reader.OneShape(),TopAbs_FACE);
        const auto restored = TopoDS::Face(faces.Current());
        BRepAdaptor_Surface geometry(restored);
        GProp_GProps expectedArea, restoredArea;
        BRepGProp::SurfaceProperties(face,expectedArea);
        BRepGProp::SurfaceProperties(restored,restoredArea);
        const bool geometryMatches = best->angle>0
            ? geometry.GetType()==GeomAbs_Cone && std::abs(std::abs(geometry.Cone().SemiAngle())-best->angle)<=1e-6
            : geometry.GetType()==GeomAbs_Cylinder && std::abs(geometry.Cylinder().Radius()-best->radius)<=1e-6;
        if (!geometryMatches || std::abs(expectedArea.Mass()-restoredArea.Mass())>expectedArea.Mass()*1e-5)
            throw std::runtime_error("Observed curved trim changed during STEP round trip");
    }
    const auto bytes = step.str();
    QSaveFile file(args[4]);
    if (!QDir().mkpath(QFileInfo(args[4]).absolutePath()) || !file.open(QIODevice::WriteOnly) ||
        file.write(bytes.data(), qint64(bytes.size())) != qint64(bytes.size()) || !file.commit())
        throw std::runtime_error("Cannot save fitted surface STEP");
    report["status"] = "surface_fitted";
    report["surface_fitted"] = true;
    report["step_round_trip_verified"] = true;
    report["face_count"] = faceCount;
    report["solids"] = 0;
    report["step"] = QFileInfo(args[4]).absoluteFilePath();
}
