#include "AssemblyInternal.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <stdexcept>

namespace reconstruction::detail::assembly {
namespace {
constexpr int sectors = 16;
constexpr int capLayers = 3;
constexpr int wallLayers = 6;
constexpr double pi = 3.14159265358979323846;

struct RadialObservation {
    PointId id;
    double height, radius;
    double axialNormal, radialNormal;
    int sector;
};

struct Cap {
    double height;
    int direction;
};

std::vector<Cap> observedCaps(const std::vector<RadialObservation>& points,
    double inner, double outer, double tolerance) {
    std::vector<Cap> caps;
    for (int direction : {-1, 1}) {
        std::vector<const RadialObservation*> axial;
        for (const auto& p : points) {
            if (p.axialNormal * direction >= .98 && p.radius >= inner-tolerance && p.radius <= outer+tolerance)
                axial.push_back(&p);
        }
        std::sort(axial.begin(), axial.end(), [](auto a, auto b) { return a->height < b->height; });
        for (std::size_t i = 0; i < axial.size();) {
            std::size_t j = i;
            double heightSum = 0;
            std::bitset<sectors * capLayers> bins;
            std::bitset<sectors> around;
            while (j < axial.size() && axial[j]->height-axial[i]->height <= 2*tolerance) {
                const auto& p = *axial[j++];
                const double areaFraction = (p.radius-inner)/(outer-inner)*(p.radius+inner)/(outer+inner);
                const int row = std::clamp(int(areaFraction*capLayers), 0, capLayers-1);
                bins.set(p.sector*capLayers+row);
                around.set(p.sector);
                heightSum += p.height;
            }
            if (j-i >= 96 && around.all() && bins.count() >= sectors*capLayers*.92)
                caps.push_back({heightSum / (j-i), direction});
            i = j;
        }
    }
    return caps;
}

std::unordered_set<PointId> tubeSupport(const std::vector<RadialObservation>& points,
    double inner, double outer, double low, double high, double tolerance) {
    std::bitset<sectors * wallLayers> walls[2];
    std::unordered_set<PointId> support;
    std::size_t interior = 0;
    for (const auto& p : points) {
        if (p.height < low-tolerance || p.height > high+tolerance ||
            p.radius < inner-tolerance || p.radius > outer+tolerance) continue;
        const bool onInner = std::abs(p.radius-inner) <= tolerance && p.radialNormal <= -.98;
        const bool onOuter = std::abs(p.radius-outer) <= tolerance && p.radialNormal >= .98;
        if (onInner || onOuter) {
            const int row = std::clamp(int((p.height-low)*wallLayers/(high-low)),0,wallLayers-1);
            walls[onOuter].set(p.sector*wallLayers+row);
            support.insert(p.id);
        } else if ((std::abs(p.height-low) <= tolerance && p.axialNormal <= -.98) ||
                   (std::abs(p.height-high) <= tolerance && p.axialNormal >= .98)) {
            support.insert(p.id);
        } else if (p.height > low+tolerance && p.height < high-tolerance &&
                   p.radius > inner+tolerance && p.radius < outer-tolerance) {
            ++interior;
        }
    }
    if (interior > std::max<std::size_t>(4,support.size()/500)) return {};
    for (const auto& wall : walls) {
        if (wall.count() < sectors*wallLayers*.92) return {};
        int lower = 0, upper = 0;
        for (int u = 0; u < sectors; ++u) {
            int occupied = 0;
            for (int v=0; v<wallLayers; ++v) occupied += wall.test(u*wallLayers+v);
            if (occupied == 0) return {};
            lower += wall.test(u*wallLayers);
            upper += wall.test(u*wallLayers+wallLayers-1);
        }
        // Independently observed, full annular caps anchor the ends when a
        // few wall cells nearest the ends happen to lack observations.
        if (std::min(lower,upper) < sectors-2) return {};
    }
    return support;
}

std::vector<CylinderEquation> capConstrainedCylinders(const WorkingData& work,
        const std::vector<FiniteSurfacePatch>& patches, double tolerance) {
    std::vector<Vec3> axes;
    for (const auto& a : patches) {
        if (!a.planar || !a.orientationKnown || a.workingPointIds.size()<96) continue;
        if (std::any_of(axes.begin(),axes.end(),[&](Vec3 axis){ return std::abs(dot(axis,a.normal))>.9999; })) continue;
        if (std::none_of(patches.begin(),patches.end(),[&](const auto& b) {
                return b.planar && b.orientationKnown && b.workingPointIds.size()>=96 && dot(a.normal,b.normal)<-.9999 &&
                       dot(a.normal,subtract(a.origin,b.origin))>8*tolerance;
            })) continue;
        axes.push_back(canonicalDirection(a.normal));
        if (axes.size()==4) break;
    }
    std::vector<CylinderEquation> cylinders;
    for (const auto axis : axes) {
        const auto fitted = fitCylinderWallsAlongAxis(work,patches,axis,tolerance);
        cylinders.insert(cylinders.end(),fitted.begin(),fitted.end());
        if (cylinders.size()>=16) { cylinders.resize(16); break; }
    }
    return cylinders;
}
}

std::vector<CylinderEquation> fitCylinderWallsAlongAxis(const WorkingData& work,
        const std::vector<FiniteSurfacePatch>& patches, Vec3 axis, double tolerance) {
    std::vector<CylinderEquation> cylinders;
    for (const auto& patch : patches) {
        if (patch.planar || !patch.orientationKnown || patch.workingPointIds.size()<100) continue;
        const auto& indices = work.groups[patch.id-1];
        auto transverse = [&](Vec3 v) { return subtract(v,scale(axis,dot(v,axis))); };
        auto position = [&](std::size_t i) { return transverse(subtract(work.points[i].position,patch.origin)); };
        auto normal = [&](std::size_t i) { return transverse(*work.points[i].normal); };
        Vec3 meanPosition{}, meanNormal{};
        for (auto i : indices) {
            meanPosition = add(meanPosition,position(i));
            meanNormal = add(meanNormal,normal(i));
        }
        meanPosition = scale(meanPosition,1.0/indices.size());
        meanNormal = scale(meanNormal,1.0/indices.size());
        double covariance = 0, variance = 0;
        for (auto i : indices) {
            const auto n = subtract(normal(i),meanNormal);
            covariance += dot(subtract(position(i),meanPosition),n);
            variance += dot(n,n);
        }
        if (variance < indices.size()*.1) continue;
        // On a signed cylindrical wall, transverse position is the axis
        // centre plus a common signed radius times the transverse normal.
        const double radius = covariance/variance;
        if (std::abs(radius) < 8*tolerance) continue;
        const auto center = subtract(meanPosition,scale(meanNormal,radius));
        const auto supported = std::count_if(indices.begin(),indices.end(),[&](auto i) {
            const auto radial = subtract(position(i),center);
            const double r = length(radial);
            return std::abs(r-std::abs(radius))<=tolerance && dot(radial,normal(i))*std::copysign(1.0,radius)>=.98*r;
        });
        if (supported >= indices.size()*.90)
            cylinders.push_back({transverse(add(patch.origin,center)),axis,std::abs(radius)});
        if (cylinders.size()==16) return cylinders;
    }
    return cylinders;
}

void closeAnnularTube(const PointStore& source, const WorkingData& work, AssemblyReconstructionResult& result,
                      const std::function<bool()>& cancelled) {
    if (work.points.size() < 400 || std::none_of(result.patches.begin(), result.patches.end(), [](const auto& p) {
            return !p.planar;
        })) return;
    // Bound the proposal search, then verify against the original signed observations.
    constexpr std::size_t proposalBudget = 16000;
    std::vector<PointSample> proposalPoints;
    const auto stride = std::max<std::size_t>(1,(work.points.size()+proposalBudget-1)/proposalBudget);
    for (std::size_t i = 0; i < work.points.size(); i += stride) {
        const auto& p = work.points[i];
        if (p.normal && p.normalOrientationKnown) proposalPoints.push_back(p);
    }
    const double tolerance = std::max(work.tolerance*4,1e-6);
    PrimitiveDetectionOptions options;
    options.detectPlanes = options.detectSpheres = options.detectCones = options.detectTori = false;
    options.cylinder.minimumSupportPoints = 100;
    options.cylinder.maximumCylinders = 16;
    options.cylinder.minimumNormalAlignment = .98;
    options.cylinder.distanceThreshold = tolerance;
    options.cylinder.minimumRadius = 8*tolerance;
    PrimitiveDetectionResult detection;
    std::string failure;
    detectPrimitiveEvidence(PointStore(std::move(proposalPoints)),options,detection,failure);
    std::vector<CylinderEquation> cylinders;
    for (const auto& e : detection.evidence) cylinders.push_back(std::get<CylinderEvidence>(e).cylinder);
    if (cylinders.size()<2) cylinders = capConstrainedCylinders(work,result.patches,tolerance);

    modeling::ProfileBodyParams best;
    std::unordered_set<PointId> bestSupport;
    const auto minimumSupport = std::max<std::size_t>(384,source.size()/5);
    for (const auto& outer : cylinders) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const auto axis = outer.axisDirection;
        const auto x = normalized(cross(axis,std::abs(axis.x) < .8 ? Vec3{1,0,0} : Vec3{0,1,0}));
        const auto y = cross(axis,x);
        std::vector<double> innerRadii;
        for (const auto& inner : cylinders) {
            if (outer.radius-inner.radius <= 4*tolerance || std::abs(dot(inner.axisDirection,axis)) < .9999 ||
                length(cross(subtract(inner.axisPoint,outer.axisPoint),axis)) > tolerance) continue;
            innerRadii.push_back(inner.radius);
        }
        if (innerRadii.empty()) continue;

        std::vector<RadialObservation> local;
        for (const auto& p : source.points()) {
            if (!p.normal || !p.normalOrientationKnown || length(*p.normal) <= 1e-12) continue;
            const auto q = subtract(p.position,outer.axisPoint);
            const double u = dot(q,x), v = dot(q,y), radius = std::hypot(u,v);
            if (radius <= tolerance || radius > outer.radius+tolerance) continue;
            const double height = dot(q,axis);
            const auto normal = normalized(*p.normal);
            const auto radial = subtract(q,scale(axis,height));
            const int sector = std::min(sectors-1,int((std::atan2(v,u)+pi)*sectors/(2*pi)));
            local.push_back({p.id,height,radius,dot(normal,axis),dot(normal,radial)/radius,sector});
        }
        for (const double inner : innerRadii) {
            const auto caps = observedCaps(local,inner,outer.radius,tolerance);
            if (caps.size() > 16) continue;
            for (const auto& bottom : caps) for (const auto& top : caps) {
                const double depth = top.height-bottom.height;
                if (bottom.direction != -1 || top.direction != 1 || depth <= 8*tolerance) continue;
                auto support = tubeSupport(local,inner,outer.radius,bottom.height,top.height,tolerance);
                if (support.size() < minimumSupport || support.size() <= bestSupport.size()) continue;

                modeling::ProfileBodyParams body;
                body.pose = {add(outer.axisPoint,scale(axis,bottom.height)),x,axis};
                body.sizeX = body.referenceSizeX = body.sizeY = body.referenceSizeY = 2*outer.radius;
                body.sizeZ = body.depth = depth;
                body.envelope.push_back({0,0,0,depth,outer.radius,outer.radius,inner,inner});
                TopoDS_Shape built;
                if (!modeling::buildProfileBody(body,built,failure)) continue;
                best = std::move(body);
                bestSupport = std::move(support);
            }
        }
    }
    if (!bestSupport.empty())
        publishAnalyticProfile(source,work,result,std::move(best),bestSupport,"closed_annular_tube",
            "Signed inner and outer walls span two observed annular end faces");
}
}
