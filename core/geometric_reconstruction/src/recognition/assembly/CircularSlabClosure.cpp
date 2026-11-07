#include "AssemblyInternal.h"
#include "CircularProfile.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <stdexcept>
#include <modeling/PrimitiveEditing.h>

namespace reconstruction::detail::assembly {
namespace {
constexpr double turn = 6.28318530717958647692;

struct Sample {
    PointId id;
    Vec3 p, n;
    double radius;
};

using Cut = ProfileHalfPlane;
using Curve = modeling::ProfileCurve2D;

Vec3 atAngle(double radius, double angle) {
    return {radius*std::cos(angle),radius*std::sin(angle),0};
}

bool inside(const std::vector<Cut>& cuts, Vec3 p, double tolerance) {
    return std::all_of(cuts.begin(),cuts.end(),[&](const Cut& cut) {
        return dot(cut.normal,p) <= cut.offset+tolerance;
    });
}

}

std::vector<Curve> circularOutline(double radius, const std::vector<Cut>& cuts, double tolerance) {
    std::vector<double> angles{0,turn/2,turn};
    std::vector<Curve> pieces;
    for (const auto& cut : cuts) {
        const auto center = scale(cut.normal,cut.offset);
        const Vec3 tangent{-cut.normal.y,cut.normal.x,0};
        const double half = std::sqrt((radius-cut.offset)*(radius+cut.offset));
        for (double s : {-half,half}) {
            const auto point = add(center,scale(tangent,s));
            double angle = std::atan2(point.y,point.x);
            if (angle < 0) angle += turn;
            angles.push_back(angle);
        }
        double lo = -half, hi = half;
        for (const auto& other : cuts) {
            const double slope = dot(other.normal,tangent);
            const double offset = other.offset-dot(other.normal,center);
            if (slope > 1e-12) hi = std::min(hi,offset/slope);
            else if (slope < -1e-12) lo = std::max(lo,offset/slope);
            else if (offset < -tolerance) { hi = lo; break; }
        }
        if (hi-lo <= tolerance) continue;
        const auto a = add(center,scale(tangent,lo)), b = add(center,scale(tangent,hi));
        pieces.push_back({false,a.x,a.y,b.x,b.y});
    }
    std::sort(angles.begin(),angles.end());
    for (std::size_t i=1; i<angles.size(); ++i) {
        const double a = angles[i-1], b = angles[i];
        if ((b-a)*radius <= tolerance || !inside(cuts,atAngle(radius,(a+b)*.5),1e-8*radius)) continue;
        const auto p = atAngle(radius,a), q = atAngle(radius,b);
        pieces.push_back({true,p.x,p.y,q.x,q.y,0,0,radius,a,b-a});
    }
    if (pieces.size() < 2 || std::none_of(pieces.begin(),pieces.end(),[](const auto& p) { return p.circular; })) return {};
    // Chords and arcs are oriented with material to their left. Join their
    // analytic endpoints without stitching across an unsupported gap.
    const double weld = std::max(1e-8*radius,1e-7);
    const auto joins = [&](const Curve& a, const Curve& b) { return std::hypot(a.x2-b.x1,a.y2-b.y1) <= weld; };
    for (std::size_t i=1; i<pieces.size(); ++i) {
        const auto next = std::find_if(pieces.begin()+i,pieces.end(),[&](const auto& b) { return joins(pieces[i-1],b); });
        if (next == pieces.end()) return {};
        std::iter_swap(pieces.begin()+i,next);
    }
    return joins(pieces.back(),pieces.front()) ? pieces : std::vector<Curve>{};
}

double profileWallCoordinate(const Curve& c, Vec3 p, Vec3 n, double tolerance) {
    if (c.circular) {
        const Vec3 radial{p.x-c.centerX,p.y-c.centerY,0};
        const double radius = length(radial);
        if (radius <= 0 || std::abs(radius-c.radius) > tolerance || dot(n,radial) < .98*radius)
            return -1;
        const double angle = std::remainder(std::atan2(radial.y,radial.x)-c.startAngle-c.sweepAngle*.5,turn)+c.sweepAngle*.5;
        return angle/c.sweepAngle;
    }
    const Vec3 edge{c.x2-c.x1,c.y2-c.y1,0}, fromStart{p.x-c.x1,p.y-c.y1,0};
    const double span = length(edge);
    const Vec3 normal{edge.y/span,-edge.x/span,0};
    if (std::abs(dot(normal,fromStart)) > tolerance || dot(n,normal) < .98) return -1;
    return dot(edge,fromStart)/(span*span);
}

namespace {

struct Fit {
    std::unordered_set<PointId> support;
    bool inferredCap = false;
};

Fit verify(const std::vector<Sample>& points, const std::vector<Cut>& cuts, const std::vector<Curve>& outline,
           double radius, double low, double high, double tolerance, const double (&contact)[2]) {
    constexpr int grid = 14, around = 8, axial = 3;
    const double cell = 2*radius/grid;
    std::bitset<grid*grid> caps[2];
    std::vector<std::bitset<around*axial>> walls(outline.size());
    Fit fit;
    for (const auto& p : points) {
        if (p.p.z < low-tolerance || p.p.z > high+tolerance || p.radius > radius+tolerance ||
            !inside(cuts,p.p,tolerance)) continue;
        const bool bottom = std::abs(p.p.z-low) <= tolerance && p.n.z <= -.98;
        const bool top = std::abs(p.p.z-high) <= tolerance && p.n.z >= .98;
        if (bottom || top) {
            const int x = std::clamp(int((p.p.x+radius)/cell),0,grid-1);
            const int y = std::clamp(int((p.p.y+radius)/cell),0,grid-1);
            caps[top].set(x*grid+y);
            fit.support.insert(p.id);
            continue;
        }
        const int row = std::clamp(int((p.p.z-low)*axial/(high-low)),0,axial-1);
        for (std::size_t i=0; i<outline.size(); ++i) {
            const double u = profileWallCoordinate(outline[i],p.p,p.n,tolerance);
            if (u < 0 || u > 1) continue;
            walls[i].set(std::min(around-1,int(u*around))*axial+row);
            fit.support.insert(p.id);
        }
    }
    if (fit.support.size() < points.size()*.98) return {};
    for (const auto& wall : walls) {
        if (wall.count() < around*axial*.85) return {};
        for (int u=0; u<around; ++u)
            if (!wall.test(u*axial) && !wall.test(u*axial+1) && !wall.test(u*axial+2)) return {};
    }
    std::bitset<grid*grid> expected, visible[2];
    for (int x=0; x<grid; ++x) for (int y=0; y<grid; ++y) {
        const Vec3 p{-radius+(x+.5)*cell,-radius+(y+.5)*cell,0};
        const double r = length(p);
        if (r > radius-cell*.35 || !inside(cuts,p,-cell*.35)) continue;
        const int bin = x*grid+y;
        expected.set(bin);
        for (int cap : {0,1}) if (r > contact[cap]+cell*.45) visible[cap].set(bin);
    }
    if (expected.count() < 24) return {};
    for (int cap : {0,1}) {
        if ((caps[cap]&expected).count() >= expected.count()*.92) continue;
        // A missing region is admissible only on the face touching an
        // independently closed body, and the opposite face must be observed.
        if (contact[cap] <= 0 || visible[cap].count() < expected.count()*.33 ||
            (caps[cap]&visible[cap]).count() < visible[cap].count()*.92 ||
            (caps[1-cap]&expected).count() < expected.count()*.92) return {};
        fit.inferredCap = true;
    }
    return fit;
}

bool adjacentTube(const modeling::ProfileBodyParams& tube, const CylinderEquation& cylinder,
                  double low, double high, double tolerance, double (&contact)[2]) {
    const auto axis = cylinder.axisDirection;
    const auto fromAxis = subtract(tube.pose.origin,cylinder.axisPoint);
    const double alignment = dot(axis,tube.pose.zDirection);
    if (tube.envelope.size() != 1 || std::abs(alignment) < .9999 ||
        length(cross(fromAxis,axis)) > tolerance) return false;
    const double start = dot(fromAxis,axis), end = start+alignment*tube.depth;
    const auto lower = std::min(start,end), upper = std::max(start,end);
    const auto radius = tube.envelope.front().startRadius;
    if (radius >= cylinder.radius-tolerance) return false;
    if (std::abs(upper-low) <= tolerance) contact[0] = radius;
    else if (std::abs(lower-high) <= tolerance) contact[1] = radius;
    else return false;
    return true;
}
}

void closeCircularSlab(const PointStore& source, const WorkingData& work, AssemblyReconstructionResult& result,
                       const std::function<bool()>& cancelled) {
    const bool residual = !result.candidate.profileBodies.empty();
    const auto count = residual ? result.unassignedPointIds.size() : source.size();
    if (count < 500 || std::none_of(result.patches.begin(),result.patches.end(),[](const auto& p) { return !p.planar; })) return;
    const std::unordered_set<PointId> remaining(result.unassignedPointIds.begin(),result.unassignedPointIds.end());
    std::vector<const PointSample*> observations;
    observations.reserve(count);
    for (const auto& p : source.points()) {
        if ((!residual || remaining.count(p.id)) && p.normal && p.normalOrientationKnown && length(*p.normal)>1e-12)
            observations.push_back(&p);
    }
    if (observations.size() < count*.98) return;
    std::vector<PointSample> proposals;
    constexpr std::size_t budget = 16000;
    const auto stride = std::max<std::size_t>(1,(observations.size()+budget-1)/budget);
    for (std::size_t i=0; i<observations.size(); i+=stride) proposals.push_back(*observations[i]);
    const double tolerance = std::max(work.tolerance*4,1e-6);
    PrimitiveDetectionOptions options;
    options.detectSpheres = options.detectCones = options.detectTori = false;
    options.plane.maximumPlanes = 12;
    options.cylinder.maximumCylinders = 8;
    options.plane.minimumSupportPoints = options.cylinder.minimumSupportPoints = 100;
    options.plane.distanceThreshold = options.cylinder.distanceThreshold = tolerance;
    options.plane.minimumNormalAlignment = options.cylinder.minimumNormalAlignment = .98;
    PrimitiveDetectionResult evidence;
    std::string failure;
    if (!detectPrimitiveEvidence(PointStore(std::move(proposals)),options,evidence,failure)) return;
    for (const auto& proposal : evidence.evidence) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const auto* candidate = std::get_if<CylinderEvidence>(&proposal);
        if (!candidate) continue;
        const auto& c = candidate->cylinder;
        const auto axis = c.axisDirection;
        const auto x = normalized(cross(axis,std::abs(axis.x)<.8 ? Vec3{1,0,0} : Vec3{0,1,0})), y = cross(axis,x);
        auto local = [&](Vec3 p) { return Vec3{dot(p,x),dot(p,y),dot(p,axis)}; };
        std::vector<Sample> points;
        points.reserve(observations.size());
        for (const auto* p : observations) {
            const auto q = local(subtract(p->position,c.axisPoint));
            points.push_back({p->id,q,local(normalized(*p->normal)),std::hypot(q.x,q.y)});
        }
        std::vector<Cut> cuts;
        std::vector<double> caps[2];
        for (const auto& item : evidence.evidence) {
            const auto* plane = std::get_if<PlaneEvidence>(&item);
            if (!plane) continue;
            auto n = local(plane->plane.normal);
            double offset = -plane->plane.offset-dot(plane->plane.normal,c.axisPoint);
            int positive = 0, negative = 0;
            for (const auto& p : points) {
                if (p.radius > c.radius+tolerance || std::abs(dot(p.p,n)-offset)>tolerance) continue;
                const double alignment = dot(p.n,n);
                positive += alignment >= .98;
                negative += alignment <= -.98;
            }
            if (std::max(positive,negative)<100 || std::min(positive,negative)>.1*std::max(positive,negative)) continue;
            if (negative > positive) { n = scale(n,-1); offset = -offset; }
            if (std::abs(n.z) >= .9999) caps[n.z>0].push_back(offset/n.z);
            else if (std::abs(n.z)*c.radius <= tolerance && std::abs(offset) < c.radius-tolerance)
                cuts.push_back({normalized(Vec3{n.x,n.y,0}),offset/std::hypot(n.x,n.y)});
        }
        if (cuts.empty()) continue;
        const auto outline = circularOutline(c.radius,cuts,tolerance);
        if (outline.empty()) continue;
        for (const double low : caps[0]) for (const double high : caps[1]) {
            const double depth = high-low;
            if (depth <= 8*tolerance || depth > c.radius) continue;
            double contact[2]{};
            if (residual && !adjacentTube(result.candidate.profileBodies.front(),c,low,high,tolerance,contact)) continue;
            auto fit = verify(points,cuts,outline,c.radius,low,high,tolerance,contact);
            if (fit.support.empty()) continue;
            modeling::ProfileBodyParams body;
            body.pose = {add(c.axisPoint,scale(axis,low)),x,axis};
            body.outline = outline;
            body.depth = body.sizeZ = depth;
            body.sizeX = body.referenceSizeX = body.sizeY = body.referenceSizeY = 2*c.radius;
            if (!modeling::refreshProfileBodyDimensions(body,failure)) continue;
            publishAnalyticProfile(source,work,result,std::move(body),fit.support,
                fit.inferredCap ? "closed_circular_slab_inferred_contact" : "closed_circular_slab",
                fit.inferredCap ? "The observed end perimeter bounds a planar continuation at the tube contact" :
                                  "Signed caps, chords and circular walls support a closed slab");
            return;
        }
    }
}
}
