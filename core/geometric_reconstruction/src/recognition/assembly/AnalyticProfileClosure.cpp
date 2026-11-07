#include "AssemblyInternal.h"
#include "BoundaryGeometry.h"
#include "CircularProfile.h"
#include <modeling/PrimitiveEditing.h>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <Bnd_Box.hxx>
#include <ElSLib.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Torus.hxx>

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace reconstruction::detail::assembly {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double minimumCoverage = .98;

struct LocalPoint {
    PointId id;
    Vec3 position;
    Vec3 normal;
};

struct Side {
    Vec3 normal;
    double offset;
};

struct FiniteWall {
    Side plane;
    double low, high;
};

struct Circle {
    double x, y, radius;
};

struct Bore {
    Circle circle;
    std::vector<modeling::AxialSection> sections;

    double openingRadius(double z) const {
        if (z == sections.front().start) return sections.front().startRadius;
        if (z == sections.back().end) return sections.back().endRadius;
        return 0;
    }
};

modeling::AxialSection boreSection(const Circle& hole, double a, double b, double ra, double rb) {
    if (a > b) { std::swap(a,b); std::swap(ra,rb); }
    return {hole.x,hole.y,a,b,ra,rb};
}

struct CornerArc {
    Vec3 center, start, end;
    double radius, angle, sweep;
};

using Corners = std::vector<std::optional<CornerArc>>;

double boundaryOffset(const Side& side, Vec3 p) {
    return dot(side.normal, p) - side.offset;
}

std::vector<Vec3> clippedOutline(const std::vector<Side>& sides, double extent) {
    std::vector<Vec3> polygon{{-extent,-extent,0}, {extent,-extent,0},
                              {extent,extent,0}, {-extent,extent,0}};
    for (const auto& side : sides) {
        std::vector<Vec3> clipped;
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const auto a = polygon[i], b = polygon[(i + 1) % polygon.size()];
            const double da = boundaryOffset(side, a), db = boundaryOffset(side, b);
            if (da <= 0) clipped.push_back(a);
            if ((da < 0 && db > 0) || (da > 0 && db < 0))
                clipped.push_back(add(a, scale(subtract(b, a), da / (da - db))));
        }
        polygon = std::move(clipped);
    }
    // Every side of the outline must come from observations.
    if (polygon.size() < 3 || std::any_of(polygon.begin(), polygon.end(), [&](Vec3 p) {
            return std::max(std::abs(p.x), std::abs(p.y)) >= extent * .99;
        })) return {};
    return polygon;
}

bool insideOutline(const std::vector<Side>& sides, Vec3 point, double tolerance) {
    return std::none_of(sides.begin(), sides.end(), [&](const Side& s) { return boundaryOffset(s, point) > tolerance; });
}

bool insideRoundedOutline(const std::vector<Side>& sides, const Corners& corners, Vec3 p, double tolerance) {
    if (!insideOutline(sides, p, tolerance)) return false;
    for (const auto& arc : corners) {
        if (!arc || cross(subtract(arc->end,arc->start), subtract(p,arc->start)).z >= 0) continue;
        if (std::hypot(p.x-arc->center.x, p.y-arc->center.y) > arc->radius+tolerance) return false;
    }
    return true;
}

std::vector<std::vector<Vec3>> finiteWallLoops(const std::vector<FiniteWall>& walls, double tolerance, bool inside) {
    if (walls.size()<3 || walls.size()>48) return {};
    const auto absent = walls.size();
    std::vector<std::size_t> successor(walls.size(),absent);
    std::vector<Vec3> ends(walls.size());
    for (std::size_t i=0; i<walls.size(); ++i) {
        const auto& a = walls[i];
        const auto n = a.plane.normal;
        for (std::size_t j=0; j<walls.size(); ++j) {
            const auto& b = walls[j];
            const auto m = b.plane.normal;
            const auto determinant = cross(n,m).z;
            if (std::abs(determinant)<1e-3) continue;
            const Vec3 intersection{(a.plane.offset*m.y-b.plane.offset*n.y)/determinant,
                                    (n.x*b.plane.offset-m.x*a.plane.offset)/determinant,0};
            const double gapA = -n.y*intersection.x+n.x*intersection.y-a.high;
            const double gapB = b.low-(-m.y*intersection.x+m.x*intersection.y);
            if (std::abs(gapA)>std::max(4*tolerance,.025*(a.high-a.low)) ||
                std::abs(gapB)>std::max(4*tolerance,.025*(b.high-b.low))) continue;
            if (successor[i]!=absent) {
                successor[i] = absent;
                break;
            }
            successor[i] = j;
            ends[i] = intersection;
        }
    }
    std::vector<std::vector<Vec3>> outlines;
    for (std::size_t start=0; start<walls.size(); ++start) {
        std::vector<Vec3> vertices;
        auto next = start;
        do {
            if (successor[next]==absent) break;
            vertices.push_back(ends[next]);
            next = successor[next];
        } while (next > start && vertices.size()<walls.size());
        if (next!=start || vertices.size()<3) continue;
        BoundaryLoop loop;
        for (const auto& p : vertices) loop.push_back({p.x,p.y});
        if (boundaryArea(loop)*(inside ? -1 : 1)<=tolerance*tolerance || !simpleBoundary(loop)) continue;
        outlines.push_back(std::move(vertices));
    }
    return outlines;
}

bool insideFiniteOutline(const std::vector<Vec3>& polygon, Vec3 p, double margin) {
    bool inside = false;
    double distance = INFINITY;
    p.z = 0;
    for (std::size_t i=0; i<polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i+1)%polygon.size()];
        const auto edge = subtract(b,a);
        const auto delta = subtract(p,a);
        const double t = std::clamp(dot(delta,edge)/dot(edge,edge),0.0,1.0);
        distance = std::min(distance,length(subtract(delta,scale(edge,t))));
        if ((a.y>p.y)!=(b.y>p.y) && p.x < a.x+(p.y-a.y)*edge.x/edge.y) inside = !inside;
    }
    return inside ? distance>=-margin : distance<=margin;
}

struct ProfileVoid {
    std::vector<modeling::ProfileCurve2D> curves;
    std::vector<Vec3> polygon;
    std::optional<Circle> circle;
    ProfileHalfPlane cut{};

    bool contains(Vec3 p, double margin) const {
        if (!circle) return insideFiniteOutline(polygon,p,margin);
        return std::hypot(p.x-circle->x,p.y-circle->y)<=circle->radius+margin &&
               dot(cut.normal,p)<=cut.offset+margin;
    }
};

bool observedVoidWalls(const ProfileVoid& hole, const std::vector<LocalPoint>& points,
                       double height, double tolerance) {
    constexpr int sectors = 8, layers = 3;
    std::vector<std::bitset<sectors*layers>> bins(hole.curves.size());
    std::size_t contradictions = 0;
    for (const auto& p : points) {
        const auto q = p.position;
        if (q.z < -tolerance || q.z > height+tolerance || !hole.contains(q,tolerance)) continue;
        if (std::abs(p.normal.z)>=.95 &&
            (std::abs(q.z)<=tolerance || std::abs(q.z-height)<=tolerance) &&
            hole.contains(q,-2*tolerance) && ++contradictions>2) return false;
        const int axial = std::clamp(int(q.z*layers/height),0,layers-1);
        for (std::size_t i=0; i<hole.curves.size(); ++i) {
            const double u = profileWallCoordinate(hole.curves[i],q,scale(p.normal,-1),tolerance);
            if (u<0 || u>1) continue;
            const int along = std::min(sectors-1,int(u*sectors));
            bins[i].set(along*layers+axial);
        }
    }
    return std::all_of(bins.begin(),bins.end(),[](const auto& wall) {
        if (wall.count()<sectors*layers*.85) return false;
        for (int i=0; i<sectors; ++i)
            if (!wall.test(i*layers) && !wall.test(i*layers+1) && !wall.test(i*layers+2)) return false;
        return true;
    });
}

std::optional<ProfileVoid> clippedCircularVoid(const Circle& circle, const std::vector<FiniteWall>& walls,
        const std::vector<LocalPoint>& points, double height, double tolerance) {
    const Vec3 center{circle.x,circle.y,0};
    for (const auto& wall : walls) {
        const auto n = scale(wall.plane.normal,-1);
        const double offset = -wall.plane.offset-dot(n,center);
        if (std::abs(offset)>=circle.radius-4*tolerance) continue;
        const auto tangent = Vec3{-wall.plane.normal.y,wall.plane.normal.x,0};
        const double middle = dot(tangent,center);
        const double half = std::sqrt((circle.radius-offset)*(circle.radius+offset));
        const double endpointTolerance = std::max(4*tolerance,.025*2*half);
        if (std::abs(middle-half-wall.low)>endpointTolerance ||
            std::abs(middle+half-wall.high)>endpointTolerance) continue;
        std::size_t outsideArc = 0;
        for (const auto& p : points) {
            if (p.position.z < -tolerance || p.position.z > height+tolerance ||
                dot(n,p.position) <= -wall.plane.offset+tolerance) continue;
            const Vec3 radial{p.position.x-circle.x,p.position.y-circle.y,0};
            if (std::abs(length(radial)-circle.radius)<=tolerance &&
                dot(radial,p.normal)<=-.98*circle.radius && ++outsideArc>=24) break;
        }
        if (outsideArc>=24) continue;
        auto curves = circularOutline(circle.radius,{{n,offset}},tolerance);
        if (curves.empty()) continue;
        for (auto& c : curves) {
            c.x1 += circle.x;
            c.x2 += circle.x;
            c.centerX += circle.x;
            c.y1 += circle.y;
            c.y2 += circle.y;
            c.centerY += circle.y;
        }
        ProfileVoid hole{std::move(curves),{},circle,{n,-wall.plane.offset}};
        if (observedVoidWalls(hole,points,height,tolerance)) return hole;
    }
    return std::nullopt;
}

double angleAlongArc(const CornerArc& arc, double dx, double dy) {
    return std::remainder(std::atan2(dy,dx) - arc.angle - arc.sweep * .5, 2*pi) + arc.sweep * .5;
}

bool arcSupportsPoint(const CornerArc& arc, const LocalPoint& point, double tolerance) {
    const double dx = point.position.x - arc.center.x, dy = point.position.y - arc.center.y;
    const double r = std::hypot(dx,dy);
    if (std::abs(r-arc.radius) > tolerance || dx*point.normal.x+dy*point.normal.y < r*.95) return false;
    const double angle = angleAlongArc(arc,dx,dy);
    return angle >= -tolerance/arc.radius && angle <= arc.sweep+tolerance/arc.radius;
}

Corners roundedCorners(const std::vector<Vec3>& polygon, const std::vector<Circle>& cylinders,
                       const std::vector<LocalPoint>& points, double height, double tolerance) {
    Corners corners(polygon.size());
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto vertex = polygon[i];
        const auto before = subtract(vertex, polygon[(i+polygon.size()-1)%polygon.size()]);
        const auto after = subtract(polygon[(i+1)%polygon.size()], vertex);
        if (std::min(length(before),length(after)) <= tolerance * 2) continue;
        const auto d1 = normalized(before), d2 = normalized(after);
        const Vec3 n1{d1.y,-d1.x,0}, n2{d2.y,-d2.x,0};
        const double det = cross(n1,n2).z;
        if (det <= .1) continue;
        for (const auto& cylinder : cylinders) {
            const Vec3 guess{cylinder.x,cylinder.y,0};
            const double r = cylinder.radius;
            const double a = dot(n1, subtract(vertex,guess))-r, b = dot(n2, subtract(vertex,guess))-r;
            if (std::abs(a) > tolerance || std::abs(b) > tolerance) continue;
            // Constrain tangency to both observed support lines so the
            // resulting analytic arc and straight segments share endpoints.
            const auto center = add(guess, {(a*n2.y-b*n1.y)/det,(n1.x*b-n2.x*a)/det,0});
            const auto start = add(center,scale(n1,r)), end = add(center,scale(n2,r));
            const double beforeTrim = dot(subtract(vertex,start),d1), afterTrim = dot(subtract(end,vertex),d2);
            if (beforeTrim <= tolerance || beforeTrim > length(before)+tolerance ||
                afterTrim <= tolerance || afterTrim > length(after)+tolerance) continue;
            const CornerArc arc{center,start,end,r,std::atan2(n1.y,n1.x),std::atan2(det,dot(n1,n2))};
            constexpr int sectors = 8, layers = 3;
            std::bitset<sectors * layers> bins;
            for (const auto& p : points) {
                if (p.position.z < -tolerance || p.position.z > height+tolerance || !arcSupportsPoint(arc,p,tolerance)) continue;
                const auto v = subtract(p.position, center);
                const auto u = std::clamp(int(angleAlongArc(arc,v.x,v.y)*sectors/arc.sweep),0,sectors-1);
                const auto t = std::clamp(int(p.position.z*layers/height),0,layers-1);
                bins.set(u*layers+t);
            }
            if (bins.count() >= sectors*layers*.85) { corners[i] = arc; break; }
        }
    }
    return corners;
}

bool cylindricalWall(const Circle& hole, const std::vector<LocalPoint>& points, const std::vector<Side>& sides,
                     double start, double end, double tolerance, bool corroborated, bool exterior = false) {
    constexpr int sectors = 16, layers = 4;
    const double height = end - start;
    std::bitset<sectors * layers> bins;
    std::bitset<sectors> around;
    std::bitset<sectors> expected;
    for (int u = 0; u < sectors; ++u) {
        const double angle = -pi + (u + .5) * 2 * pi / sectors;
        expected.set(u, insideOutline(sides, {hole.x + hole.radius * std::cos(angle),
                                              hole.y + hole.radius * std::sin(angle), 0}, tolerance));
    }
    std::size_t count = 0;
    double low = end, high = start;
    for (const auto& p : points) {
        const double dx = p.position.x - hole.x, dy = p.position.y - hole.y;
        const double radius = std::hypot(dx, dy);
        if (std::abs(radius - hole.radius) > tolerance || radius <= tolerance ||
            (exterior ? 1 : -1) * (dx * p.normal.x + dy * p.normal.y) < radius * .95 ||
            !insideOutline(sides, p.position, tolerance) ||
            p.position.z < start - tolerance || p.position.z > end + tolerance) continue;
        const auto u = std::min(sectors - 1, int((std::atan2(dy, dx) + pi) * sectors / (2 * pi)));
        const auto v = std::clamp(int((p.position.z-start) * layers / height), 0, layers - 1);
        bins.set(u * layers + v);
        around.set(u);
        low = std::min(low, p.position.z);
        high = std::max(high, p.position.z);
        ++count;
    }
    if (count < 48 || low > start + height * .1 || high < end - height * .1 || expected.count() < 3) return false;
    if (corroborated && expected.all()) {
        // Cap openings or a drill tip corroborate the angular extent, while
        // the sampled cylinder must still span its full axial interval.
        std::bitset<layers> occupiedLayers;
        for (int u = 0; u < sectors; ++u) for (int v = 0; v < layers; ++v)
            if (bins.test(u * layers + v)) occupiedLayers.set(v);
        return around.count() >= 4 && occupiedLayers.all();
    }
    std::size_t covered = 0;
    for (int u = 0; u < sectors; ++u) if (expected.test(u))
        for (int v = 0; v < layers; ++v) covered += bins.test(u * layers + v);
    // Notches at the outline need observations only where the bore meets material.
    return (around & expected) == expected && covered >= expected.count() * layers * .9;
}

double sectionRadius(const modeling::AxialSection& section, double z) {
    const double t = std::clamp((z-section.start)/(section.end-section.start), 0.0, 1.0);
    return section.startRadius + t*(section.endRadius-section.startRadius);
}

bool boreSurface(const modeling::AxialSection& section, const LocalPoint& p, double radius, double tolerance) {
    if (radius <= 1e-12) return false;
    const double slope = (section.endRadius-section.startRadius)/(section.end-section.start);
    const double norm = std::hypot(1.0, slope);
    const double dx = p.position.x-section.centerX, dy = p.position.y-section.centerY;
    return std::abs(radius-sectionRadius(section,p.position.z)) <= tolerance*norm &&
        (-(dx*p.normal.x+dy*p.normal.y)/radius+slope*p.normal.z)/norm >= (slope == 0 ? .95 : .98);
}

bool observedSides(const std::vector<Vec3>& polygon, const Corners& corners, const std::vector<LocalPoint>& points,
                   double height, double tolerance) {
    constexpr int along = 8, vertical = 3;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto next = (i + 1) % polygon.size();
        const auto start = corners[i] ? corners[i]->end : polygon[i];
        const auto end = corners[next] ? corners[next]->start : polygon[next];
        const auto edge = subtract(end, start);
        if (dot(edge, subtract(polygon[next],polygon[i])) <= 0) return false;
        const double span = length(edge);
        if (span <= 2 * tolerance) return false;
        const auto direction = scale(edge, 1 / span);
        const Vec3 outward{direction.y,-direction.x,0};
        std::bitset<along * vertical> bins;
        for (const auto& p : points) {
            const auto delta = subtract(p.position, start);
            if (std::abs(dot(outward, delta)) > tolerance || dot(outward, p.normal) < .95 ||
                p.position.z < -tolerance || p.position.z > height + tolerance) continue;
            const double s = dot(delta, direction);
            if (s < -tolerance || s > span + tolerance) continue;
            const int u = std::clamp(int(s * along / span), 0, along - 1);
            const int v = std::clamp(int(p.position.z * vertical / height), 0, vertical - 1);
            bins.set(u * vertical + v);
        }
        if (bins.count() < along * vertical * .75) return false;
        int missing = -1;
        for (int u = 0; u < along; ++u) {
            bool observed = false;
            for (int v = 0; v < vertical; ++v) observed |= bins.test(u * vertical + v);
            if (observed) continue;
            if (missing >= 0 || (u != 0 && u != along-1)) return false;
            missing = u;
        }
        if (missing < 0) continue;
        // The adjoining side and both caps must reach the corner before a
        // single unsampled end strip can be extended to the intersection.
        const auto cornerIndex = missing == 0 ? i : next;
        if (corners[cornerIndex]) return false;
        const auto vertex = polygon[cornerIndex];
        const auto adjacent = missing == 0
            ? subtract(vertex,polygon[(i+polygon.size()-1)%polygon.size()])
            : subtract(polygon[(next+1)%polygon.size()],vertex);
        const double adjacentSpan = length(adjacent);
        if (adjacentSpan <= 2*tolerance) return false;
        const auto n = normalized(Vec3{adjacent.y,-adjacent.x,0});
        const double cornerReach = std::min(span,adjacentSpan)/along;
        std::bitset<vertical> neighbor;
        bool lowCap = false, highCap = false;
        for (const auto& p : points) {
            const auto delta = subtract(p.position,vertex);
            if (std::hypot(delta.x,delta.y) > cornerReach || delta.z < -tolerance || delta.z > height+tolerance) continue;
            lowCap |= std::abs(delta.z) <= tolerance && p.normal.z <= -.95;
            highCap |= std::abs(delta.z-height) <= tolerance && p.normal.z >= .95;
            if (std::abs(dot(n,delta)) <= tolerance && dot(n,p.normal) >= .95)
                neighbor.set(std::clamp(int(delta.z*vertical/height),0,vertical-1));
        }
        if (!lowCap || !highCap || !neighbor.all()) return false;
    }
    return true;
}

bool hasOpening(const Circle& h, const FiniteSurfacePatch& cap, Vec3 origin, Vec3 x, Vec3 y, double tolerance) {
    const double trimTolerance = std::min(h.radius * .45, std::max(tolerance * 4, cap.spacing * 2));
    const double circleArea = pi * h.radius * h.radius;
    for (const auto& loop : cap.loops) {
        if (loop.size() < 6) continue;
        double area = 0, winding = 0;
        std::size_t near = 0;
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const auto u = subtract(loop[i], origin), v = subtract(loop[(i + 1) % loop.size()], origin);
            area += dot(cross(u,v), cap.normal);
            const Vec3 a{dot(u,x)-h.x,dot(u,y)-h.y,0}, b{dot(v,x)-h.x,dot(v,y)-h.y,0};
            near += std::abs(length(a)-h.radius) <= trimTolerance;
            winding += std::atan2(cross(a,b).z,dot(a,b));
        }
        // The inner boundary is recovered from the nearest material samples,
        // so its polygon can sit outside the independently fitted wall.
        const double voidArea = -area / 2;
        if (voidArea >= circleArea * .6 && voidArea <= circleArea * 2.25 &&
            std::abs(winding) > pi && near >= loop.size() * .6) return true;
    }
    return false;
}

std::vector<Circle> cylinderCandidates(const std::vector<LocalPoint>& points, const std::vector<Side>& sides,
                                      double height, double tolerance, std::size_t limit) {
    std::vector<PointSample> residual;
    for (const auto& p : points) {
        if (std::abs(p.normal.z) > .1 || p.position.z < -tolerance || p.position.z > height + tolerance) continue;
        if (std::any_of(sides.begin(), sides.end(), [&](const Side& side) {
                return std::abs(boundaryOffset(side, p.position)) <= tolerance && dot(side.normal, p.normal) >= .95;
            })) continue;
        residual.push_back({p.id, p.position, p.normal, 1.0, std::nullopt, true});
    }
    if (residual.size() < 48) return {};
    PrimitiveDetectionOptions options;
    options.detectPlanes = options.detectSpheres = options.detectCones = options.detectTori = false;
    options.cylinder.minimumSupportPoints = 48;
    options.cylinder.minimumNormalAlignment = .98;
    options.cylinder.distanceThreshold = tolerance;
    options.cylinder.minimumRadius = tolerance * 4;
    options.cylinder.maximumCylinders = limit;
    PrimitiveDetectionResult result;
    std::string error;
    if (!detectPrimitiveEvidence(PointStore(std::move(residual)), options, result, error)) return {};
    std::vector<Circle> cylinders;
    for (const auto& e : result.evidence) {
        const auto& cylinder = std::get<CylinderEvidence>(e);
        const auto& c = cylinder.cylinder;
        if (std::abs(c.axisDirection.z) < .999 ||
            std::hypot(c.axisDirection.x, c.axisDirection.y) * height > tolerance) continue;
        const auto center = add(c.axisPoint, scale(c.axisDirection, (height * .5 - c.axisPoint.z) / c.axisDirection.z));
        cylinders.push_back({center.x, center.y, c.radius});
    }
    return cylinders;
}

std::vector<ConeEquation> boreConeCandidates(const std::vector<LocalPoint>& points, double height, double tolerance,
                                            std::size_t limit) {
    std::vector<PointSample> residual;
    for (const auto& p : points) {
        if (p.position.z <= tolerance || p.position.z >= height-tolerance ||
            std::abs(p.normal.z) < .1 || std::abs(p.normal.z) > .9999) continue;
        residual.push_back({p.id, p.position, p.normal, 1.0, std::nullopt, true});
    }
    if (residual.size() < 48) return {};
    PrimitiveDetectionOptions options;
    options.detectPlanes = options.detectCylinders = options.detectSpheres = options.detectTori = false;
    options.cone.minimumSupportPoints = 48;
    options.cone.minimumNormalAlignment = .98;
    options.cone.distanceThreshold = tolerance;
    options.cone.maximumCones = limit;
    PrimitiveDetectionResult result;
    std::string error;
    if (!detectPrimitiveEvidence(PointStore(std::move(residual)), options, result, error)) return {};
    std::vector<ConeEquation> cones;
    for (const auto& evidence : result.evidence) cones.push_back(std::get<ConeEvidence>(evidence).cone);
    return cones;
}

bool supportedConeSection(const modeling::AxialSection& s, const std::vector<LocalPoint>& points,
                          double tolerance, bool openingObserved = false) {
    constexpr int sectors = 12, layers = 3;
    std::bitset<sectors * layers> bins;
    std::bitset<sectors> around;
    std::size_t count = 0;
    for (const auto& p : points) {
        if (p.position.z < s.start || p.position.z > s.end) continue;
        const double dx = p.position.x-s.centerX, dy = p.position.y-s.centerY;
        if (!boreSurface(s,p,std::hypot(dx,dy),tolerance)) continue;
        const double radius = sectionRadius(s,p.position.z);
        const double fraction = (p.position.z-s.start)/(s.end-s.start)*(radius+s.startRadius)/(s.endRadius+s.startRadius);
        const int u = std::min(sectors-1,int((std::atan2(dy,dx)+pi)*sectors/(2*pi)));
        const int v = std::clamp(int(fraction*layers),0,layers-1);
        bins.set(u*layers+v);
        around.set(u);
        ++count;
    }
    if (count<48) return false;
    if (around.all() && bins.count()>=sectors*layers*.85) return true;
    // The independently observed opening can corroborate a partially
    // sampled countersink if several sectors span its entire depth.
    if (!openingObserved || around.count()<=sectors/2) return false;
    for (int layer=0; layer<layers; ++layer) {
        int covered = 0;
        for (int sector=0; sector<sectors; ++sector) covered += bins.test(sector*layers+layer);
        if (covered<sectors/2) return false;
    }
    return true;
}

bool interiorCircle(const Circle& circle, const std::vector<Side>& sides, double tolerance) {
    return std::none_of(sides.begin(),sides.end(),[&](const Side& side) {
        return boundaryOffset(side,{circle.x,circle.y,0}) > -circle.radius-tolerance;
    });
}

bool coaxialBoreCone(const Circle& hole, const ConeEquation& cone, double height, double tolerance) {
    const auto axis = cone.axisDirection;
    if (std::abs(axis.z) < .999 || std::hypot(axis.x,axis.y)*height > tolerance) return false;
    const auto middle = add(cone.apex,scale(axis,(height*.5-cone.apex.z)/axis.z));
    return std::hypot(middle.x-hole.x,middle.y-hole.y) <= tolerance;
}

std::optional<Bore> blindBore(const Circle& hole, const std::vector<ConeEquation>& cones,
                             const std::vector<LocalPoint>& points, const std::vector<Side>& sides,
                             double height, double tolerance, const bool openings[2]) {
    // Restrict blind cuts to the interior. A blind notch on an outer edge
    // would need additional intersections with the outline to verify closure.
    if (!interiorCircle(hole,sides,tolerance)) return std::nullopt;
    for (const auto& cone : cones) {
        if (!coaxialBoreCone(hole,cone,height,tolerance)) continue;
        const int cap = cone.axisDirection.z > 0 ? 1 : 0;
        const double depth = hole.radius/std::tan(cone.openingAngleRadians);
        const double seam = cone.apex.z + (cap ? depth : -depth);
        if (depth <= 4*tolerance || cone.apex.z <= 4*tolerance || cone.apex.z >= height-4*tolerance ||
            seam <= 4*tolerance || seam >= height-4*tolerance || openings[1-cap]) continue;
        const auto tip = boreSection(hole,cone.apex.z,seam,0,hole.radius);
        if (!supportedConeSection(tip,points,tolerance)) continue;
        const auto wall = boreSection(hole,seam,cap ? height : 0,hole.radius,hole.radius);
        if (cylindricalWall(hole,points,sides,wall.start,wall.end,tolerance,openings[cap]))
            return cap ? Bore{hole,{tip,wall}} : Bore{hole,{wall,tip}};
    }
    return std::nullopt;
}

std::optional<modeling::AxialSection> countersink(const Circle& hole, const std::vector<ConeEquation>& cones,
    const std::vector<LocalPoint>& points, const std::vector<Side>& sides, int cap, double height, double tolerance,
    const std::function<bool(const Circle&)>& openingObserved = {}) {
    for (const auto& cone : cones) {
        if ((cone.axisDirection.z > 0) != (cap == 1) || !coaxialBoreCone(hole,cone,height,tolerance)) continue;
        const double face = cap ? height : 0;
        const double slope = std::tan(cone.openingAngleRadians);
        const double seam = cone.apex.z + (cap ? 1 : -1)*hole.radius/slope;
        if (seam <= 4*tolerance || seam >= height-4*tolerance) continue;
        const double openingRadius = std::abs(face-cone.apex.z)*slope;
        if (openingRadius <= hole.radius+4*tolerance ||
            !interiorCircle({hole.x,hole.y,openingRadius},sides,tolerance)) continue;
        const auto section = boreSection(hole,seam,face,hole.radius,openingRadius);
        const bool corroborated = openingObserved && openingObserved({hole.x,hole.y,openingRadius});
        if (supportedConeSection(section,points,tolerance,corroborated)) return section;
    }
    return std::nullopt;
}

std::optional<Bore> countersunkBore(const Circle& hole, const std::vector<ConeEquation>& cones,
    const std::vector<LocalPoint>& points, const std::vector<Side>& sides, double height, double tolerance,
    const FiniteSurfacePatch& bottom, const FiniteSurfacePatch& top, Vec3 x, Vec3 y) {
    const auto observedOpening = [&](const Circle& circle, const FiniteSurfacePatch& face, int cap) {
        std::bitset<12> around;
        const auto end = cap ? height : 0;
        const auto sign = cap ? 1 : -1;
        const auto reach = std::max(4*tolerance,2*face.spacing);
        std::size_t contradictions = 0;
        for (const auto& p : points) {
            if (std::abs(p.position.z-end)>tolerance || p.normal.z*sign<.98) continue;
            const auto dx = p.position.x-circle.x, dy = p.position.y-circle.y;
            const auto r = std::hypot(dx,dy);
            if (r<circle.radius-2*tolerance && ++contradictions>4) return false;
            if (r<circle.radius-tolerance || r>circle.radius+reach) continue;
            const int sector = std::clamp(int((std::atan2(dy,dx)+pi)*12/(2*pi)),0,11);
            around.set(sector);
        }
        return around.all() || hasOpening(circle,face,bottom.origin,x,y,tolerance);
    };
    const auto countersinks = std::array{
        countersink(hole,cones,points,sides,0,height,tolerance,[&](const auto& circle) {
            return observedOpening(circle,bottom,0);
        }),
        countersink(hole,cones,points,sides,1,height,tolerance,[&](const auto& circle) {
            return observedOpening(circle,top,1);
        })};
    if (!countersinks[0] && !countersinks[1]) return std::nullopt;
    const double low = countersinks[0] ? countersinks[0]->end : 0;
    const double high = countersinks[1] ? countersinks[1]->start : height;
    if (high-low <= 8*tolerance) return std::nullopt;
    const bool supportedEnds = (countersinks[0] || hasOpening(hole,bottom,bottom.origin,x,y,tolerance)) &&
        (countersinks[1] || hasOpening(hole,top,bottom.origin,x,y,tolerance));
    if (!cylindricalWall(hole,points,sides,low,high,tolerance,supportedEnds)) return std::nullopt;
    Bore bore{hole,{}};
    if (countersinks[0]) bore.sections.push_back(*countersinks[0]);
    bore.sections.push_back(boreSection(hole,low,high,hole.radius,hole.radius));
    if (countersinks[1]) bore.sections.push_back(*countersinks[1]);
    return bore;
}

std::vector<double> observedShoulders(const Circle& wide, double inner, int direction,
        const std::vector<LocalPoint>& points, double height, double tolerance) {
    struct Hit { double z, radius, angle; };
    std::vector<Hit> hits;
    for (const auto& p : points) {
        if (p.normal.z*direction < .98 || p.position.z <= 4*tolerance || p.position.z >= height-4*tolerance) continue;
        const double dx = p.position.x-wide.x, dy = p.position.y-wide.y, r = std::hypot(dx,dy);
        if (r >= inner && r <= wide.radius) hits.push_back({p.position.z,r,std::atan2(dy,dx)});
    }
    std::sort(hits.begin(),hits.end(),[](auto a, auto b) { return a.z < b.z; });
    constexpr int sectors = 12, layers = 3;
    std::vector<double> shoulders;
    for (std::size_t i=0; i<hits.size();) {
        std::size_t j=i;
        double sum = 0;
        std::bitset<sectors*layers> bins;
        std::bitset<sectors> around;
        while (j<hits.size() && hits[j].z-hits[i].z<=2*tolerance) {
            const auto& hit = hits[j++];
            const int u = std::min(sectors-1,int((hit.angle+pi)*sectors/(2*pi)));
            const double area = (hit.radius-inner)/(wide.radius-inner)*(hit.radius+inner)/(wide.radius+inner);
            const int v = std::clamp(int(area*layers),0,layers-1);
            bins.set(u*layers+v);
            around.set(u);
            sum += hit.z;
        }
        // Both complete cylindrical walls are checked separately against this
        // level. They bound the annulus across a limited gap in its sampling.
        if (j-i>=64 && around.count()>=sectors*3/4 && bins.count()>=around.count()*layers*.85)
            shoulders.push_back(sum/(j-i));
        i=j;
    }
    return shoulders;
}

std::optional<Bore> steppedBore(const Circle& wide, const std::vector<Circle>& cylinders,
        const std::vector<ConeEquation>& cones, const std::vector<LocalPoint>& points,
        const std::vector<Side>& sides, double height, double tolerance) {
    if (!interiorCircle(wide,sides,tolerance)) return std::nullopt;
    for (const auto& candidate : cylinders) {
        if (wide.radius-candidate.radius<=4*tolerance || std::hypot(wide.x-candidate.x,wide.y-candidate.y)>tolerance) continue;
        const Circle narrow{wide.x,wide.y,candidate.radius};
        for (int wideCap : {0,1}) {
            const auto levels = observedShoulders(wide,narrow.radius,wideCap ? 1 : -1,points,height,tolerance);
            if (levels.empty()) continue;
            const Circle& lower = wideCap ? narrow : wide;
            const Circle& upper = wideCap ? wide : narrow;
            const auto mouthLow = countersink(lower,cones,points,sides,0,height,tolerance);
            const auto mouthHigh = countersink(upper,cones,points,sides,1,height,tolerance);
            const double low = mouthLow ? mouthLow->end : 0;
            const double high = mouthHigh ? mouthHigh->start : height;
            for (const double shoulder : levels) {
                if (shoulder-low<=4*tolerance || high-shoulder<=4*tolerance ||
                    !cylindricalWall(lower,points,sides,low,shoulder,tolerance,false) ||
                    !cylindricalWall(upper,points,sides,shoulder,high,tolerance,false)) continue;
                Bore bore{wide,{}};
                if (mouthLow) bore.sections.push_back(*mouthLow);
                bore.sections.push_back(boreSection(lower,low,shoulder,lower.radius,lower.radius));
                bore.sections.push_back(boreSection(upper,shoulder,high,upper.radius,upper.radius));
                if (mouthHigh) bore.sections.push_back(*mouthHigh);
                return bore;
            }
        }
    }
    return std::nullopt;
}

std::vector<modeling::AxialSection> annularGrooves(const Circle& exterior, const std::vector<Circle>& cylinders,
        const std::vector<LocalPoint>& points, double height, double tolerance) {
    std::vector<modeling::AxialSection> grooves;
    for (const auto& outer : cylinders) {
        if (std::hypot(outer.x-exterior.x,outer.y-exterior.y)+outer.radius >= exterior.radius-tolerance) continue;
        for (const auto& candidate : cylinders) {
            if (outer.radius-candidate.radius <= 4*tolerance ||
                std::hypot(outer.x-candidate.x,outer.y-candidate.y) > tolerance) continue;
            const Circle inner{outer.x,outer.y,candidate.radius};
            for (const int direction : {-1,1}) {
                for (const auto floor : observedShoulders(outer,inner.radius,direction,points,height,tolerance)) {
                    const double start = direction > 0 ? floor : 0;
                    const double end = direction > 0 ? height : floor;
                    if (end-start <= 4*tolerance ||
                        !cylindricalWall(outer,points,{},start,end,tolerance,false) ||
                        !cylindricalWall(inner,points,{},start,end,tolerance,false,true)) continue;
                    auto section = boreSection(outer,start,end,outer.radius,outer.radius);
                    section.innerStartRadius = section.innerEndRadius = inner.radius;
                    grooves.push_back(section);
                }
            }
        }
    }
    return grooves;
}

struct Profile {
    modeling::ProfileBodyParams body;
    std::unordered_set<PointId> support;
};

struct RimFaceEvidence {
    static constexpr int grid = 6;
    TopoDS_Face face;
    BRepAdaptor_Surface surface;
    Bnd_Box bounds;
    double u0, u1, v0, v1;
    std::bitset<grid*grid> expected, observed;
    std::size_t support = 0;

    RimFaceEvidence(const TopoDS_Face& f, double tolerance)
        : face(f), surface(f), u0(surface.FirstUParameter()), u1(surface.LastUParameter()),
          v0(surface.FirstVParameter()), v1(surface.LastVParameter()) {
        BRepBndLib::AddOptimal(f,bounds,false,false);
        bounds.Enlarge(tolerance);
        for (int i=0; i<grid; ++i) for (int j=0; j<grid; ++j) {
            const gp_Pnt2d uv(u0+(i+.5)*(u1-u0)/grid,v0+(j+.5)*(v1-v0)/grid);
            if (BRepClass_FaceClassifier(face,uv,1e-8).State()==TopAbs_IN) expected.set(i*grid+j);
        }
    }

    bool match(gp_Pnt point, Vec3 direction, double tolerance, bool record = true) {
        if (bounds.IsOut(point)) return false;
        double u=0,v=0;
        switch (surface.GetType()) {
        case GeomAbs_Plane: ElSLib::Parameters(surface.Plane(),point,u,v); break;
        case GeomAbs_Cylinder: ElSLib::Parameters(surface.Cylinder(),point,u,v); break;
        case GeomAbs_Torus: ElSLib::Parameters(surface.Torus(),point,u,v); break;
        default: return false;
        }
        const auto wrap = [](double value, double middle) {
            return middle+std::remainder(value-middle,2*pi);
        };
        if (surface.IsUPeriodic()) u = wrap(u,(u0+u1)*.5);
        if (surface.IsVPeriodic()) v = wrap(v,(v0+v1)*.5);
        gp_Pnt closest;
        gp_Vec du,dv;
        surface.D1(u,v,closest,du,dv);
        if (point.SquareDistance(closest)>tolerance*tolerance) return false;
        const auto normal = du.Crossed(dv);
        const double length = normal.Magnitude();
        const double sign = face.Orientation()==TopAbs_REVERSED ? -1 : 1;
        if (length<1e-12 || normal.Dot(gp_Vec(direction.x,direction.y,direction.z))*sign<.98*length)
            return false;
        const auto state = BRepClass_FaceClassifier(face,closest,tolerance).State();
        if (state!=TopAbs_IN && state!=TopAbs_ON) return false;
        const auto i = std::clamp(int((u-u0)*grid/(u1-u0)),0,grid-1);
        const auto j = std::clamp(int((v-v0)*grid/(v1-v0)),0,grid-1);
        if (record) {
            observed.set(i*grid+j);
            ++support;
        }
        return true;
    }

    bool sufficientlyObserved() const {
        const auto confirmed = observed&expected;
        if (support>=16 && confirmed.count()>=expected.count()*.7) return true;
        // The toroidal segment of the same rounded rim fixes the radius
        // where the adjoining straight blend has gaps along its length.
        if (surface.GetType()!=GeomAbs_Cylinder || std::abs(surface.Cylinder().Axis().Direction().Z())>1e-4 ||
            support<48 || confirmed.count()<expected.count()*.55) return false;
        bool starts = false, ends = false;
        for (int u=0; u<grid; ++u) {
            bool occupied = false;
            for (int v=0; v<grid; ++v) occupied |= confirmed.test(u*grid+v);
            if (!occupied) return false;
            starts |= confirmed.test(u*grid);
            ends |= confirmed.test(u*grid+grid-1);
        }
        return starts && ends;
    }
};

struct ChannelInterior {
    double width, height, gauge, bend;

    bool contains(Vec3 p, double margin) const {
        if (p.x<=gauge+margin || p.x>=width-gauge-margin || p.y>=height-gauge-margin) return false;
        const auto knee = height-bend;
        if (p.y<=knee || (p.x>=bend && p.x<=width-bend)) return true;
        const auto center = p.x<bend ? bend : width-bend;
        return std::hypot(p.x-center,p.y-knee)<bend-gauge-margin;
    }
};

bool hasChannelEnds(const std::vector<LocalPoint>& points, const ChannelInterior& section,
                    const std::vector<modeling::ProfileTrimPlane>& ends, double tolerance) {
    for (const auto& end : ends) {
        std::size_t supported = 0, contradictory = 0;
        for (const auto& point : points) {
            if (dot(point.normal,end.outward)<.995 ||
                std::abs(dot(subtract(point.position,end.point),end.outward))>tolerance) continue;
            const auto q = point.position;
            if (q.x < -tolerance || q.x>section.width+tolerance ||
                q.y < -tolerance || q.y>section.height+tolerance) continue;
            if (section.contains(q,4*tolerance)) ++contradictory;
            else ++supported;
        }
        if (supported<48 || contradictory>.05*supported) return false;
    }
    return true;
}

bool hasChannelWalls(const std::vector<LocalPoint>& points, const ChannelInterior& section,
                     const std::vector<modeling::ProfileCurve2D>& outline, double depth, double tolerance) {
    // The central run of a channel follows its measured straight and
    // concentric bend walls. Screen this before building and trimming CAD.
    const auto stride = 1+(points.size()-1)/4096;
    std::size_t eligible = 0, supported = 0;
    for (std::size_t i=0; i<points.size(); i+=stride) {
        const auto& p = points[i];
        const auto q = p.position;
        if (q.z<.2*depth || q.z>.8*depth || std::abs(p.normal.z)>.02 ||
            q.x < -tolerance || q.x>section.width+tolerance ||
            q.y < -tolerance || q.y>section.height+tolerance || section.contains(q,4*tolerance)) continue;
        ++eligible;
        const auto agrees = [&](const auto& curve) {
            const auto sign = curve.circular && curve.sweepAngle<0 ? -1 : 1;
            const auto normal = scale(p.normal,sign);
            const auto coordinate = profileWallCoordinate(curve,q,normal,2*tolerance);
            if (coordinate>=0 && coordinate<=1) return true;
            const auto contact = profileWallCoordinate(curve,q,scale(normal,-1),tolerance);
            return contact>=0 && contact<=1;
        };
        supported += std::any_of(outline.begin(),outline.end(),agrees);
    }
    return eligible>=64 && supported>=.94*eligible;
}

bool verifyAnalyticProfile(Profile& fitted, const std::vector<LocalPoint>& points,
    const std::vector<FiniteSurfacePatch>& patches, double tolerance,
    std::optional<ChannelInterior> channel = std::nullopt) {
    TopoDS_Shape body;
    std::string error;
    if (!modeling::buildProfileBody(fitted.body,body,error)) return false;
    Bnd_Box bounds;
    BRepBndLib::AddOptimal(body,bounds,false,false);
    bounds.Enlarge(tolerance);
    std::vector<RimFaceEvidence> faces;
    for (TopExp_Explorer it(body,TopAbs_FACE); it.More(); it.Next())
        faces.emplace_back(TopoDS::Face(it.Current()),tolerance);
    std::unordered_set<PointId> outside;
    for (const auto& sample : points) {
        const auto& p = sample.position;
        const gp_Pnt point(p.x,p.y,p.z);
        if (bounds.IsOut(point)) {
            outside.insert(sample.id);
            continue;
        }
        if (std::any_of(faces.begin(),faces.end(),[&](auto& f) { return f.match(point,sample.normal,tolerance); })) {
            fitted.support.insert(sample.id);
            continue;
        }
        if (!channel) continue;
        if (std::any_of(faces.begin(),faces.end(),[&](auto& f) {
            return f.match(point,scale(sample.normal,-1),tolerance,false);
        })) {
            outside.insert(sample.id);
            continue;
        }
        if (channel->contains(p,4*tolerance)) outside.insert(sample.id);
    }
    const auto eligible = points.size()-outside.size();
    if (eligible<points.size()*.8 || fitted.support.size()<eligible*minimumCoverage) return false;
    if (std::any_of(faces.begin(),faces.end(),[](const auto& f) { return !f.sufficientlyObserved(); })) return false;
    for (const auto& patch : patches) {
        std::size_t included = 0, covered = 0;
        for (const auto id : patch.pointIds) {
            included += outside.count(id)==0;
            covered += fitted.support.count(id);
        }
        if (included>=24 && covered<.85*included) return false;
    }
    return true;
}

std::optional<double> roundedRimRadius(const CornerArc& arc, const std::vector<LocalPoint>& points,
                                       double height, double tolerance, bool top) {
    struct Vote { double radius; int sector, layer; };
    std::vector<Vote> votes;
    const auto sign = top ? 1 : -1;
    for (const auto& p : points) {
        const Vec3 radial{p.position.x-arc.center.x,p.position.y-arc.center.y,0};
        const auto rho = length(radial);
        const auto inset = arc.radius-rho;
        const auto depth = top ? height-p.position.z : p.position.z;
        const auto axial = p.normal.z*sign;
        if (rho<=tolerance || axial<.05 || axial>.999 || inset<0 || depth<0 ||
            std::max(inset,depth)>=height*.35) continue;
        const auto outward = dot(p.normal,scale(radial,1/rho));
        if (outward<.05 || std::hypot(outward,axial)<.9995) continue;
        const auto angle = angleAlongArc(arc,radial.x,radial.y);
        if (angle<0 || angle>arc.sweep) continue;
        const auto radialFactor = 1-outward, axialFactor = 1-axial;
        const auto r = (inset*radialFactor+depth*axialFactor)/
                       (radialFactor*radialFactor+axialFactor*axialFactor);
        if (r<=4*tolerance || r>=std::min(height*.35,arc.radius*.5) ||
            std::abs(r*radialFactor-inset)>tolerance || std::abs(r*axialFactor-depth)>tolerance) continue;
        votes.push_back({r,std::min(7,int(8*angle/arc.sweep)),
                         std::clamp(int(6*std::asin(std::min(1.,axial))/pi),0,2)});
    }
    if (votes.size()<48) return std::nullopt;
    std::sort(votes.begin(),votes.end(),[](const auto& a, const auto& b) { return a.radius<b.radius; });
    std::size_t best = 0;
    std::optional<double> result;
    std::array<int,24> bins{};
    int occupied = 0;
    const auto bin = [](const auto& vote) { return vote.sector*3+vote.layer; };
    for (std::size_t low=0,high=0; low<votes.size(); ++low) {
        while (high<votes.size() && votes[high].radius-votes[low].radius<=2*tolerance)
            occupied += ++bins[bin(votes[high++])]==1;
        if (high-low>best && high-low>=48 && occupied>=20) {
            best = high-low;
            result = votes[(low+high)/2].radius;
        }
        occupied -= --bins[bin(votes[low])]==0;
    }
    return result;
}

void recoverReentrantCorners(const std::vector<Vec3>& polygon, const std::vector<Circle>& cylinders,
    const std::vector<LocalPoint>& points, double height, double tolerance, Corners& corners) {
    for (std::size_t i=0; i<polygon.size(); ++i) {
        if (corners[i]) continue;
        const auto vertex = polygon[i];
        const auto before = subtract(polygon[(i+polygon.size()-1)%polygon.size()],vertex);
        const auto after = subtract(polygon[(i+1)%polygon.size()],vertex);
        const auto u = normalized(before), v = normalized(after);
        const auto sweep = std::atan2(cross(u,v).z,dot(u,v));
        if (sweep>=-.1) continue;
        for (const auto& cylinder : cylinders) {
            const auto r = cylinder.radius;
            if (std::hypot(cylinder.x-vertex.x,cylinder.y-vertex.y)>tolerance ||
                r>=std::min(length(before),length(after))-tolerance) continue;
            const CornerArc arc{vertex,add(vertex,scale(u,r)),add(vertex,scale(v,r)),r,std::atan2(u.y,u.x),sweep};
            std::bitset<24> occupied;
            for (const auto& p : points) {
                const auto radial = subtract(p.position,vertex);
                const auto rho = std::hypot(radial.x,radial.y);
                if (p.position.z<0 || p.position.z>height || std::abs(rho-r)>tolerance ||
                    radial.x*p.normal.x+radial.y*p.normal.y>-.98*rho) continue;
                const auto fraction = angleAlongArc(arc,radial.x,radial.y)/sweep;
                if (fraction<0 || fraction>1) continue;
                occupied.set(std::min(7,int(8*fraction))*3+std::min(2,int(3*p.position.z/height)));
            }
            if (occupied.count()<20) continue;
            corners[i] = arc;
            break;
        }
    }
}

std::optional<Profile> fitRoundedRim(const std::vector<Vec3>& polygon, Corners corners,
    const std::vector<Circle>& cylinders, const std::vector<Side>& sides, const std::vector<LocalPoint>& points,
    const std::vector<FiniteSurfacePatch>& patches, double height, double tolerance) {
    if (polygon.empty() || polygon.size()>12) return std::nullopt;
    Profile fitted;
    auto& body = fitted.body;
    std::vector<std::tuple<std::size_t,double,bool>> rounds;
    for (std::size_t i=0; i<corners.size(); ++i) if (corners[i])
        for (bool top : {false,true})
            if (const auto radius = roundedRimRadius(*corners[i],points,height,tolerance,top))
                rounds.emplace_back(i,*radius,top);
    if (rounds.empty()) return std::nullopt;
    recoverReentrantCorners(polygon,cylinders,points,height,tolerance,corners);
    for (std::size_t i=0; i<polygon.size(); ++i) {
        const auto next = (i+1)%polygon.size();
        if (const auto& arc = corners[i]) {
            for (const auto& [corner,r,top] : rounds)
                if (corner==i) body.rimRounds.push_back({body.outline.size(),r,top});
            body.outline.push_back({true,0,0,0,0,arc->center.x,arc->center.y,arc->radius,arc->angle,arc->sweep});
        }
        const auto a = corners[i] ? corners[i]->end : polygon[i];
        const auto b = corners[next] ? corners[next]->start : polygon[next];
        if (length(subtract(b,a))<=tolerance) return std::nullopt;
        body.outline.push_back({false,a.x,a.y,b.x,b.y});
    }
    for (const auto& hole : cylinders) {
        if (!interiorCircle(hole,sides,tolerance) || !cylindricalWall(hole,points,sides,0,height,tolerance,false)) continue;
        body.cavities.push_back(boreSection(hole,-4*tolerance,height+4*tolerance,hole.radius,hole.radius));
    }
    auto lower = polygon.front(), upper = lower;
    for (const auto& p : polygon) {
        lower.x=std::min(lower.x,p.x); lower.y=std::min(lower.y,p.y);
        upper.x=std::max(upper.x,p.x); upper.y=std::max(upper.y,p.y);
    }
    body.depth = body.sizeZ = height;
    body.referenceSizeX = body.sizeX = upper.x-lower.x;
    body.referenceSizeY = body.sizeY = upper.y-lower.y;
    if (verifyAnalyticProfile(fitted,points,patches,tolerance)) return fitted;
    return std::nullopt;
}

struct ChannelPlane {
    Vec3 normal, point;
    std::size_t support;
};

std::optional<std::pair<double,double>> channelEndRange(const ChannelPlane& first, const ChannelPlane& last,
    Vec3 origin, Vec3 u, Vec3 v, Vec3 axis, double width, double height) {
    const double firstAlong = dot(first.normal,axis);
    const double lastAlong = dot(last.normal,axis);
    double start = INFINITY, end = -INFINITY;
    for (double x : {0.,width}) for (double y : {0.,height}) {
        const auto point = add(origin,add(scale(u,x),scale(v,y)));
        const double low = dot(first.normal,subtract(first.point,point))/firstAlong;
        const double high = dot(last.normal,subtract(last.point,point))/lastAlong;
        if (low>=high) return std::nullopt;
        start = std::min(start,low);
        end = std::max(end,high);
    }
    return std::pair{start,end};
}

std::vector<ChannelPlane> channelPlanes(const std::vector<FiniteSurfacePatch>& patches, double tolerance) {
    std::vector<ChannelPlane> planes;
    for (const auto& patch : patches) {
        if (!patch.planar || !patch.orientationKnown || patch.workingPointIds.size()<100) continue;
        const auto it = std::find_if(planes.begin(),planes.end(),[&](const auto& p) {
            return dot(p.normal,patch.normal)>.99999 && std::abs(dot(p.normal,subtract(p.point,patch.origin)))<tolerance;
        });
        if (it==planes.end()) planes.push_back({patch.normal,patch.origin,patch.workingPointIds.size()});
        else it->support += patch.workingPointIds.size();
    }
    return planes;
}

std::vector<modeling::ProfileCurve2D> channelOutline(double width, double height, double gauge, double bend) {
    std::vector<modeling::ProfileCurve2D> curves;
    const auto line = [&](double x, double y, double a, double b) {
        if (std::hypot(a-x,b-y)>1e-9) curves.push_back({false,x,y,a,b});
    };
    const auto arc = [&](double center, double radius, double angle, double sweep) {
        curves.push_back({true,0,0,0,0,center,height-bend,radius,angle,sweep});
    };
    const auto knee = height-bend;
    line(0,0,gauge,0);
    line(gauge,0,gauge,knee);
    arc(bend,bend-gauge,pi,-pi*.5);
    line(bend,height-gauge,width-bend,height-gauge);
    arc(width-bend,bend-gauge,pi*.5,-pi*.5);
    line(width-gauge,knee,width-gauge,0);
    line(width-gauge,0,width,0);
    line(width,0,width,knee);
    arc(width-bend,bend,0,pi*.5);
    line(width-bend,height,bend,height);
    arc(bend,bend,pi*.5,pi*.5);
    line(0,knee,0,0);
    return curves;
}

std::optional<double> refineChannelBend(const std::vector<LocalPoint>& points, double width, double height,
                                        double gauge, double proposal, double tolerance) {
    std::vector<double> votes;
    std::array<std::size_t,4> support{};
    for (const auto& p : points) {
        if (std::abs(p.normal.z)>.001 || std::abs(p.normal.x)<.08 || std::abs(p.normal.y)<.08) continue;
        const bool left = p.position.x<width*.5;
        const double sign = left ? 1 : -1;
        const Vec3 corner{left ? 0. : width,height,0};
        const auto delta = subtract(Vec3{p.position.x,p.position.y,0},corner);
        const Vec3 normal{p.normal.x,p.normal.y,0};
        for (bool inner : {false,true}) {
            if (normal.x*sign*(inner ? 1 : -1)<0 || normal.y*(inner ? -1 : 1)<0) continue;
            const auto direction = add({sign,-1,0},scale(normal,inner ? -1 : 1));
            const auto target = inner ? subtract(delta,scale(normal,gauge)) : delta;
            const double radius = dot(target,direction)/dot(direction,direction);
            if (std::abs(radius-proposal)>3*tolerance ||
                length(subtract(target,scale(direction,radius)))>tolerance) continue;
            votes.push_back(radius);
            ++support[(left ? 0 : 2)+(inner ? 1 : 0)];
        }
    }
    if (*std::min_element(support.begin(),support.end())<80) return std::nullopt;
    const auto middle = votes.begin()+votes.size()/2;
    std::nth_element(votes.begin(),middle,votes.end());
    return *middle;
}

void fitChannelFeatures(modeling::ProfileBodyParams& body, const std::vector<LocalPoint>& points,
                    double width, double height, double gauge, double tolerance) {
    TopoDS_Shape host;
    std::string error;
    if (!modeling::buildProfileBody(body,host,error)) return;
    Bnd_Box bounds;
    BRepBndLib::AddOptimal(host,bounds,false,false);
    bounds.Enlarge(tolerance);
    std::vector<RimFaceEvidence> faces;
    for (TopExp_Explorer it(host,TopAbs_FACE); it.More(); it.Next())
        faces.emplace_back(TopoDS::Face(it.Current()),tolerance);
    std::vector<PointSample> residual;
    for (const auto& p : points) {
        const auto q = p.position;
        const gp_Pnt point(q.x,q.y,q.z);
        if (bounds.IsOut(point) || std::any_of(faces.begin(),faces.end(),[&](auto& f) {
            return f.match(point,p.normal,tolerance);
        })) continue;
        residual.push_back({p.id,p.position,p.normal,1,std::nullopt,true});
    }
    if (residual.size()<32) return;
    for (const auto axis : {Vec3{1,0,0},Vec3{0,1,0}}) {
        const bool across = axis.x==1;
        const std::array<std::pair<double,double>,2> walls = across
            ? std::array{std::pair{0.,gauge},std::pair{width-gauge,width}}
            : std::array{std::pair{height-gauge,height},std::pair{0.,0.}};
        for (const auto& [low,high] : walls) {
            if (low>=high) continue;
            struct Hit { Vec3 position, normal; double coordinate; };
            std::vector<Hit> hits;
            for (const auto& p : residual) {
                const auto coordinate = dot(p.position,axis);
                if (coordinate<low-tolerance || coordinate>high+tolerance || std::abs(dot(*p.normal,axis))>.005) continue;
                hits.push_back({subtract(p.position,scale(axis,coordinate)),
                    normalized(subtract(*p.normal,scale(axis,dot(*p.normal,axis)))),coordinate});
            }
            if (hits.size()<32) continue;
            struct Candidate { Vec3 center; double radius=0; std::size_t votes=0; };
            std::map<std::array<long long,3>,Candidate> candidates;
            const auto step = 1+(hits.size()-1)/256;
            const auto bin = [&](double x) { return std::llround(x/(4*tolerance)); };
            for (std::size_t i=0; i<hits.size(); i+=step) for (std::size_t j=i+step; j<hits.size(); j+=step) {
                const auto dn = subtract(hits[j].normal,hits[i].normal);
                const auto dp = subtract(hits[j].position,hits[i].position);
                const auto variance = dot(dn,dn);
                if (variance<.15) continue;
                const auto radius = dot(dp,dn)/variance;
                if (std::abs(radius)<4*tolerance || length(subtract(dp,scale(dn,radius)))>2*tolerance || (!across && radius>0)) continue;
                const auto center = scale(add(add(hits[i].position,hits[j].position),
                                              scale(add(hits[i].normal,hits[j].normal),-radius)),.5);
                auto& vote = candidates[{bin(across ? center.y : center.x),bin(center.z),bin(radius)}];
                vote.center = add(vote.center,center);
                vote.radius += radius;
                ++vote.votes;
            }
            std::vector<Candidate> ranked;
            for (const auto& entry : candidates) if (entry.second.votes>=4) ranked.push_back(entry.second);
            std::stable_sort(ranked.begin(),ranked.end(),[](const auto& a, const auto& b) { return a.votes>b.votes; });
            if (ranked.size()>12) ranked.resize(12);
            for (const auto& vote : ranked) {
                const auto estimate = scale(vote.center,1./vote.votes);
                const auto radius = vote.radius/vote.votes;
                const auto supported = [&](const Hit& hit) {
                    const auto radial = subtract(hit.position,estimate);
                    return std::abs(length(radial)-std::abs(radius))<=tolerance &&
                        dot(radial,hit.normal)*std::copysign(1.,radius)>.995*std::abs(radius);
                };
                std::bitset<4> layers;
                Vec3 meanPosition{}, meanNormal{};
                std::size_t count = 0;
                for (const auto& hit : hits) if (supported(hit)) {
                    layers.set(std::clamp(int(4*(hit.coordinate-low)/(high-low)),0,3));
                    meanPosition = add(meanPosition,hit.position);
                    meanNormal = add(meanNormal,hit.normal);
                    ++count;
                }
                if (count<32 || !layers.all()) continue;
                meanPosition = scale(meanPosition,1./count);
                meanNormal = scale(meanNormal,1./count);
                double covariance=0, variance=0;
                for (const auto& hit : hits) if (supported(hit)) {
                    const auto delta = subtract(hit.normal,meanNormal);
                    covariance += dot(subtract(hit.position,meanPosition),delta);
                    variance += dot(delta,delta);
                }
                if (variance<.05*count) continue;
                const auto signedRadius = covariance/variance;
                const auto r = std::abs(signedRadius);
                const auto center = subtract(meanPosition,scale(meanNormal,signedRadius));
                if (r<=4*tolerance || std::abs(signedRadius-radius)>2*tolerance) continue;
                if (signedRadius>0) {
                    if (std::abs(center.y-r)>2*tolerance) continue;
                    const auto middle = (low+high)*.5;
                    const auto wall = std::find_if(body.outline.begin(),body.outline.end(),[&](const auto& curve) {
                        return !curve.circular && curve.y1==0 && curve.y2==0 &&
                            std::min(curve.x1,curve.x2)<middle && std::max(curve.x1,curve.x2)>middle;
                    });
                    if (wall==body.outline.end()) continue;
                    const auto index = std::size_t(wall-body.outline.begin());
                    for (std::size_t i=0; i<body.trimPlanes.size(); ++i) {
                        const auto& plane = body.trimPlanes[i];
                        if (std::abs(plane.outward.x)*gauge>tolerance ||
                            std::abs(dot(plane.outward,subtract(center,plane.point))+r)>2*tolerance) continue;
                        if (std::any_of(body.rimRounds.begin(),body.rimRounds.end(),[&](const auto& round) {
                            return round.trimPlane==i && round.curveIndex==index;
                        })) continue;
                        body.rimRounds.push_back({index,r,false,i});
                    }
                    continue;
                }
                if (std::any_of(body.cavities.begin(),body.cavities.end(),[&](const auto& cut) {
                    return dot(cut.axis,axis)>.99999 && std::abs(cut.start-low)<.1*gauge &&
                        std::abs(cut.startRadius-r)<tolerance &&
                        length(subtract(center,{cut.centerX,cut.centerY,cut.centerZ}))<tolerance;
                })) continue;
                const auto pad = std::max(4*tolerance,.05*gauge);
                body.cavities.push_back({center.x,center.y,low-pad,high+pad,r,r,0,0,center.z,axis});
            }
        }
    }
}

std::optional<Profile> fitBentChannel(const PointStore& source, const WorkingData& work,
    const std::vector<FiniteSurfacePatch>& patches, const std::function<bool()>& cancelled) {
    const double tolerance = std::max(6*work.tolerance,1e-6);
    const auto planes = channelPlanes(patches,tolerance);
    if (planes.size()<8 || planes.size()>28 || source.size()<3000 || source.size()>250000) return std::nullopt;
    const auto opposite = [](Vec3 a, Vec3 b) { return dot(a,b)<-.99999; };
    std::size_t proposals = 0;
    for (const auto& outer : planes) for (const auto& inner : planes) {
        const auto v = outer.normal;
        const auto gauge = dot(v,subtract(outer.point,inner.point));
        if (gauge<12*tolerance || !opposite(v,inner.normal)) continue;
        const auto base = std::min_element(planes.begin(),planes.end(),[&](const auto& a, const auto& b) {
            return (opposite(v,a.normal) ? dot(v,a.point) : INFINITY) <
                   (opposite(v,b.normal) ? dot(v,b.point) : INFINITY);
        });
        const auto height = dot(v,subtract(outer.point,base->point));
        if (height<5*gauge) continue;
        for (const auto& left : planes) {
            if (std::abs(dot(left.normal,v))>.00001) continue;
            const auto u = normalized(subtract(scale(left.normal,-1),scale(v,-dot(left.normal,v))));
            const auto w = cross(u,v);
            const auto near = [&](Vec3 normal, double level) {
                return std::any_of(planes.begin(),planes.end(),[&](const auto& p) {
                    return dot(normal,p.normal)>.99999 && std::abs(dot(u,p.point)-level)<tolerance;
                });
            };
            const auto x0 = dot(u,left.point);
            if (!near(u,x0+gauge)) continue;
            for (const auto& right : planes) {
                if (!opposite(left.normal,right.normal)) continue;
                const auto width = dot(u,right.point)-x0;
                if (width<5*gauge || !near(scale(u,-1),x0+width-gauge)) continue;
                const ChannelPlane* ends[2]{};
                for (const auto& p : planes) {
                    const auto component = dot(w,p.normal);
                    if (std::abs(component)<.7) continue;
                    const auto side = component>0 ? 1 : 0;
                    if (!ends[side] || ends[side]->support<p.support) ends[side]=&p;
                }
                if (!ends[0] || !ends[1]) continue;
                const auto sectionOrigin = add(scale(u,x0),scale(v,dot(v,base->point)));
                const auto range = channelEndRange(*ends[0],*ends[1],sectionOrigin,u,v,w,width,height);
                if (!range || range->second-range->first<3*std::max(width,height)) continue;
                const auto [start,end] = *range;
                if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
                if (++proposals>6) return std::nullopt;
                const auto origin = add(sectionOrigin,scale(w,start));
                const auto local = [&](Vec3 p) { return Vec3{dot(p,u),dot(p,v),dot(p,w)}; };
                const auto cylinders = fitCylinderWallsAlongAxis(work,patches,w,tolerance);
                const auto alignedBend = [&](double radius, double centerX, double centerY) {
                    return std::any_of(cylinders.begin(),cylinders.end(),[&](const auto& c) {
                        const auto center = local(subtract(c.axisPoint,origin));
                        return std::abs(c.radius-radius)<2*tolerance &&
                            std::hypot(center.x-centerX,center.y-centerY)<2*tolerance;
                    });
                };
                std::vector<double> tried;
                for (const auto& cylinder : cylinders) {
                    const auto bend = cylinder.radius;
                    if (bend<=gauge+8*tolerance || bend>=std::min(width*.5,height)-8*tolerance ||
                        (!alignedBend(bend,bend,height-bend) && !alignedBend(bend,width-bend,height-bend)) ||
                        (!alignedBend(bend-gauge,bend,height-bend) &&
                         !alignedBend(bend-gauge,width-bend,height-bend))) continue;
                    Profile fitted;
                    auto& body = fitted.body;
                    body.referenceSizeX = body.sizeX = width;
                    body.referenceSizeY = body.sizeY = height;
                    body.depth = body.sizeZ = end-start;
                    for (const auto* plane : ends)
                        body.trimPlanes.push_back({local(subtract(plane->point,origin)),local(plane->normal)});
                    std::vector<LocalPoint> points;
                    points.reserve(source.size());
                    for (const auto& p : source.points())
                        if (p.normal && p.normalOrientationKnown && length(*p.normal)>1e-12)
                            points.push_back({p.id,local(subtract(p.position,origin)),local(normalized(*p.normal))});
                    if (points.size()<.98*source.size()) return std::nullopt;
                    const auto refinedBend = refineChannelBend(points,width,height,gauge,bend,tolerance);
                    if (!refinedBend || std::any_of(tried.begin(),tried.end(),[&](double prior) {
                        return std::abs(prior-*refinedBend)<=tolerance;
                    })) continue;
                    tried.push_back(*refinedBend);
                    const ChannelInterior section{width,height,gauge,*refinedBend};
                    if (!hasChannelEnds(points,section,body.trimPlanes,tolerance)) continue;
                    body.outline = channelOutline(width,height,gauge,*refinedBend);
                    if (!hasChannelWalls(points,section,body.outline,body.depth,tolerance)) continue;
                    fitChannelFeatures(body,points,width,height,gauge,tolerance);
                    if (!verifyAnalyticProfile(fitted,points,patches,tolerance,section)) continue;
                    body.pose = {origin,u,w};
                    std::string error;
                    if (!modeling::refreshProfileBodyDimensions(body,error)) continue;
                    return fitted;
                }
            }
        }
    }
    return std::nullopt;
}

std::optional<Profile> fitRoundedEnd(const std::vector<LocalPoint>& points, const std::vector<Side>& sides,
    const std::vector<Circle>& cylinders, const std::vector<FiniteSurfacePatch>& patches,
    const FiniteSurfacePatch& bottom, const FiniteSurfacePatch& top, Vec3 x, Vec3 y, Vec3 z,
    double height, double tolerance) {
    if (sides.size()!=3 || cylinders.empty()) return std::nullopt;
    for (const auto& outer : cylinders) {
        const Vec3 center{outer.x,outer.y,0};
        const Side* tangents[2]{};
        const Side* back = nullptr;
        for (const auto& side : sides) {
            if (std::abs(boundaryOffset(side,center)+outer.radius)<=tolerance) {
                if (!tangents[0]) tangents[0] = &side;
                else tangents[1] = &side;
            } else back = &side;
        }
        if (!tangents[0] || !tangents[1] || !back ||
            dot(tangents[0]->normal,tangents[1]->normal)>-.99999 ||
            std::abs(dot(back->normal,tangents[0]->normal))*outer.radius>tolerance) continue;
        const double straightLength = -boundaryOffset(*back,center);
        if (straightLength<=8*tolerance) continue;
        const Vec3 forward = scale(back->normal,-1);
        const Vec3 across{-forward.y,forward.x,0};
        const auto radiusAt = [&](Vec3 q) {
            const auto u = dot(q,forward), v = dot(q,across);
            return std::hypot(std::max(0.0,u),v);
        };
        const Side dividingPlane{back->normal,dot(back->normal,center)};
        if (!cylindricalWall(outer,points,{dividingPlane},height*.18,height*.82,tolerance,false,true))
            continue;
        const auto cones = boreConeCandidates(points,height,tolerance,16);
        modeling::ProfileEndBevel bevels[2]{};
        const auto bevelSupported = [&](int cap, modeling::ProfileEndBevel bevel) {
            const double slope = bevel.inset/bevel.depth;
            const double norm = std::hypot(1.,slope);
            const double end = cap ? height : 0;
            const double sign = cap ? 1 : -1;
            std::bitset<24> bins;
            for (const auto& p : points) {
                const auto q = subtract(p.position,center);
                const double depth = sign*(end-q.z), u = dot(q,forward), v = dot(q,across);
                if (depth<0 || depth>bevel.depth || u < -straightLength) continue;
                const auto radial = radiusAt(q);
                if (radial<=tolerance || std::abs(radial-outer.radius+bevel.inset-depth*slope)>tolerance*norm) continue;
                const auto direction = u>0 ? scale(Vec3{q.x,q.y,0},1/radial) : scale(across,std::copysign(1.,v));
                if ((dot(p.normal,direction)+sign*slope*p.normal.z)/norm<.98) continue;
                const auto band = u<=0 ? (v<0 ? 0 : 7) : 1+std::clamp(int((std::atan2(v,u)+pi*.5)*6/pi),0,5);
                const auto layer = std::clamp(int(depth*3/bevel.depth),0,2);
                bins.set(band*3+layer);
            }
            return bins.all();
        };
        for (const auto& cone : cones) {
            if (!coaxialBoreCone(outer,cone,height,tolerance)) continue;
            const int cap = cone.axisDirection.z<0;
            const double slope = std::tan(cone.openingAngleRadians);
            const double distance = cap ? cone.apex.z-height : -cone.apex.z;
            const modeling::ProfileEndBevel bevel{outer.radius/slope-distance,outer.radius-distance*slope};
            if (distance<=0 || bevel.depth<=4*tolerance || bevel.depth>=height*.3 ||
                bevel.inset<=4*tolerance || bevel.inset>=outer.radius*.3 || !bevelSupported(cap,bevel)) continue;
            bevels[cap] = bevel;
        }

        std::vector<Bore> bores;
        for (const auto& hole : cylinders) {
            const auto offset = radiusAt({hole.x-outer.x,hole.y-outer.y,0});
            if (offset+hole.radius>=outer.radius-2*tolerance || !interiorCircle(hole,sides,tolerance)) continue;
            auto bore = countersunkBore(hole,cones,points,sides,height,tolerance,bottom,top,x,y);
            if (!bore && cylindricalWall(hole,points,sides,0,height,tolerance,false))
                bore = Bore{hole,{boreSection(hole,0,height,hole.radius,hole.radius)}};
            if (!bore || offset+std::max(bore->openingRadius(0),bore->openingRadius(height))>=
                outer.radius-std::max(bevels[0].inset,bevels[1].inset)-tolerance) continue;
            bores.push_back(std::move(*bore));
        }
        const auto width = straightLength+outer.radius;
        std::bitset<96> expectedCaps[2], observedCaps[2];
        const auto cutAt = [&](Vec3 p, int cap) {
            return std::any_of(bores.begin(),bores.end(),[&](const auto& bore) {
                const auto r = bore.openingRadius(cap ? height : 0);
                return std::hypot(p.x-bore.circle.x,p.y-bore.circle.y)<r-tolerance;
            });
        };
        for (int i=0; i<12; ++i) for (int j=0; j<8; ++j) {
            const auto p = add(center,add(scale(forward,-straightLength+(i+.5)*width/12),
                                          scale(across,outer.radius*((j+.5)/4-1))));
            for (int cap : {0,1}) if (radiusAt(subtract(p,center))<outer.radius-bevels[cap].inset && !cutAt(p,cap))
                expectedCaps[cap].set(i*8+j);
        }
        Profile fitted;
        std::unordered_set<PointId> outside;
        std::bitset<12> walls[3];
        std::size_t capSupport[2]{};
        for (const auto& p : points) {
            const auto q = subtract(p.position,center);
            const double u = dot(q,forward), v = dot(q,across);
            const auto radial = radiusAt(q);
            if (q.z < -tolerance || q.z > height+tolerance || u < -straightLength-tolerance ||
                radial>outer.radius+tolerance) {
                outside.insert(p.id);
                continue;
            }
            const int end = q.z>height*.5;
            const double depth = end ? height-q.z : q.z;
            const double sign = end ? 1 : -1;
            const auto bevel = bevels[end];
            const double slope = bevel.depth>0 && depth<bevel.depth ? bevel.inset/bevel.depth : 0;
            const double rim = slope>0 ? outer.radius-(bevel.depth-std::max(0.,depth))*slope : outer.radius;
            if (radial>rim+tolerance) continue;
            bool boreWall = false, inBore = false;
            for (const auto& bore : bores) {
                const auto r = std::hypot(p.position.x-bore.circle.x,p.position.y-bore.circle.y);
                for (const auto& section : bore.sections) {
                    if (q.z < section.start-tolerance || q.z > section.end+tolerance) continue;
                    boreWall |= boreSurface(section,p,r,tolerance);
                    inBore |= r<sectionRadius(section,q.z)-tolerance;
                }
            }
            if (inBore && !boreWall) continue;
            bool boundary = boreWall;
            if (depth<=tolerance && p.normal.z*sign>=.95 && !cutAt(p.position,end)) {
                const auto i = std::clamp(int((u+straightLength)*12/width),0,11);
                const auto j = std::clamp(int((v/outer.radius+1)*4),0,7);
                observedCaps[end].set(i*8+j);
                ++capSupport[end];
                boundary = true;
            }
            if (std::abs(u+straightLength)<=tolerance && dot(p.normal,forward)<=-.98) {
                const auto i = std::clamp(int((v/outer.radius+1)*2),0,3);
                const auto j = std::clamp(int(q.z*3/height),0,2);
                walls[0].set(i*3+j);
                boundary = true;
            }
            if (radial>tolerance && std::abs(radial-rim)<=tolerance*std::hypot(1.,slope)) {
                const auto normal = u>=0 ? scale(Vec3{q.x,q.y,0},1/radial) : scale(across,std::copysign(1.,v));
                if ((dot(normal,p.normal)+sign*slope*p.normal.z)/std::hypot(1.,slope)>=.98) {
                    if (u<0 && slope==0) {
                        const auto i = std::clamp(int((u/straightLength+1)*4),0,3);
                        const auto j = std::clamp(int((q.z-bevels[0].depth)*3/
                            (height-bevels[0].depth-bevels[1].depth)),0,2);
                        walls[v<0 ? 1 : 2].set(i*3+j);
                    }
                    boundary = true;
                }
            }
            if (boundary) fitted.support.insert(p.id);
        }
        const auto eligible = points.size()-outside.size();
        if (eligible<points.size()*.80 || fitted.support.size()<eligible*minimumCoverage ||
            capSupport[0]<.9*bottom.workingPointIds.size() || capSupport[1]<.9*top.workingPointIds.size() ||
            std::any_of(std::begin(walls),std::end(walls),[](const auto& w) { return w.count()<10; })) continue;
        if (std::any_of(patches.begin(),patches.end(),[&](const auto& patch) {
            std::size_t included=0, supported=0;
            for (const auto id : patch.pointIds) {
                included += outside.count(id)==0;
                supported += fitted.support.count(id);
            }
            return included>=24 && supported<.85*included;
        })) continue;
        if (expectedCaps[0].none() || expectedCaps[1].none() ||
            (observedCaps[0]&expectedCaps[0]).count()<.85*expectedCaps[0].count() ||
            (observedCaps[1]&expectedCaps[1]).count()<.85*expectedCaps[1].count()) continue;

        auto& body = fitted.body;
        body.pose = {add(bottom.origin,add(scale(x,center.x),scale(y,center.y))),
                     add(scale(x,forward.x),scale(y,forward.y)),z};
        body.roundedEnd = modeling::RoundedEndProfile{{},straightLength,outer.radius,bevels[0],bevels[1]};
        body.depth = body.sizeZ = height;
        body.referenceSizeX = body.sizeX = width;
        body.referenceSizeY = body.sizeY = 2*outer.radius;
        for (auto& bore : bores) for (auto section : bore.sections) {
            const Vec3 hole{section.centerX-center.x,section.centerY-center.y,0};
            section.centerX = dot(hole,forward);
            section.centerY = dot(hole,across);
            // Continue a through cutter just beyond the cap so its end face
            // cannot coincide with the host face during subtraction.
            const double slope = (section.endRadius-section.startRadius)/(section.end-section.start);
            const double extension = 4*tolerance;
            if (section.start==0) {
                section.start -= extension;
                section.startRadius -= slope*extension;
            }
            if (section.end==height) {
                section.end += extension;
                section.endRadius += slope*extension;
            }
            body.cavities.push_back(section);
        }
        TopoDS_Shape shape;
        std::string error;
        if (modeling::buildProfileBody(body,shape,error)) return fitted;
    }
    return std::nullopt;
}

std::optional<Profile> fitSlab(const PointStore& source, const WorkingData& work, const std::vector<FiniteSurfacePatch>& patches,
                               const FiniteSurfacePatch& bottom, const FiniteSurfacePatch& top,
                               double height, double tolerance) {
    const auto z = scale(bottom.normal, -1);
    const auto x = bottom.xDirection, y = cross(z, x);
    auto local = [&](Vec3 v) { return Vec3{dot(v,x), dot(v,y), dot(v,z)}; };
    std::vector<LocalPoint> points;
    points.reserve(source.size());
    double extent = 0;
    std::size_t outsideSlab = 0;
    for (const auto& p : source.points()) {
        if (!p.normal || !p.normalOrientationKnown || length(*p.normal) < 1e-12) continue;
        const auto q = local(subtract(p.position, bottom.origin));
        extent = std::max(extent, std::max(std::abs(q.x), std::abs(q.y)));
        outsideSlab += q.z < -tolerance || q.z > height + tolerance;
        points.push_back({p.id, q, local(normalized(*p.normal))});
    }
    const auto inSlab = points.size()-outsideSlab;
    // A separate surface beyond a verified end face can remain unassigned.
    // Inside the slab, retain the same strict coverage requirement.
    if (inSlab < source.size()*.80) return std::nullopt;

    std::vector<Side> sides;
    std::vector<FiniteWall> finiteWalls;
    auto addSide = [&](Vec3 n, Vec3 position) {
        if (std::abs(n.z) >= .1 || std::abs(n.z) * height > tolerance) return;
        const auto direction = normalized(Vec3{n.x,n.y,0});
        const Side side{direction, dot(direction, position)};
        if (std::any_of(finiteWalls.begin(), finiteWalls.end(), [&](const auto& wall) {
                return dot(wall.plane.normal, side.normal) >= .9999 && std::abs(wall.plane.offset - side.offset) <= tolerance;
            })) return;
        const Vec3 along{-direction.y,direction.x,0};
        double low = INFINITY, high = -INFINITY;
        std::size_t outside = 0;
        std::vector<double> positions;
        for (const auto& p : points) {
            if (p.position.z < -tolerance || p.position.z > height+tolerance) continue;
            const double distance = boundaryOffset(side,p.position);
            outside += distance > tolerance;
            if (std::abs(distance) > tolerance || dot(direction,p.normal) < .999) continue;
            const double s = dot(along,p.position);
            low = std::min(low,s); high = std::max(high,s);
            positions.push_back(s);
        }
        // A tangent strip on a curved corner can resemble a plane within
        // the distance tolerance. A straight wall needs transverse extent.
        const double minimumSpan = std::max(height * .25, tolerance * 8);
        if (positions.size() < 24 || high-low < minimumSpan) return;
        std::sort(positions.begin(),positions.end());
        std::vector<double> gaps;
        for (std::size_t i=1; i<positions.size(); ++i) {
            const auto gap = positions[i]-positions[i-1];
            if (gap>tolerance) gaps.push_back(gap);
        }
        if (gaps.empty()) return;
        const auto middle = gaps.begin()+gaps.size()/2;
        std::nth_element(gaps.begin(),middle,gaps.end());
        const double separation = std::max({8*tolerance,.015*(high-low),6*(*middle)});
        for (std::size_t first=0, last=1; last<=positions.size(); ++last) {
            if (last<positions.size() && positions[last]-positions[last-1]<=separation) continue;
            if (last-first>=24 && positions[last-1]-positions[first]>=minimumSpan)
                finiteWalls.push_back({side,positions[first],positions[last-1]});
            first = last;
        }
        if (outside <= inSlab * (1 - minimumCoverage)) sides.push_back(side);
    };
    for (const auto& patch : patches) {
        if (patch.planar && patch.orientationKnown && patch.workingPointIds.size() >= 24)
            addSide(local(patch.normal), local(subtract(patch.origin, bottom.origin)));
    }
    const double clipExtent = std::max(extent * 2, height * 2);
    if (sides.size() > 24) return std::nullopt;
    auto polygon = clippedOutline(sides, clipExtent);
    if (polygon.empty() || !observedSides(polygon, Corners(polygon.size()), points, height, tolerance)) {
        // Sparse longitudinal strips can split a genuine wall into columns
        // smaller than the connected-patch minimum. Recover its plane using
        // only observations tangent to the slab, then recheck finite support.
        std::vector<PointSample> walls;
        for (const auto& p : points) {
            if (std::abs(p.normal.z) < .1 && p.position.z >= -tolerance && p.position.z <= height + tolerance)
                walls.push_back({p.id, p.position, p.normal, 1.0, std::nullopt, true});
        }
        PrimitiveDetectionOptions options;
        options.detectCylinders = options.detectSpheres = options.detectCones = options.detectTori = false;
        options.plane.minimumSupportPoints = 24;
        options.plane.maximumPlanes = 24;
        options.plane.distanceThreshold = tolerance;
        options.plane.minimumNormalAlignment = .98;
        PrimitiveDetectionResult proposals;
        std::string error;
        const PointStore wallPoints(std::move(walls));
        detectPrimitiveEvidence(wallPoints, options, proposals, error);
        for (const auto& proposal : proposals.evidence) {
            const auto& plane = std::get<PlaneEvidence>(proposal).plane;
            std::size_t positive = 0, negative = 0;
            for (const auto& p : wallPoints.points()) {
                if (std::abs(dot(plane.normal, p.position) + plane.offset) > tolerance) continue;
                const double alignment = dot(plane.normal, *p.normal);
                positive += alignment >= .98;
                negative += alignment <= -.98;
            }
            const auto vote = std::max(positive, negative);
            if (vote < 24 || vote < (positive + negative) * .95) continue;
            addSide(scale(plane.normal, positive > negative ? 1 : -1), scale(plane.normal, -plane.offset));
        }
        if (sides.size() > 24) return std::nullopt;
        polygon = clippedOutline(sides, clipExtent);
    }
    bool finiteOutline = false;
    if (polygon.empty() || !observedSides(polygon,Corners(polygon.size()),points,height,tolerance)) {
        auto observed = finiteWallLoops(finiteWalls,tolerance,false);
        if (observed.size()==1 && observedSides(observed.front(),Corners(observed.front().size()),points,height,tolerance)) {
            polygon = std::move(observed.front());
            finiteOutline = true;
            sides.clear();
            for (std::size_t i=0; i<polygon.size(); ++i) {
                const auto edge = subtract(polygon[(i+1)%polygon.size()],polygon[i]);
                const auto normal = normalized(Vec3{edge.y,-edge.x,0});
                sides.push_back({normal,dot(normal,polygon[i])});
            }
        }
    }
    // Round flanges often carry a full circle of individual bores in
    // addition to their coaxial surfaces.
    constexpr std::size_t profileLimit = 16, flangeLimit = 40;
    auto cylinders = cylinderCandidates(points, sides, height, tolerance, (polygon.empty() || finiteOutline) ? flangeLimit : profileLimit);
    if (cylinders.empty() || finiteOutline) {
        for (const auto& fitted : fitCylinderWallsAlongAxis(work,patches,z,tolerance)) {
            const auto center = local(subtract(fitted.axisPoint,bottom.origin));
            if (std::any_of(cylinders.begin(),cylinders.end(),[&](const auto& c) {
                return std::hypot(c.x-center.x,c.y-center.y) <= tolerance && std::abs(c.radius-fitted.radius) <= tolerance;
            })) continue;
            cylinders.push_back({center.x,center.y,fitted.radius});
        }
    }
    const auto corners = finiteOutline ? Corners(polygon.size()) : roundedCorners(polygon, cylinders, points, height, tolerance);
    if (auto rim = fitRoundedRim(polygon,corners,cylinders,sides,points,patches,height,tolerance)) {
        rim->body.pose = {bottom.origin,x,z};
        return rim;
    }
    std::optional<Circle> exterior;
    if (polygon.empty() || !observedSides(polygon, corners, points, height, tolerance)) {
        if (auto rounded = fitRoundedEnd(points,sides,cylinders,patches,bottom,top,x,y,z,height,tolerance))
            return rounded;
        for (const auto& circle : cylinders) {
            if (!cylindricalWall(circle,points,{},0,height,tolerance,false,true)) continue;
            const auto outside = std::count_if(points.begin(),points.end(),[&](const auto& p) {
                return p.position.z >= -tolerance && p.position.z <= height+tolerance &&
                       std::hypot(p.position.x-circle.x,p.position.y-circle.y) > circle.radius+tolerance;
            });
            if (outside <= inSlab * (1-minimumCoverage)) {
                exterior = circle;
                break;
            }
        }
        if (!exterior) return std::nullopt;
        sides.clear();
    }
    const auto inside = [&](Vec3 p, double margin) {
        if (exterior) return std::hypot(p.x-exterior->x,p.y-exterior->y) <= exterior->radius+margin;
        if (finiteOutline) return insideFiniteOutline(polygon,p,margin);
        return insideRoundedOutline(sides,corners,p,margin);
    };
    const std::vector<Side> unconstrained;
    const auto& boreSides = finiteOutline ? unconstrained : sides;
    const auto grooves = exterior ? annularGrooves(*exterior,cylinders,points,height,tolerance)
                                  : std::vector<modeling::AxialSection>{};
    std::vector<Bore> holes;
    std::vector<ProfileVoid> voids;
    if (!exterior) for (auto& loop : finiteWallLoops(finiteWalls,tolerance,true)) {
        if (std::any_of(loop.begin(),loop.end(),[&](auto p) { return !inside(p,-2*tolerance); })) continue;
        std::reverse(loop.begin(),loop.end());
        ProfileVoid hole;
        hole.polygon = std::move(loop);
        for (std::size_t i=0; i<hole.polygon.size(); ++i) {
            const auto a = hole.polygon[i], b = hole.polygon[(i+1)%hole.polygon.size()];
            hole.curves.push_back({false,a.x,a.y,b.x,b.y});
        }
        if (observedVoidWalls(hole,points,height,tolerance)) voids.push_back(std::move(hole));
    }
    std::optional<std::vector<ConeEquation>> cones;
    for (const auto& hole : cylinders) {
        if ((exterior || finiteOutline) && !inside({hole.x,hole.y,0}, -hole.radius-tolerance)) continue;
        if (std::any_of(grooves.begin(),grooves.end(),[&](const auto& g) {
            return std::hypot(hole.x-g.centerX,hole.y-g.centerY) <= tolerance &&
                   (std::abs(hole.radius-g.innerStartRadius) <= tolerance || std::abs(hole.radius-g.startRadius) <= tolerance);
        })) continue;
        if (!exterior && inside({hole.x,hole.y,0},-hole.radius-tolerance)) {
            if (auto clipped = clippedCircularVoid(hole,finiteWalls,points,height,tolerance)) {
                voids.push_back(std::move(*clipped));
                continue;
            }
        }
        const bool openings[] = {hasOpening(hole,bottom,bottom.origin,x,y,tolerance),
                                 hasOpening(hole,top,bottom.origin,x,y,tolerance)};
        std::optional<Bore> bore;
        if (cylindricalWall(hole,points,boreSides,0,height,tolerance,openings[0] && openings[1])) {
            bore = Bore{hole,{boreSection(hole,0,height,hole.radius,hole.radius)}};
        } else {
            if (!cones) cones = boreConeCandidates(points,height,tolerance,exterior ? flangeLimit : profileLimit);
            bore = steppedBore(hole,cylinders,*cones,points,boreSides,height,tolerance);
            if (!bore) bore = countersunkBore(hole,*cones,points,boreSides,height,tolerance,bottom,top,x,y);
            if (!bore) bore = blindBore(hole,*cones,points,boreSides,height,tolerance,openings);
        }
        if (!bore) continue;
        std::size_t contradictoryCapPoints = 0;
        for (const auto& p : points) {
            const auto q = p.position;
            const bool onBottom = std::abs(q.z)<=tolerance && p.normal.z<=-.95;
            const bool onTop = std::abs(q.z-height)<=tolerance && p.normal.z>=.95;
            if (!onBottom && !onTop) continue;
            const auto radius = bore->openingRadius(onTop ? height : 0);
            if (std::hypot(q.x-hole.x,q.y-hole.y)<radius-2*tolerance && inside(q,tolerance) &&
                ++contradictoryCapPoints>4) break;
        }
        if (contradictoryCapPoints>4) continue;
        const auto maxRadius = [](const Bore& b) {
            return std::max({b.circle.radius,b.sections.front().startRadius,b.sections.back().endRadius});
        };
        if (std::none_of(holes.begin(), holes.end(), [&](const Bore& other) {
                return std::hypot(hole.x-other.circle.x,hole.y-other.circle.y) < maxRadius(*bore)+maxRadius(other)+tolerance;
            })) holes.push_back(std::move(*bore));
    }
    if (exterior && holes.empty() && grooves.empty()) return std::nullopt;

    Vec3 low, high;
    if (exterior) {
        low = {exterior->x-exterior->radius,exterior->y-exterior->radius,0};
        high = {exterior->x+exterior->radius,exterior->y+exterior->radius,0};
    } else {
        low = high = polygon.front();
        for (const auto& point : polygon) {
            low.x = std::min(low.x,point.x); low.y = std::min(low.y,point.y);
            high.x = std::max(high.x,point.x); high.y = std::max(high.y,point.y);
        }
    }
    const double width = high.x - low.x, length = high.y - low.y;
    if (std::min(width, length) + tolerance < 4 * height) return std::nullopt;
    constexpr int capColumns = 12, capRows = 8;
    std::bitset<capColumns * capRows> expectedCaps[2], observedCaps[2];
    for (int i = 0; i < capColumns; ++i) for (int j = 0; j < capRows; ++j) {
        const Vec3 p{low.x + (i+.5)*width/capColumns, low.y + (j+.5)*length/capRows, 0};
        if (!inside(p,0)) continue;
        if (std::any_of(voids.begin(),voids.end(),[&](const auto& h) { return h.contains(p,0); })) continue;
        for (int cap : {0,1}) {
            const auto inGroove = std::any_of(grooves.begin(),grooves.end(),[&](const auto& g) {
                const auto radius = std::hypot(p.x-g.centerX,p.y-g.centerY);
                return (cap ? g.end == height : g.start == 0) &&
                       radius > g.innerStartRadius && radius < g.startRadius;
            });
            if (inGroove) continue;
            if (std::none_of(holes.begin(), holes.end(), [&](const Bore& h) {
                    return std::hypot(p.x-h.circle.x,p.y-h.circle.y) < h.openingRadius(cap*height);
                })) expectedCaps[cap].set(i * capRows + j);
        }
    }

    Profile profile;
    std::unordered_set<PointId> outsideEnvelope;
    // Verify the reconstructed trimmed faces against original, signed observations.
    std::size_t caps[2]{};
    for (const auto& p : points) {
        const auto q = p.position;
        if (q.z < -tolerance || q.z > height + tolerance || !inside(q,tolerance)) {
            outsideEnvelope.insert(p.id);
            continue;
        }
        bool boundary = false;
        bool insideVoid = false;
        for (const auto& hole : voids) {
            if (!hole.contains(q,tolerance)) continue;
            insideVoid |= hole.contains(q,-tolerance);
            boundary |= std::any_of(hole.curves.begin(),hole.curves.end(),[&](const auto& c) {
                const double u = profileWallCoordinate(c,q,scale(p.normal,-1),tolerance);
                return u>=0 && u<=1;
            });
        }
        for (const auto& g : grooves) {
            const auto dx = q.x-g.centerX, dy = q.y-g.centerY, radius = std::hypot(dx,dy);
            if (q.z < g.start-tolerance || q.z > g.end+tolerance ||
                radius < g.innerStartRadius-tolerance || radius > g.startRadius+tolerance) continue;
            const double radialNormal = dx*p.normal.x+dy*p.normal.y;
            boundary |= (std::abs(radius-g.innerStartRadius) <= tolerance && radialNormal >= .95*radius) ||
                        (std::abs(radius-g.startRadius) <= tolerance && radialNormal <= -.95*radius);
            const bool opensTop = g.end == height;
            const double floor = opensTop ? g.start : g.end;
            boundary |= std::abs(q.z-floor) <= tolerance && p.normal.z*(opensTop ? 1 : -1) >= .95;
            insideVoid |= (q.z-floor)*(opensTop ? 1 : -1) > tolerance &&
                          radius > g.innerStartRadius+tolerance && radius < g.startRadius-tolerance;
        }
        for (const auto& bore : holes) {
            const auto& h = bore.circle;
            const double radius = std::hypot(q.x-h.x,q.y-h.y);
            bool shoulder = false;
            for (std::size_t i=1; i<bore.sections.size(); ++i) {
                const auto& before = bore.sections[i-1];
                const auto& after = bore.sections[i];
                const double change = after.startRadius-before.endRadius;
                if (std::abs(change)<=tolerance || std::abs(q.z-before.end)>tolerance) continue;
                const auto radii = std::minmax(before.endRadius,after.startRadius);
                shoulder |= radius>=radii.first-tolerance && radius<=radii.second+tolerance &&
                            p.normal.z*std::copysign(1.0,change)>=.95;
            }
            if (shoulder) { boundary = true; continue; }
            for (const auto& section : bore.sections) {
                if (q.z < section.start-tolerance || q.z > section.end+tolerance) continue;
                const double slope = (section.endRadius-section.startRadius)/(section.end-section.start);
                const double clearance = tolerance*std::hypot(1.0,slope);
                boundary |= boreSurface(section,p,radius,tolerance);
                if (q.z >= section.start && q.z <= section.end)
                    insideVoid |= radius < sectionRadius(section,q.z)-clearance;
            }
        }
        if (insideVoid) continue;
        if (exterior) {
            const auto dx = q.x-exterior->x, dy = q.y-exterior->y;
            boundary |= std::abs(std::hypot(dx,dy)-exterior->radius) <= tolerance &&
                        dx*p.normal.x+dy*p.normal.y >= .95*exterior->radius;
        } else {
            boundary |= std::any_of(corners.begin(),corners.end(), [&](const auto& arc) { return arc && arcSupportsPoint(*arc,p,tolerance); });
        }
        const bool bottomMatch = std::abs(q.z) <= tolerance && p.normal.z <= -.95;
        const bool topMatch = std::abs(q.z - height) <= tolerance && p.normal.z >= .95;
        caps[0] += bottomMatch;
        caps[1] += topMatch;
        if (bottomMatch || topMatch) {
            const auto i = std::clamp(int((q.x-low.x)*capColumns/width), 0, capColumns - 1);
            const auto j = std::clamp(int((q.y-low.y)*capRows/length), 0, capRows - 1);
            observedCaps[topMatch ? 1 : 0].set(i * capRows + j);
        }
        boundary |= bottomMatch || topMatch || std::any_of(sides.begin(), sides.end(), [&](const Side& s) {
            return std::abs(boundaryOffset(s,q)) <= tolerance && dot(s.normal,p.normal) >= .95;
        });
        if (boundary) profile.support.insert(p.id);
    }
    if (profile.support.size() < inSlab * minimumCoverage ||
        caps[0] < bottom.workingPointIds.size() * .9 || caps[1] < top.workingPointIds.size() * .9) return std::nullopt;
    for (int k : {0,1}) {
        const auto& cap = observedCaps[k];
        const auto& expectedCap = expectedCaps[k];
        if (expectedCap.none()) return std::nullopt;
        if ((cap & expectedCap).count() < expectedCap.count() * .85) return std::nullopt;
        for (int i = 0; i < capColumns; ++i) {
            bool expected = false, observed = false;
            for (int j = 0; j < capRows; ++j) {
                expected |= expectedCap.test(i*capRows+j);
                observed |= cap.test(i*capRows+j);
            }
            if (expected && !observed) return std::nullopt;
        }
        for (int j = 0; j < capRows; ++j) {
            bool expected = false, observed = false;
            for (int i = 0; i < capColumns; ++i) {
                expected |= expectedCap.test(i*capRows+j);
                observed |= cap.test(i*capRows+j);
            }
            if (expected && !observed) return std::nullopt;
        }
    }
    // A small second feature can be less than the global outlier budget.
    // Keep it unresolved when it forms a coherent patch of its own.
    for (const auto& patch : patches) {
        if (patch.pointIds.size() < 24) continue;
        const auto eligible = std::count_if(patch.pointIds.begin(),patch.pointIds.end(),
            [&](auto id) { return outsideEnvelope.count(id) == 0; });
        if (eligible < 24) continue;
        const auto supported = std::count_if(patch.pointIds.begin(), patch.pointIds.end(),
            [&](auto id) { return profile.support.count(id) != 0; });
        if (supported < eligible * .85) return std::nullopt;
    }

    auto& body = profile.body;
    body.pose = {bottom.origin, x, z};
    body.depth = body.sizeZ = height;
    if (exterior) for (int half : {0,1}) {
        body.outline.push_back({true,0,0,0,0,exterior->x,exterior->y,exterior->radius,half*pi,pi});
    }
    for (std::size_t i = 0; !exterior && i < polygon.size(); ++i) {
        const auto next = (i+1) % polygon.size();
        if (corners[i]) {
            const auto& arc = *corners[i];
            body.outline.push_back({true,0,0,0,0,arc.center.x,arc.center.y,arc.radius,arc.angle,arc.sweep});
        }
        const auto a = corners[i] ? corners[i]->end : polygon[i];
        const auto b = corners[next] ? corners[next]->start : polygon[next];
        body.outline.push_back({false,a.x,a.y,b.x,b.y});
    }
    body.referenceSizeX = body.sizeX = width;
    body.referenceSizeY = body.sizeY = length;
    body.cavities = grooves;
    for (auto& hole : voids) body.profileVoids.push_back(std::move(hole.curves));
    for (const auto& bore : holes)
        body.cavities.insert(body.cavities.end(),bore.sections.begin(),bore.sections.end());
    TopoDS_Shape built;
    std::string failure;
    if (!modeling::buildProfileBody(body, built, failure)) return std::nullopt;
    return profile;
}
}

void closeAnalyticProfile(const PointStore& source, const WorkingData& work, AssemblyReconstructionResult& result,
                         const std::function<bool()>& cancelled) {
    std::vector<const FiniteSurfacePatch*> caps;
    for (const auto& p : result.patches)
        if (p.planar && p.orientationKnown && p.workingPointIds.size() >= std::max(80.0, result.workingPoints * .02))
            caps.push_back(&p);
    if (caps.size() < 2 || caps.size() > 32) return;
    std::stable_sort(caps.begin(), caps.end(), [](auto a, auto b) {
        return a->workingPointIds.size() > b->workingPointIds.size();
    });
    for (std::size_t i = 0; i < caps.size(); ++i) for (std::size_t j = i+1; j < caps.size(); ++j) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const auto& bottom = *caps[i];
        const auto& top = *caps[j];
        const double height = dot(bottom.normal, subtract(bottom.origin, top.origin));
        const double tolerance = std::max(result.distanceTolerance * 6, height * .001);
        if (height <= tolerance * 12 || dot(bottom.normal, top.normal) > -.9999 ||
            std::min(length(subtract(bottom.upper,bottom.lower)), length(subtract(top.upper,top.lower))) < 4 * height) continue;
        auto fitted = fitSlab(source, work, result.patches, bottom, top, height, tolerance);
        if (!fitted) continue;
        publishAnalyticProfile(source, work, result, std::move(fitted->body), fitted->support,
            "closed_analytic_profile", "Observed caps and walls agree with the analytic profile");
        return;
    }
    if (auto channel = fitBentChannel(source,work,result.patches,cancelled))
        publishAnalyticProfile(source,work,result,std::move(channel->body),channel->support,
            "closed_analytic_profile", "Paired bend walls and measured end cuts close the channel");
}
}
