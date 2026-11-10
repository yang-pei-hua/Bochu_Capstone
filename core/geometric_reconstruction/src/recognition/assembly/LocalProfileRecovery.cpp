#include "AssemblyInternal.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <modeling/ProfileBody.h>
#include <reconstruction/modeling/ModelingAdapter.h>

#include <algorithm>
#include <iterator>
#include <map>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace reconstruction::detail::assembly {
namespace {
struct BoundedSolid {
    TopoDS_Shape shape;
    Bnd_Box bounds;
};

bool contains(Vec3 lower, Vec3 upper, Vec3 point, double tolerance = 0) {
    return point.x >= lower.x-tolerance && point.x <= upper.x+tolerance &&
           point.y >= lower.y-tolerance && point.y <= upper.y+tolerance &&
           point.z >= lower.z-tolerance && point.z <= upper.z+tolerance;
}

struct CapPlane {
    Vec3 normal, origin, lower, upper;
    std::size_t observations;
};

struct CapPair {
    CapPlane first, second;
    bool admits(const PointSample& sample, double margin) const {
        const auto behind = [&](const CapPlane& cap) {
            const auto offset = dot(cap.normal,subtract(sample.position,cap.origin));
            if (offset > margin) return false;
            if (offset < -margin || !sample.normal || !sample.normalOrientationKnown) return true;
            return dot(cap.normal,*sample.normal) >= -.98 * length(*sample.normal);
        };
        return behind(first) && behind(second);
    }
};

std::optional<CapPair> strongestCapPair(const std::vector<CapPlane>& planes, double tolerance) {
    std::optional<CapPair> best;
    std::pair<std::size_t,std::size_t> bestSupport{79,0};
    for (auto first = planes.begin(); first != planes.end(); ++first) {
        const auto& a = *first;
        for (auto second = std::next(first); second != planes.end(); ++second) {
            const auto& b = *second;
            const auto support = std::pair{std::min(a.observations,b.observations),
                                           std::max(a.observations,b.observations)};
            if (support.first < 80 || support <= bestSupport || dot(a.normal,b.normal)>-.9999) continue;
            const auto depth = dot(a.normal,subtract(a.origin,b.origin));
            if (depth>12*tolerance && std::min(length(subtract(a.upper,a.lower)),length(subtract(b.upper,b.lower)))>4*depth) {
                best = CapPair{a,b};
                bestSupport = support;
            }
        }
    }
    return best;
}

std::vector<CapPlane> partPlanes(const AssemblyPart& part, const std::vector<FiniteSurfacePatch>& patches,
                                 double tolerance) {
    std::vector<CapPlane> planes;
    for (const auto id : part.patchIds) {
        const auto& patch = patches.at(id-1);
        if (!patch.planar || !patch.orientationKnown || patch.pointIds.empty()) continue;
        const auto found = std::find_if(planes.begin(),planes.end(),[&](const auto& p) {
            return dot(p.normal,patch.normal) >= .9999 &&
                   std::abs(dot(p.normal,subtract(patch.origin,p.origin))) <= tolerance;
        });
        if (found == planes.end()) {
            planes.push_back({patch.normal,patch.origin,patch.lower,patch.upper,patch.workingPointIds.size()});
        } else {
            found->lower = {std::min(found->lower.x,patch.lower.x),std::min(found->lower.y,patch.lower.y),
                            std::min(found->lower.z,patch.lower.z)};
            found->upper = {std::max(found->upper.x,patch.upper.x),std::max(found->upper.y,patch.upper.y),
                            std::max(found->upper.z,patch.upper.z)};
            found->observations += patch.workingPointIds.size();
        }
    }
    return planes;
}

std::optional<CapPair> opposingCaps(const AssemblyPart& part, const std::vector<FiniteSurfacePatch>& patches,
                                   double tolerance) {
    return strongestCapPair(partPlanes(part,patches,tolerance),tolerance);
}

bool overlappingCaps(const CapPlane& a, const CapPlane& b, double depth, double tolerance) {
    const auto shift = scale(a.normal,depth);
    const auto bLow = add(b.lower,shift), bHigh = add(b.upper,shift);
    const auto overlap = [tolerance](double a0, double a1, double b0, double b1) {
        const auto extent = std::max(a1-a0,b1-b0);
        return std::min(a1,b1)-std::max(a0,b0)+tolerance >= .55*extent;
    };
    return overlap(a.lower.x,a.upper.x,bLow.x,bHigh.x) &&
           overlap(a.lower.y,a.upper.y,bLow.y,bHigh.y) &&
           overlap(a.lower.z,a.upper.z,bLow.z,bHigh.z);
}

std::pair<double,double> extentAlong(const CapPlane& plane, Vec3 axis) {
    const Vec3 low{axis.x<0 ? plane.upper.x : plane.lower.x,axis.y<0 ? plane.upper.y : plane.lower.y,
                   axis.z<0 ? plane.upper.z : plane.lower.z};
    const auto high = subtract(add(plane.upper,plane.lower),low);
    return {dot(axis,low),dot(axis,high)};
}

struct ChannelWindow {
    CapPair flanks, vertical;
    Vec3 along;
    double start, end, gauge;
};

std::vector<ChannelWindow> channelWindows(const std::vector<CapPlane>& planes, double tolerance) {
    std::vector<ChannelWindow> windows;
    const auto opposing = [](Vec3 a, Vec3 b) { return dot(a,b)<-.99999; };
    for (const auto& left : planes) {
        if (left.observations<100) continue;
        // Choose a single orientation for each pair of outside flanks.
        const auto u = scale(left.normal,-1);
        const double dominant = std::abs(u.x)>=std::max(std::abs(u.y),std::abs(u.z)) ? u.x :
                                (std::abs(u.y)>=std::abs(u.z) ? u.y : u.z);
        if (dominant<0) continue;
        for (const auto& leftInner : planes) {
            const double gauge = dot(u,subtract(leftInner.origin,left.origin));
            if (gauge<=12*tolerance || leftInner.observations<100 || !opposing(left.normal,leftInner.normal) ||
                !overlappingCaps(leftInner,left,gauge,4*tolerance)) continue;
            const CapPlane* right = nullptr;
            double width = INFINITY;
            for (const auto& outside : planes) {
                const auto distance = dot(u,subtract(outside.origin,left.origin));
                if (!opposing(left.normal,outside.normal) || outside.observations<100 ||
                    distance<=5*gauge || distance>=width || !overlappingCaps(outside,left,distance,4*tolerance)) continue;
                const bool hasInner = std::any_of(planes.begin(),planes.end(),[&](const auto& inside) {
                    return inside.observations>=100 && opposing(inside.normal,outside.normal) &&
                           std::abs(dot(u,subtract(outside.origin,inside.origin))-gauge)<=4*tolerance &&
                           overlappingCaps(outside,inside,gauge,4*tolerance);
                });
                if (hasInner) { right=&outside; width=distance; }
            }
            if (!right) continue;
            for (const auto& roof : planes) {
                const auto v = roof.normal;
                if (roof.observations<80 || std::abs(dot(u,v))>.00001) continue;
                const auto [roofLow,roofHigh] = extentAlong(roof,u);
                const auto leftLevel = dot(u,left.origin), rightLevel = dot(u,right->origin);
                if (std::min(roofHigh,rightLevel)-std::max(roofLow,leftLevel)<.4*width) continue;
                const auto leftV = extentAlong(left,v);
                const auto rightV = extentAlong(*right,v);
                const double lowerWall = std::min(leftV.first,rightV.first);
                const double upperWall = std::max(leftV.second,rightV.second);
                const auto roofLevel = dot(v,roof.origin);
                if (roofLevel<=upperWall+2*gauge || roofLevel-lowerWall<5*gauge) continue;
                const CapPlane* foot = nullptr;
                bool hasInner = false;
                for (const auto& plane : planes) {
                    if (!opposing(v,plane.normal) || plane.observations<80) continue;
                    const auto level = dot(v,plane.origin);
                    const auto [low,high] = extentAlong(plane,u);
                    if (high<leftLevel-tolerance || low>rightLevel+tolerance) continue;
                    if (std::abs(roofLevel-level-gauge)<=4*tolerance &&
                        std::min(high,rightLevel)-std::max(low,leftLevel)>=.4*width) hasInner=true;
                    if (std::abs(level-lowerWall)<=2*gauge &&
                        (!foot || std::abs(level-lowerWall)<std::abs(dot(v,foot->origin)-lowerWall))) foot=&plane;
                }
                if (!foot || !hasInner) continue;
                const auto along = normalized(cross(u,v));
                const auto leftSpan = extentAlong(left,along);
                const auto rightSpan = extentAlong(*right,along);
                const auto start = std::min(leftSpan.first,rightSpan.first);
                const auto end = std::max(leftSpan.second,rightSpan.second);
                if (end-start<3*std::max(width,roofLevel-lowerWall)) continue;
                const auto extension = .06*(end-start);
                windows.push_back({{left,*right},{roof,*foot},along,start-extension,end+extension,gauge});
                if (windows.size()==8) return windows;
            }
        }
    }
    return windows;
}

enum class CapRanking { Span, Concentration };

std::vector<CapPair> rankedCapPairs(const std::vector<CapPlane>& planes, double tolerance, std::size_t limit,
                                  double minimumSpanRatio = 4, CapRanking ranking = CapRanking::Span) {
    std::vector<std::pair<double,CapPair>> ranked;
    ranked.reserve(limit);
    for (auto first=planes.begin(); first!=planes.end(); ++first) {
        if (first->observations<48) continue;
        for (auto second=std::next(first); second!=planes.end(); ++second) {
            if (second->observations<48 || dot(first->normal,second->normal)>-.9999) continue;
            const auto depth = dot(first->normal,subtract(first->origin,second->origin));
            const auto span = std::min(length(subtract(first->upper,first->lower)),length(subtract(second->upper,second->lower)));
            if (depth<=12*tolerance || span<=minimumSpanRatio*depth ||
                !overlappingCaps(*first,*second,depth,tolerance*4)) continue;
            const double score = ranking==CapRanking::Concentration
                ? std::min(first->observations,second->observations)/span
                : span/std::sqrt(depth);
            const auto position = std::upper_bound(ranked.begin(),ranked.end(),score,
                [](double value, const auto& candidate) { return value>candidate.first; });
            if (position==ranked.end() && ranked.size()==limit) continue;
            ranked.emplace(position,score,CapPair{*first,*second});
            if (ranked.size()>limit) ranked.pop_back();
        }
    }
    std::vector<CapPair> pairs;
    pairs.reserve(ranked.size());
    for (const auto& entry : ranked) pairs.push_back(entry.second);
    return pairs;
}

std::vector<CapPair> contactCapPairs(const std::vector<CapPlane>& planes, double tolerance) {
    const auto thinPairs = rankedCapPairs(planes,tolerance,6);
    std::vector<CapPair> contacts;
    for (const auto& compact : rankedCapPairs(planes,tolerance,8,1)) {
        const auto depth = dot(compact.first.normal,subtract(compact.first.origin,compact.second.origin));
        const auto span = std::min(length(subtract(compact.first.upper,compact.first.lower)),
                                   length(subtract(compact.second.upper,compact.second.lower)));
        if (span>4*depth) continue;
        const auto sharesEnd = [&](const CapPlane& end, const CapPlane& other) {
            return dot(end.normal,other.normal)<-.9999 &&
                   std::abs(dot(end.normal,subtract(end.origin,other.origin)))<=4*tolerance &&
                   overlappingCaps(end,other,0,4*tolerance);
        };
        const auto adjoining = std::find_if(thinPairs.begin(),thinPairs.end(),[&](const auto& thin) {
            return sharesEnd(compact.first,thin.first) || sharesEnd(compact.first,thin.second) ||
                   sharesEnd(compact.second,thin.first) || sharesEnd(compact.second,thin.second);
        });
        if (adjoining==thinPairs.end()) continue;
        contacts.push_back(*adjoining);
        contacts.push_back(compact);
    }
    return contacts;
}

std::vector<CapPlane> sampledCapPlanes(const PointStore& residual,
                                       const std::vector<FiniteSurfacePatch>& patches, double tolerance) {
    struct AxisSupport { Vec3 axis; std::size_t observations; };
    std::vector<AxisSupport> axes;
    for (const auto& patch : patches) {
        if (!patch.planar || !patch.orientationKnown) continue;
        const auto found = std::find_if(axes.begin(),axes.end(),[&](const auto& entry) {
            return std::abs(dot(entry.axis,patch.normal))>.9999;
        });
        if (found==axes.end()) axes.push_back({patch.normal,patch.workingPointIds.size()});
        else found->observations += patch.workingPointIds.size();
    }
    std::stable_sort(axes.begin(),axes.end(),[](const auto& a, const auto& b) { return a.observations>b.observations; });
    if (axes.size()>8) axes.resize(8);

    struct Hit { double level; Vec3 position; };
    std::vector<CapPlane> planes;
    for (const auto& entry : axes) for (const int sign : {-1,1}) {
        const auto normal = scale(entry.axis,sign);
        std::vector<Hit> hits;
        for (const auto& sample : residual.points()) {
            if (!sample.normal || !sample.normalOrientationKnown || length(*sample.normal)<1e-12 ||
                dot(normal,*sample.normal)<.995*length(*sample.normal)) continue;
            hits.push_back({dot(normal,sample.position),sample.position});
        }
        std::sort(hits.begin(),hits.end(),[](const auto& a, const auto& b) { return a.level<b.level; });
        std::vector<CapPlane> axisPlanes;
        for (std::size_t first=0; first<hits.size();) {
            std::size_t end = first;
            double levelSum = 0;
            CapPlane cap{normal,{},hits[first].position,hits[first].position,0};
            while (end<hits.size() && hits[end].level-hits[first].level<=tolerance) {
                const auto& hit = hits[end++];
                levelSum += hit.level;
                const auto p = hit.position;
                cap.lower = {std::min(cap.lower.x,p.x),std::min(cap.lower.y,p.y),std::min(cap.lower.z,p.z)};
                cap.upper = {std::max(cap.upper.x,p.x),std::max(cap.upper.y,p.y),std::max(cap.upper.z,p.z)};
            }
            cap.observations = end-first;
            if (cap.observations>=48) {
                cap.origin = scale(normal,levelSum/cap.observations);
                axisPlanes.push_back(cap);
            }
            first = end;
        }
        std::stable_sort(axisPlanes.begin(),axisPlanes.end(),[](const auto& a, const auto& b) {
            return length(subtract(a.upper,a.lower))>length(subtract(b.upper,b.lower));
        });
        if (axisPlanes.size()>12) axisPlanes.resize(12);
        planes.insert(planes.end(),axisPlanes.begin(),axisPlanes.end());
    }
    return planes;
}

std::vector<CapPair> residualCaps(const PointStore& residual, double tolerance) {
    PrimitiveDetectionOptions options;
    options.detectCylinders = options.detectSpheres = options.detectCones = options.detectTori = false;
    options.plane.distanceThreshold = tolerance;
    options.plane.minimumNormalAlignment = .98;
    options.plane.minimumSupportPoints = 100;
    options.plane.maximumPlanes = 16;
    PrimitiveDetectionResult proposals;
    std::string error;
    if (!detectPrimitiveEvidence(residual,options,proposals,error)) return {};
    std::vector<CapPlane> planes;
    for (const auto& item : proposals.evidence) {
        const auto& plane = std::get<PlaneEvidence>(item).plane;
        const auto origin = scale(plane.normal,-plane.offset);
        for (const int sign : {-1,1}) {
            CapPlane cap{scale(plane.normal,sign),origin,{INFINITY,INFINITY,INFINITY},{-INFINITY,-INFINITY,-INFINITY},0};
            for (const auto& p : residual.points()) {
                if (!p.normal || !p.normalOrientationKnown ||
                    std::abs(dot(plane.normal,p.position)+plane.offset)>tolerance ||
                    dot(cap.normal,*p.normal)<.98*length(*p.normal) || length(*p.normal)<1e-12) continue;
                const auto q = p.position;
                cap.lower = {std::min(cap.lower.x,q.x),std::min(cap.lower.y,q.y),std::min(cap.lower.z,q.z)};
                cap.upper = {std::max(cap.upper.x,q.x),std::max(cap.upper.y,q.y),std::max(cap.upper.z,q.z)};
                ++cap.observations;
            }
            if (cap.observations>=options.plane.minimumSupportPoints) planes.push_back(cap);
        }
    }
    const auto pair = strongestCapPair(planes,tolerance);
    return pair ? std::vector<CapPair>{*pair} : std::vector<CapPair>{};
}

struct CapFootprint {
    Vec3 lower{INFINITY,INFINITY,INFINITY}, upper{-INFINITY,-INFINITY,-INFINITY};
    std::size_t observations[2]{};

    void include(Vec3 low, Vec3 high) {
        lower = {std::min(lower.x,low.x),std::min(lower.y,low.y),std::min(lower.z,low.z)};
        upper = {std::max(upper.x,high.x),std::max(upper.y,high.y),std::max(upper.z,high.z)};
    }
};

std::vector<CapFootprint> connectedCapFootprints(const CapPair& caps, const PointStore& residual,
                                                  double tolerance) {
    const auto normal = caps.first.normal;
    const auto basis = std::abs(normal.x)<.8 ? Vec3{1,0,0} : Vec3{0,1,0};
    const auto u = normalized(cross(normal,basis)), v = cross(normal,u);
    const auto depth = dot(normal,subtract(caps.first.origin,caps.second.origin));
    const auto span = std::min(length(subtract(caps.first.upper,caps.first.lower)),
                               length(subtract(caps.second.upper,caps.second.lower)));
    const auto support = caps.first.observations+caps.second.observations;
    const auto spacing = std::clamp(span/std::sqrt(double(support)),depth,span/4);
    using CellIndex = std::pair<int,int>;
    std::map<CellIndex,CapFootprint> cells;
    const auto origin = caps.first.lower;
    for (const auto& p : residual.points()) {
        if (!p.normal || !p.normalOrientationKnown) continue;
        const auto normalLength = length(*p.normal);
        if (normalLength<1e-12) continue;
        const auto local = subtract(p.position,origin);
        const CapPlane* ends[] = {&caps.first,&caps.second};
        for (int i=0; i<2; ++i) {
            const auto& end = *ends[i];
            if (std::abs(dot(end.normal,subtract(p.position,end.origin)))>tolerance ||
                dot(end.normal,*p.normal)<.995*normalLength) continue;
            auto& cell = cells[{int(std::floor(dot(local,u)/spacing)),int(std::floor(dot(local,v)/spacing))}];
            cell.include(p.position,p.position);
            ++cell.observations[i];
        }
    }
    std::vector<CapFootprint> footprints;
    while (!cells.empty()) {
        std::vector<CellIndex> queue;
        CapFootprint component;
        const auto visit = [&](CellIndex index) {
            const auto cell = cells.find(index);
            if (cell==cells.end()) return;
            component.include(cell->second.lower,cell->second.upper);
            for (int i=0; i<2; ++i) component.observations[i] += cell->second.observations[i];
            queue.push_back(index);
            cells.erase(cell);
        };
        visit(cells.begin()->first);
        for (std::size_t i=0; i<queue.size(); ++i) {
            const auto [x,y] = queue[i];
            for (int dx=-1; dx<=1; ++dx)
                for (int dy=-1; dy<=1; ++dy)
                    if (dx || dy) visit({x+dx,y+dy});
        }
        if (std::min(component.observations[0],component.observations[1])<48) continue;
        const auto padding = .5*spacing;
        const Vec3 border{padding,padding,padding};
        component.lower = subtract(component.lower,border);
        component.upper = add(component.upper,border);
        footprints.push_back(component);
    }
    std::stable_sort(footprints.begin(),footprints.end(),[](const auto& a, const auto& b) {
        return std::min(a.observations[0],a.observations[1])>std::min(b.observations[0],b.observations[1]);
    });
    return footprints;
}

BoundedSolid boundedSolid(const TopoDS_Shape& shape) {
    BoundedSolid result{shape, {}};
    BRepBndLib::AddOptimal(shape, result.bounds, false, false);
    return result;
}

bool conflictsWithAssembly(const BoundedSolid& solid, const std::vector<BoundedSolid>& existing,
                           const WorkingData& work, const std::unordered_set<PointId>& group,
                           double tolerance, const std::function<bool()>& cancelled) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(solid.shape, properties);
    const double allowedOverlap = std::max(properties.Mass() * 1e-6, tolerance * tolerance * tolerance);
    for (const auto& other : existing) {
        if (solid.bounds.IsOut(other.bounds)) continue;
        BRepAlgoAPI_Common intersection(solid.shape, other.shape);
        if (!intersection.IsDone()) return true;
        GProp_GProps overlap;
        // Boolean contact can leave open surface fragments beside closed
        // slivers. Only closed shells contribute material volume.
        BRepGProp::VolumeProperties(intersection.Shape(), overlap, true);
        if (overlap.Mass() > allowedOverlap) return true;
    }

    BRepClass3d_SolidClassifier classifier(solid.shape);
    const auto maximumConflicts = std::max<std::size_t>(2, group.size()/200);
    std::size_t conflicts = 0;
    for (std::size_t i = 0; i < work.points.size(); ++i) {
        if (i%4096 == 0 && cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const auto& sample = work.points[i];
        const auto& p = sample.position;
        const gp_Pnt point(p.x, p.y, p.z);
        if (solid.bounds.IsOut(point) || group.count(sample.id)) continue;
        classifier.Perform(point, tolerance);
        if (classifier.State() == TopAbs_IN && ++conflicts > maximumConflicts) return true;
    }
    return false;
}
}

void recoverLocalProfiles(const PointStore& source, const WorkingData& work, const AssemblyOptions& options,
                          AssemblyReconstructionResult& result, const std::function<bool()>& cancelled) {
    if (!options.buildSolids || !options.maximumLocalProfileAttempts ||
        source.size() <= work.points.size() || result.parts.size() < 2) return;

    std::vector<AssemblyPart*> pending;
    for (auto& part : result.parts) {
        if (!part.closed && part.orientationKnown && part.patchIds.size() >= 2 &&
            part.pointIds.size() >= options.minimumPartPointsForLocalRecovery &&
            part.pointIds.size() <= options.maximumWorkingPoints)
            pending.push_back(&part);
    }
    std::stable_sort(pending.begin(), pending.end(), [](const auto* a, const auto* b) {
        return a->pointIds.size() > b->pointIds.size();
    });
    if (pending.size() > options.maximumLocalProfileAttempts) pending.resize(options.maximumLocalProfileAttempts);
    if (pending.empty()) return;

    std::vector<BoundedSolid> accepted;
    if (!result.candidate.profileBodies.empty() || !result.candidate.curvedSolids.empty()) {
        modeling::ModelingCore core;
        if (!commitCompoundCandidate(result.candidate, core).success) return;
        for (TopExp_Explorer solids(core.bodyShape(), TopAbs_SOLID); solids.More(); solids.Next())
            accepted.push_back(boundedSolid(solids.Current()));
    }

    std::unordered_map<PointId, const PointSample*> observations;
    observations.reserve(source.size());
    for (const auto& sample : source.points()) observations.emplace(sample.id, &sample);
    auto localOptions = options;
    localOptions.maximumLocalProfileAttempts = 0;
    localOptions.maximumProjectionParts = 0;
    const auto appendSamples = [&](std::vector<PointSample>& samples, const std::vector<PointId>& ids) {
        for (const auto id : ids) samples.push_back(*observations.at(id));
    };
    enum class CropKind { Group, Signed, SignedEnvelope, Channel };
    const auto fitAndPublish = [&](AssemblyPart& part, std::vector<PointSample> samples,
                                   const std::vector<AssemblyPart*>& donors, CropKind crop = CropKind::Group) {
        const bool signedCrop = crop!=CropKind::Group;
        const bool checkEnvelopeCoverage = crop==CropKind::SignedEnvelope || crop==CropKind::Channel;
        const bool withContext = signedCrop || !donors.empty();
        const PointStore localSource(std::move(samples), source.unit());
        AssemblyReconstructionResult local;
        std::string error;
        if (!reconstructAssembly(localSource, localOptions, local, error, cancelled) ||
            local.candidate.profileBodies.size() != 1 || !local.candidate.curvedSolids.empty() ||
            local.candidate.surfaceCoverage < (withContext && !checkEnvelopeCoverage ? .98 : .80)) return;
        const auto closed = std::find_if(local.parts.begin(), local.parts.end(), [](const auto& candidate) {
            return candidate.closed;
        });
        if (closed == local.parts.end() || closed->status == "closed_circular_slab_inferred_contact") return;
        if (withContext && closed->status != "closed_analytic_profile" &&
            !(signedCrop && (closed->status == "closed_circular_slab" || closed->status == "closed_annular_tube")) &&
            !(part.pointIds.empty() && closed->status == "closed_solid")) return;

        auto& profile = local.candidate.profileBodies.front();
        if (crop==CropKind::Channel && profile.trimPlanes.empty()) return;
        TopoDS_Shape shape;
        if (!modeling::buildProfileBody(profile, shape, error)) return;
        auto solid = boundedSolid(shape);
        const std::unordered_set<PointId> support(closed->pointIds.begin(), closed->pointIds.end());
        const double tolerance = local.distanceTolerance * 4;
        if (withContext) {
            const auto supported = [&](const auto& ids, double fraction) {
                return std::count_if(ids.begin(),ids.end(),[&](auto id) { return support.count(id); }) >= fraction * ids.size();
            };
            double x0,y0,z0,x1,y1,z1;
            solid.bounds.Get(x0,y0,z0,x1,y1,z1);
            const double margin = std::max(tolerance, result.distanceTolerance * 4);
            if (!supported(part.pointIds,signedCrop ? .70 : .98) ||
                (!signedCrop && std::any_of(donors.begin(), donors.end(),
                    [&](auto* donor) { return !supported(donor->pointIds,.98); })) ||
                !contains(part.lower,part.upper,{x0,y0,z0},margin) || !contains(part.lower,part.upper,{x1,y1,z1},margin)) return;
        }
        if (local.candidate.surfaceCoverage < .98 && profile.trimPlanes.empty()) {
            // A separated group can contain an unrelated patch beyond the
            // profile. Leave those observations unassigned and require the same support
            // within the fitted envelope as for an isolated group.
            if (closed->status != "closed_analytic_profile" &&
                !(checkEnvelopeCoverage && closed->status == "closed_solid")) return;
            auto envelope = solid.bounds;
            envelope.Enlarge(tolerance);
            const auto unexplained = std::count_if(localSource.points().begin(), localSource.points().end(), [&](const auto& sample) {
                const auto& p = sample.position;
                return !support.count(sample.id) && !envelope.IsOut(gp_Pnt(p.x,p.y,p.z));
            });
            if (support.size() < .98 * (support.size() + unexplained)) return;
        }
        std::unordered_set<PointId> group;
        if (!withContext) {
            group.reserve(localSource.size());
            for (const auto& sample : localSource.points()) group.insert(sample.id);
        }
        if (conflictsWithAssembly(solid, accepted, work, withContext ? support : group, tolerance, cancelled)) return;

        const auto isSupported = [&](auto id) { return support.count(id) != 0; };
        const auto retainSupported = [&](std::vector<PointId>& ids, std::vector<PointId>& unassigned) {
            const auto unsupported = std::stable_partition(ids.begin(), ids.end(), isSupported);
            unassigned.insert(unassigned.end(), unsupported, ids.end());
            ids.erase(unsupported, ids.end());
        };
        for (auto* donor : donors) {
            if (signedCrop) {
                const auto transfer = [&](auto& ids, auto& destination) {
                    const auto split = std::stable_partition(ids.begin(),ids.end(),isSupported);
                    destination.insert(destination.end(),ids.begin(),split);
                    ids.erase(ids.begin(),split);
                };
                transfer(donor->workingPointIds,part.workingPointIds);
                donor->pointIds.erase(std::remove_if(donor->pointIds.begin(),donor->pointIds.end(),isSupported),
                                      donor->pointIds.end());
                auto& patchIds = donor->patchIds;
                patchIds.erase(std::remove_if(patchIds.begin(),patchIds.end(),[&](auto patchId) {
                    auto& patch = result.patches.at(patchId-1);
                    if (patch.pointIds.empty() || std::all_of(patch.pointIds.begin(),patch.pointIds.end(),isSupported)) {
                        part.patchIds.push_back(patchId);
                        return true;
                    }
                    patch.pointIds.erase(std::remove_if(patch.pointIds.begin(),patch.pointIds.end(),isSupported),
                                         patch.pointIds.end());
                    return false;
                }),patchIds.end());
                continue;
            }
            retainSupported(donor->pointIds, result.unassignedPointIds);
            retainSupported(donor->workingPointIds, result.unassignedWorkingPointIds);
            part.workingPointIds.insert(part.workingPointIds.end(), donor->workingPointIds.begin(), donor->workingPointIds.end());
            part.patchIds.insert(part.patchIds.end(), donor->patchIds.begin(), donor->patchIds.end());
            donor->pointIds.clear();
            donor->workingPointIds.clear();
            donor->patchIds.clear();
        }
        retainSupported(part.pointIds, result.unassignedPointIds);
        retainSupported(part.workingPointIds, result.unassignedWorkingPointIds);
        if (withContext) {
            auto& unassigned = result.unassignedPointIds;
            unassigned.erase(std::remove_if(unassigned.begin(), unassigned.end(), isSupported), unassigned.end());
            auto& working = result.unassignedWorkingPointIds;
            for (const auto id : working) if (isSupported(id)) part.workingPointIds.push_back(id);
            working.erase(std::remove_if(working.begin(), working.end(), isSupported), working.end());
            auto& workingIds = part.workingPointIds;
            std::sort(workingIds.begin(), workingIds.end());
            workingIds.erase(std::unique(workingIds.begin(), workingIds.end()), workingIds.end());
            part.pointIds = closed->pointIds;
        }
        for (const auto patchId : part.patchIds) {
            auto& ids = result.patches.at(patchId-1).pointIds;
            ids.erase(std::remove_if(ids.begin(), ids.end(), [&](auto id) { return !isSupported(id); }), ids.end());
        }
        part.closed = true;
        part.lower = closed->lower;
        part.upper = closed->upper;
        part.openEdges = part.nonmanifoldEdges = 0;
        part.confidence = closed->confidence;
        part.status = closed->status;
        part.reason = signedCrop ? "Closed from full-resolution observations separated by signed cap planes"
                   : withContext ? "Closed from full-resolution observations including enclosed surface fragments"
                                 : "Closed from full-resolution observations in the separated group";
        result.candidate.profileBodies.push_back(std::move(profile));
        accepted.push_back(std::move(solid));
    };
    for (auto* part : pending) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        std::vector<PointSample> samples;
        samples.reserve(part->pointIds.size());
        appendSamples(samples, part->pointIds);
        fitAndPublish(*part, std::move(samples), {});
    }

    // An open slab can have internal bore walls that were separated into
    // individual patches. Retry with those fragments and unowned observations.
    std::size_t contextAttempts = 0;
    const auto contextBudget = std::min<std::size_t>(16, options.maximumLocalProfileAttempts);
    for (auto* part : pending) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        if (contextAttempts==contextBudget) break;
        if (part->closed) continue;
        const auto caps = opposingCaps(*part,result.patches,result.distanceTolerance);
        if (!caps) continue;
        std::vector<AssemblyPart*> donors;
        for (auto& candidate : result.parts) {
            if (candidate.closed || candidate.patchIds.size()!=1 || candidate.pointIds.empty()) continue;
            if (contains(part->lower,part->upper,candidate.lower) && contains(part->lower,part->upper,candidate.upper))
                donors.push_back(&candidate);
        }
        if (donors.empty()) continue;
        const auto assignedSize = std::accumulate(donors.begin(), donors.end(), part->pointIds.size(),
            [](auto total, const auto* donor) { return total + donor->pointIds.size(); });
        if (assignedSize>options.maximumWorkingPoints) continue;
        const double margin = result.distanceTolerance * 4;
        std::vector<PointSample> samples;
        samples.reserve(assignedSize);
        appendSamples(samples, part->pointIds);
        for (const auto* donor : donors) appendSamples(samples, donor->pointIds);
        for (const auto id : result.unassignedPointIds) {
            if (samples.size()>options.maximumWorkingPoints) break;
            const auto& sample = *observations.at(id);
            if (contains(part->lower,part->upper,sample.position,margin)) samples.push_back(sample);
        }
        if (samples.size()>options.maximumWorkingPoints) continue;
        std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return a.id<b.id; });
        ++contextAttempts;
        fitAndPublish(*part, std::move(samples), donors);
    }
    std::size_t signedAttempts = 0;
    const double margin = result.distanceTolerance * 4;
    std::unordered_map<std::size_t,std::size_t> previousCropSizes;
    const auto fitSignedContext = [&](AssemblyPart* part) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        if (signedAttempts == contextBudget || part->closed) return;
        const auto caps = opposingCaps(*part,result.patches,result.distanceTolerance);
        if (!caps) return;
        const auto overlaps = [&](const AssemblyPart& other) {
            return other.lower.x <= part->upper.x+margin && other.upper.x >= part->lower.x-margin &&
                   other.lower.y <= part->upper.y+margin && other.upper.y >= part->lower.y-margin &&
                   other.lower.z <= part->upper.z+margin && other.upper.z >= part->lower.z-margin;
        };
        std::vector<PointSample> samples;
        const auto appendAdmitted = [&](const std::vector<PointId>& ids) {
            for (const auto id : ids) {
                if (samples.size()>options.maximumWorkingPoints) return;
                const auto& sample = *observations.at(id);
                if (contains(part->lower,part->upper,sample.position,margin) && caps->admits(sample,margin))
                    samples.push_back(sample);
            }
        };
        appendAdmitted(part->pointIds);
        const auto retained = samples.size();
        if (retained < .70*part->pointIds.size()) return;
        std::vector<AssemblyPart*> donors;
        for (auto& candidate : result.parts) {
            if (&candidate == part || candidate.closed || !overlaps(candidate)) continue;
            const auto before = samples.size();
            appendAdmitted(candidate.pointIds);
            if (samples.size() != before) donors.push_back(&candidate);
        }
        appendAdmitted(result.unassignedPointIds);
        if (samples.size()>options.maximumWorkingPoints ||
            (retained>options.minimumRetainedFraction*part->pointIds.size() &&
             samples.size()-retained<options.minimumResidualSamples)) return;
        auto& previous = previousCropSizes[part->id];
        if (previous == samples.size()) return;
        previous = samples.size();
        std::sort(samples.begin(),samples.end(),[](const auto& a, const auto& b) { return a.id<b.id; });
        ++signedAttempts;
        fitAndPublish(*part,std::move(samples),donors,CropKind::Signed);
    };
    const auto solidsBeforeSigned = accepted.size();
    for (auto* part : pending) fitSignedContext(part);
    if (accepted.size() > solidsBeforeSigned) {
        // Closing a small neighbor can remove competing observations from
        // a larger candidate's envelope. Retry only crops that changed.
        for (auto* part : pending) fitSignedContext(part);
    }
    const auto byId = [](const auto& a, const auto& b) { return a.id<b.id; };
    std::size_t nextPartId = 1;
    for (const auto& part : result.parts) nextPartId = std::max(nextPartId,part.id+1);
    std::vector<AssemblyPart> recoveredSlabs;
    enum class SplitFootprint { Caps, Parent };
    const auto splitAlongCaps = [&](AssemblyPart* parent, const std::vector<CapPair>& pairs,
                                    std::size_t& attempts, std::size_t budget,
                                    SplitFootprint footprint = SplitFootprint::Caps) {
        bool parentSplit = false;
        for (const auto& caps : pairs) {
            if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
            if (attempts==budget || parent->pointIds.size()<options.minimumProfileSplitPoints) break;
            const auto& a = caps.first;
            const auto& b = caps.second;
            const Vec3 low{std::min(a.lower.x,b.lower.x),std::min(a.lower.y,b.lower.y),std::min(a.lower.z,b.lower.z)};
            const Vec3 high{std::max(a.upper.x,b.upper.x),std::max(a.upper.y,b.upper.y),std::max(a.upper.z,b.upper.z)};
            const auto padding = scale(subtract(high,low),.04);
            AssemblyPart part;
            part.id = nextPartId;
            part.orientationKnown = true;
            part.lower = footprint==SplitFootprint::Parent ? parent->lower : subtract(low,padding);
            part.upper = footprint==SplitFootprint::Parent ? parent->upper : add(high,padding);
            std::vector<PointSample> samples;
            const auto appendAdmitted = [&](const auto& ids) {
                for (const auto id : ids) {
                    if (samples.size()>options.maximumWorkingPoints) break;
                    const auto& sample = *observations.at(id);
                    if (contains(part.lower,part.upper,sample.position,margin) && caps.admits(sample,margin))
                        samples.push_back(sample);
                }
            };
            appendAdmitted(parent->pointIds);
            const auto parentSupport = samples.size();
            if (parentSupport<options.minimumProfileSplitPoints ||
                (!parentSplit && parentSupport>options.minimumParentSupportFraction*parent->pointIds.size())) continue;
            appendAdmitted(result.unassignedPointIds);
            std::vector<AssemblyPart*> donors{parent};
            for (auto& other : result.parts) {
                if (&other==parent || other.closed) continue;
                if (other.lower.x>part.upper.x+margin || other.upper.x<part.lower.x-margin ||
                    other.lower.y>part.upper.y+margin || other.upper.y<part.lower.y-margin ||
                    other.lower.z>part.upper.z+margin || other.upper.z<part.lower.z-margin) continue;
                const auto before = samples.size();
                appendAdmitted(other.pointIds);
                if (samples.size()>before) donors.push_back(&other);
            }
            const double minimumParentFraction = footprint==SplitFootprint::Parent ? .50 : .70;
            if (samples.size()>options.maximumWorkingPoints || parentSupport<minimumParentFraction*samples.size()) continue;
            std::sort(samples.begin(),samples.end(),byId);
            ++attempts;
            fitAndPublish(part,std::move(samples),donors,CropKind::Signed);
            if (!part.closed) continue;
            part.reason = "Closed by splitting a group along its observed opposing cap planes";
            recoveredSlabs.push_back(std::move(part));
            ++nextPartId;
            parentSplit = true;
        }
    };
    // Opposite normals on the same finite plane indicate a possible contact.
    // Try its adjoining intervals before the more general split search.
    std::size_t contactAttempts = 0;
    const auto contactBudget = std::min<std::size_t>(12,options.maximumLocalProfileAttempts);
    for (auto* parent : pending) {
        if (contactAttempts==contactBudget) break;
        if (parent->closed || parent->pointIds.size()<options.minimumParentSplitPoints) continue;
        const auto planes = partPlanes(*parent,result.patches,result.distanceTolerance);
        splitAlongCaps(parent,contactCapPairs(planes,result.distanceTolerance),contactAttempts,contactBudget);
    }
    std::size_t splitAttempts = 0;
    const auto splitBudget = std::min<std::size_t>(24,options.maximumLocalProfileAttempts);
    for (auto* parent : pending) {
        if (splitAttempts==splitBudget) break;
        if (parent->closed || parent->pointIds.size()<options.minimumParentSplitPoints) continue;
        const auto pairs = rankedCapPairs(partPlanes(*parent,result.patches,result.distanceTolerance),
                                          result.distanceTolerance,6);
        if (pairs.size()<2) continue;
        splitAlongCaps(parent,pairs,splitAttempts,splitBudget);
    }
    std::vector<AssemblyPart*> remainingParts;
    auto remainingCount = result.unassignedPointIds.size();
    for (auto& part : result.parts) {
        if (part.closed || part.pointIds.empty()) continue;
        remainingParts.push_back(&part);
        remainingCount += part.pointIds.size();
    }
    const auto forEachRemaining = [&](auto visit) {
        const auto visitIds = [&](const auto& ids) {
            for (const auto id : ids) if (!visit(*observations.at(id))) return false;
            return true;
        };
        for (const auto* part : remainingParts) if (!visitIds(part->pointIds)) return false;
        return visitIds(result.unassignedPointIds);
    };
    const auto sampleRemaining = [&](std::size_t count, std::size_t budget) {
        const auto stride = 1+(count-1)/budget;
        std::vector<PointSample> proposals;
        proposals.reserve(1+(count-1)/stride);
        std::size_t offset = 0;
        forEachRemaining([&](const auto& sample) {
            if (offset++%stride==0) proposals.push_back(sample);
            return true;
        });
        std::sort(proposals.begin(),proposals.end(),byId);
        return PointStore(std::move(proposals),source.unit());
    };
    if (remainingCount>=options.minimumProfileSamples) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const bool sampled = remainingCount>options.maximumWorkingPoints;
        const auto proposalBudget = sampled ? std::min<std::size_t>(65536,options.maximumWorkingPoints) : remainingCount;
        const auto proposalSource = sampleRemaining(remainingCount,proposalBudget);
        auto pairs = sampled ? std::vector<CapPair>{} : residualCaps(proposalSource,margin);
        for (auto candidate : rankedCapPairs(sampledCapPlanes(proposalSource,result.patches,margin),margin,6)) {
            const auto samePlane = [&](const CapPlane& a, const CapPlane& b) {
                return dot(a.normal,b.normal)>.9999 && std::abs(dot(a.normal,subtract(a.origin,b.origin)))<=margin;
            };
            if (std::any_of(pairs.begin(),pairs.end(),[&](const auto& old) {
                return (samePlane(old.first,candidate.first) && samePlane(old.second,candidate.second)) ||
                       (samePlane(old.first,candidate.second) && samePlane(old.second,candidate.first));
            })) continue;
            pairs.push_back(std::move(candidate));
            if (pairs.size()==6) break;
        }
        for (const auto& caps : pairs) {
            std::vector<PointSample> samples;
            AssemblyPart part;
            part.id = nextPartId;
            part.orientationKnown = true;
            part.lower = {INFINITY,INFINITY,INFINITY};
            part.upper = {-INFINITY,-INFINITY,-INFINITY};
            const bool withinBudget = forEachRemaining([&](const auto& sample) {
                if (!caps.admits(sample,margin)) return true;
                if (samples.size()==options.maximumWorkingPoints) return false;
                samples.push_back(sample);
                const auto p = sample.position;
                part.lower = {std::min(part.lower.x,p.x),std::min(part.lower.y,p.y),std::min(part.lower.z,p.z)};
                part.upper = {std::max(part.upper.x,p.x),std::max(part.upper.y,p.y),std::max(part.upper.z,p.z)};
                return true;
            });
            if (!withinBudget || samples.size()<options.minimumProfileSamples) continue;
            std::sort(samples.begin(),samples.end(),byId);
            fitAndPublish(part,std::move(samples),remainingParts,CropKind::Signed);
            if (part.closed) {
                part.reason = "Closed from remaining observations between two independently fitted signed cap planes";
                recoveredSlabs.push_back(std::move(part));
                ++nextPartId;
                break;
            }
        }
    }
    // Small but well-sampled cap pairs can rank behind broad planes within
    // a joined group. Try them after the whole-cloud residual search.
    std::size_t concentratedAttempts = 0;
    const auto concentratedBudget = std::min<std::size_t>(12,options.maximumLocalProfileAttempts);
    for (auto* parent : pending) {
        if (concentratedAttempts==concentratedBudget) break;
        if (parent->closed || parent->pointIds.size()<options.minimumParentSplitPoints) continue;
        const auto planes = partPlanes(*parent,result.patches,result.distanceTolerance);
        const auto broad = rankedCapPairs(planes,result.distanceTolerance,6);
        if (broad.size()<2) continue;
        auto concentrated = rankedCapPairs(planes,result.distanceTolerance,6,4,CapRanking::Concentration);
        concentrated.erase(std::remove_if(concentrated.begin(),concentrated.end(),[&](const auto& pair) {
            return std::any_of(broad.begin(),broad.end(),[&](const auto& old) {
                return length(subtract(old.first.origin,pair.first.origin))<=margin &&
                       length(subtract(old.second.origin,pair.second.origin))<=margin;
            });
        }),concentrated.end());
        splitAlongCaps(parent,concentrated,concentratedAttempts,concentratedBudget);
    }
    // A sampled patch may cover only one corner of a broad face. Recount
    // signed plane observations inside the group's lateral bounds, including
    // nearby observations that the original grouping could not assign.
    std::size_t footprintAttempts = 0, footprintGroups = 0;
    const auto footprintBudget = std::min<std::size_t>(8,options.maximumLocalProfileAttempts);
    for (auto* parent : pending) {
        if (footprintAttempts==footprintBudget || footprintGroups==6) break;
        if (parent->closed || parent->pointIds.size()<options.minimumHugeParentPoints) continue;
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        ++footprintGroups;
        constexpr std::size_t voteBudget = 16384;
        const auto stride = 1+(parent->pointIds.size()+result.unassignedPointIds.size()-1)/voteBudget;
        std::vector<PointSample> votes;
        votes.reserve(voteBudget);
        std::size_t index = 0;
        const auto appendVotes = [&](const auto& ids) {
            for (const auto id : ids) {
                if (index++%stride!=0) continue;
                const auto& sample = *observations.at(id);
                if (contains(parent->lower,parent->upper,sample.position,margin)) votes.push_back(sample);
            }
        };
        appendVotes(parent->pointIds);
        appendVotes(result.unassignedPointIds);
        if (votes.size()<options.minimumProfileSamples) continue;
        const auto planes = sampledCapPlanes(PointStore(std::move(votes),source.unit()),result.patches,margin);
        auto pairs = rankedCapPairs(planes,margin,12);
        const auto parentSpan = length(subtract(parent->upper,parent->lower));
        pairs.erase(std::remove_if(pairs.begin(),pairs.end(),[&](const auto& pair) {
            const auto span = std::min(length(subtract(pair.first.upper,pair.first.lower)),
                                       length(subtract(pair.second.upper,pair.second.lower)));
            return span<.6*parentSpan;
        }),pairs.end());
        if (pairs.size()>options.maximumBroadPairs) pairs.resize(options.maximumBroadPairs);
        splitAlongCaps(parent,pairs,footprintAttempts,footprintBudget,SplitFootprint::Parent);
    }
    // A slab can cross several initial groups. Localize signed cap votes
    // before admitting full-resolution observations from those groups.
    remainingCount = std::accumulate(remainingParts.begin(),remainingParts.end(),result.unassignedPointIds.size(),
        [](auto total, const auto* part) { return total+part->pointIds.size(); });
    if (remainingCount>=options.minimumProfileSamples) {
        if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
        const auto proposalSource = sampleRemaining(remainingCount,std::min<std::size_t>(65536,options.maximumWorkingPoints));
        const auto planes = sampledCapPlanes(proposalSource,result.patches,margin);
        auto pairs = rankedCapPairs(planes,margin,24);
        pairs.erase(std::remove_if(pairs.begin(),pairs.end(),[](const auto& pair) {
            const auto [low,high] = std::minmax(pair.first.observations,pair.second.observations);
            return low<.30*high;
        }),pairs.end());
        if (pairs.size()>options.maximumProfilePairs) pairs.resize(options.maximumProfilePairs);
        std::vector<std::vector<CapFootprint>> footprints;
        for (const auto& pair : pairs) footprints.push_back(connectedCapFootprints(pair,proposalSource,margin));
        std::size_t attempts = 0;
        const auto budget = std::min<std::size_t>(8,options.maximumLocalProfileAttempts);
        for (std::size_t rank=0; rank<budget && attempts<budget; ++rank) {
            bool triedFootprint = false;
            for (std::size_t i=0; i<pairs.size() && attempts<budget; ++i) {
                if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
                if (rank>=footprints[i].size()) continue;
                triedFootprint = true;
                const auto& caps = pairs[i];
                const auto& footprint = footprints[i][rank];
                AssemblyPart part;
                part.id = nextPartId;
                part.orientationKnown = true;
                part.lower = footprint.lower;
                part.upper = footprint.upper;
                std::vector<PointSample> samples;
                const bool withinBudget = forEachRemaining([&](const auto& sample) {
                    if (!contains(part.lower,part.upper,sample.position,margin) || !caps.admits(sample,margin)) return true;
                    if (samples.size()==options.maximumWorkingPoints) return false;
                    samples.push_back(sample);
                    return true;
                });
                if (!withinBudget || samples.size()<options.minimumProfileSamples) continue;
                std::sort(samples.begin(),samples.end(),byId);
                ++attempts;
                fitAndPublish(part,std::move(samples),remainingParts,CropKind::SignedEnvelope);
                if (!part.closed) continue;
                part.reason = "Closed from a connected footprint of opposing caps across multiple groups";
                recoveredSlabs.push_back(std::move(part));
                ++nextPartId;
            }
            if (!triedFootprint) break;
        }
    }
    // The four signed flank planes can separate adjoining bent members and
    // let the fit see transverse observations that the grouping left out.
    std::size_t channelAttempts = 0;
    const auto channelBudget = std::min<std::size_t>(8,options.maximumLocalProfileAttempts);
    for (auto* parent : pending) {
        if (channelAttempts==channelBudget) break;
        if (parent->closed || parent->pointIds.size()<options.minimumLargeParentPoints) continue;
        const auto planes = partPlanes(*parent,result.patches,margin);
        if (planes.size()<options.minimumPlaneCount || planes.size()>options.maximumPlaneCount) continue;
        for (const auto& window : channelWindows(planes,margin)) {
            if (channelAttempts==channelBudget) break;
            if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
            AssemblyPart part;
            part.id = nextPartId;
            part.orientationKnown = true;
            part.lower = {INFINITY,INFINITY,INFINITY};
            part.upper = {-INFINITY,-INFINITY,-INFINITY};
            std::vector<PointSample> samples;
            const auto withinBudget = forEachRemaining([&](const auto& sample) {
                const auto axial = dot(window.along,sample.position);
                if (axial<window.start || axial>window.end ||
                    !window.flanks.admits(sample,margin) || !window.vertical.admits(sample,margin)) return true;
                if (samples.size()==options.maximumWorkingPoints) return false;
                samples.push_back(sample);
                const auto p = sample.position;
                part.lower = {std::min(part.lower.x,p.x),std::min(part.lower.y,p.y),std::min(part.lower.z,p.z)};
                part.upper = {std::max(part.upper.x,p.x),std::max(part.upper.y,p.y),std::max(part.upper.z,p.z)};
                return true;
            });
            if (!withinBudget || samples.size()<options.minimumLargeProfileSamples) continue;
            const auto axialPad = scale({std::abs(window.along.x),std::abs(window.along.y),std::abs(window.along.z)},
                                        window.gauge);
            part.lower = subtract(part.lower,axialPad);
            part.upper = add(part.upper,axialPad);
            std::sort(samples.begin(),samples.end(),byId);
            ++channelAttempts;
            fitAndPublish(part,std::move(samples),remainingParts,CropKind::Channel);
            if (!part.closed) continue;
            part.reason = "Closed from a channel footprint supported by paired flank planes";
            recoveredSlabs.push_back(std::move(part));
            ++nextPartId;
        }
    }
    result.parts.insert(result.parts.end(),std::make_move_iterator(recoveredSlabs.begin()),
                        std::make_move_iterator(recoveredSlabs.end()));
    result.parts.erase(std::remove_if(result.parts.begin(), result.parts.end(), [](const auto& part) {
        return part.pointIds.empty() && part.patchIds.empty();
    }), result.parts.end());
    std::size_t closedPoints = 0;
    for (const auto& part : result.parts) if (part.closed) closedPoints += part.pointIds.size();
    result.candidate.surfaceCoverage = result.candidate.confidence = double(closedPoints) / source.size();
}
}
