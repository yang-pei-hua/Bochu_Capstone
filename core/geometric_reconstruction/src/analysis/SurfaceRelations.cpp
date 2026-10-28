#include "detail/AnalysisInternal.h"

namespace reconstruction {
using namespace detail::analysis;

const char* surfaceRelationName(SurfaceRelationKind k) noexcept {
    switch (k) {
    case SurfaceRelationKind::Adjacent: return "adjacent";
    case SurfaceRelationKind::Parallel: return "parallel";
    case SurfaceRelationKind::Perpendicular: return "perpendicular";
    case SurfaceRelationKind::Coplanar: return "coplanar";
    case SurfaceRelationKind::Coaxial: return "coaxial";
    case SurfaceRelationKind::SameRadius: return "same_radius";
    case SurfaceRelationKind::Intersect: return "intersect";
    case SurfaceRelationKind::Tangent: return "tangent";
    case SurfaceRelationKind::AxisNormalToPlane: return "axis_normal_to_plane";
    case SurfaceRelationKind::AxisParallelToPlane: return "axis_parallel_to_plane";
    }
    return "unknown";
}

bool hasSurfaceRelation(const SurfaceAnalysisResult& a,SurfaceId first,SurfaceId second,SurfaceRelationKind kind) noexcept {
    for(const auto& r:a.relations) {
        if(r.kind!=kind || r.first==0 || r.second==0 || r.first>a.patches.size() || r.second>a.patches.size()) continue;
        const auto x=a.patches[static_cast<std::size_t>(r.first-1)].surfaceId;
        const auto y=a.patches[static_cast<std::size_t>(r.second-1)].surfaceId;
        if((x==first && y==second) || (x==second && y==first)) return true;
    }
    return false;
}

bool inferSurfaceRelations(const PointStore& points, const std::vector<RelationSurface>& nodes,
                           const SurfaceRelationOptions& options, std::vector<SurfaceRelation>& output,
                           std::string& error) {
    output.clear(); error.clear();
    SurfaceAnalysisOptions validation; validation.relations = options;
    if (!valid(validation)) { error = "Invalid surface relationship tolerances"; return false; }
    const auto map = pointMap(points);
    std::vector<std::vector<const PointSample*>> samples(nodes.size());
    std::vector<Grid> grids; grids.reserve(nodes.size());
    std::unordered_set<PatchId> unique;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (!nodes[i].id || !unique.insert(nodes[i].id).second) { error = "Duplicate or invalid relation node ID"; return false; }
        for (auto id : nodes[i].supportPointIds) {
            auto it = map.find(id);
            if (it == map.end()) { error = "Relationship support references a different point snapshot"; return false; }
            samples[i].push_back(it->second);
        }
        grids.emplace_back(samples[i], options.distanceTolerance);
    }
    const double parallel = std::cos(options.angularToleranceRadians), perpendicular = std::sin(options.angularToleranceRadians);
    for (std::size_t i = 0; i < nodes.size(); ++i) for (std::size_t j = i+1; j < nodes.size(); ++j) {
        if (samples[i].empty() || samples[j].empty()) continue;
        const auto small = samples[i].size() < samples[j].size() ? i : j, large = small == i ? j : i;
        bool adjacent = false;
        for (auto p : samples[small]) {
            grids[large].near(p->position, [&](std::size_t) { adjacent = true; });
            if (adjacent) break;
        }
        auto append = [&](SurfaceRelationKind kind, double deviation, double c) {
            output.push_back({nodes[i].id, nodes[j].id, kind, deviation, c, true});
        };
        if (adjacent) append(SurfaceRelationKind::Adjacent, 0, 0.75);
        if (!nodes[i].evidence || !nodes[j].evidence) continue;
        const auto& a = *nodes[i].evidence; const auto& b = *nodes[j].evidence;
        const auto ka = primitiveKind(a), kb = primitiveKind(b);
        const double c = std::clamp(std::min({confidence(a), confidence(b), 0.95}), 0.0, 1.0);
        const auto u = axis(a), v = axis(b);
        const double alignment = std::abs(dot(u, v));
        if (ka == PrimitiveKind::Plane && kb == PrimitiveKind::Plane) {
            if (alignment >= parallel) {
                append(SurfaceRelationKind::Parallel, std::acos(std::min(1.0, alignment)), c);
                const auto& pa = std::get<PlaneEvidence>(a).plane;
                const auto& pb = std::get<PlaneEvidence>(b).plane;
                const double d = std::abs(pa.offset - (dot(u, v) < 0 ? -pb.offset : pb.offset));
                if (d <= options.distanceTolerance) append(SurfaceRelationKind::Coplanar, d, c);
            } else {
                if (alignment <= perpendicular) append(SurfaceRelationKind::Perpendicular, std::asin(alignment), c);
                if (adjacent) append(SurfaceRelationKind::Intersect, 0, c*0.8);
            }
        }
        if (axial(ka) && axial(kb) && alignment >= parallel) {
            const auto d = subtract(center(a), center(b));
            const double deviation = length(subtract(d, scale(u, dot(d, u))));
            if (deviation <= options.distanceTolerance) append(SurfaceRelationKind::Coaxial, deviation, c);
        }
        if ((ka == PrimitiveKind::Plane && axial(kb)) || (kb == PrimitiveKind::Plane && axial(ka))) {
            if (alignment >= parallel) append(SurfaceRelationKind::AxisNormalToPlane, std::acos(std::min(1.0, alignment)), c);
            if (alignment <= perpendicular) append(SurfaceRelationKind::AxisParallelToPlane, std::asin(alignment), c);
        }
        const double ra = radius(a), rb = radius(b);
        if (ka == kb && ra > 0 && rb > 0 && std::abs(ra-rb) <= options.distanceTolerance)
            append(SurfaceRelationKind::SameRadius, std::abs(ra-rb), c);
        auto tangentPlane = [&](const PrimitiveEvidence& plane, const PrimitiveEvidence& curved) {
            const auto* p = std::get_if<PlaneEvidence>(&plane);
            const auto k = primitiveKind(curved);
            if (!p || (k != PrimitiveKind::Sphere && k != PrimitiveKind::Cylinder)) return false;
            if (k == PrimitiveKind::Cylinder && std::abs(dot(p->plane.normal, axis(curved))) > perpendicular) return false;
            return std::abs(std::abs(dot(p->plane.normal, center(curved))+p->plane.offset)-radius(curved)) <= options.distanceTolerance;
        };
        bool tangent = tangentPlane(a,b) || tangentPlane(b,a);
        if (ka == PrimitiveKind::Sphere && kb == PrimitiveKind::Sphere) {
            const double d = length(subtract(center(a), center(b)));
            tangent = std::abs(d-ra-rb) <= options.distanceTolerance ||
                (d > options.distanceTolerance && std::abs(d-std::abs(ra-rb)) <= options.distanceTolerance);
        }
        if (tangent && adjacent) append(SurfaceRelationKind::Tangent, 0, c*0.8);
    }
    return true;
}

bool applySurfaceRelationHints(const PointStore& points,const std::vector<SurfaceRelationHint>& hints,
                              SurfaceAnalysisResult& analysis,std::string& error) {
    error.clear();
    if(!analysisMatches(points,analysis)) {error="Stale relationship analysis";return false;}
    analysis.relations.erase(std::remove_if(analysis.relations.begin(),analysis.relations.end(),
        [](const auto& r){return !r.automatic;}),analysis.relations.end());
    const auto observations=pointMap(points);
    auto patchFor=[&](const std::vector<PointId>& ids) -> PatchId {
        std::unordered_set<PointId> eligible;
        for(auto id:ids) if(observations.count(id)) eligible.insert(id);
        PatchId best=0;std::size_t overlap=0;
        for(const auto& p:analysis.patches) {
            std::size_t shared=0;for(auto id:p.supportPointIds) shared+=eligible.count(id);
            if(shared>overlap && shared>=0.6*std::min(eligible.size(),p.supportPointIds.size())) {best=p.id;overlap=shared;}
        }
        return best;
    };
    std::vector<SurfaceRelation> confirmed;
    for(const auto& h:hints) {
        const auto first=patchFor(h.firstPoints),second=patchFor(h.secondPoints);
        if(!first || !second || first==second) {error="Manual relation requires two identifiable current surface patches";return false;}
        const auto it=std::find_if(analysis.relations.begin(),analysis.relations.end(),[&](const auto& r){
            return r.automatic && r.kind==h.kind && ((r.first==first && r.second==second) || (r.first==second && r.second==first));
        });
        if(it==analysis.relations.end()) {error=std::string("Manual relation conflicts with observed geometry: ")+surfaceRelationName(h.kind);return false;}
        auto relation=*it;relation.automatic=false;confirmed.push_back(relation);
    }
    analysis.relations.insert(analysis.relations.end(),confirmed.begin(),confirmed.end());
    return true;
}
} // namespace reconstruction
