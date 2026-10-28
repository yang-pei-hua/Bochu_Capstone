#include "detail/AnalysisInternal.h"
#include "preprocessing/detail/KdTree.h"

namespace reconstruction {
using namespace detail::analysis;

std::vector<std::vector<PointId>> connectedPointComponents(const PointStore& points,
    const std::vector<PointId>& ids, double distance, double maximumAdaptiveDistance) {
    std::vector<std::vector<PointId>> result;
    if (!std::isfinite(distance) || distance <= 0) return result;
    const auto map = pointMap(points);
    std::vector<const PointSample*> samples;
    for (auto id : ids) { auto it=map.find(id); if(it!=map.end()) samples.push_back(it->second); }
    if(maximumAdaptiveDistance>distance) {
        std::vector<PointSample> observations;for(auto p:samples)observations.push_back(*p);
        detail::preprocessing::KdTree tree(observations);
        std::vector<std::vector<std::pair<double,std::size_t>>> neighbors(samples.size());
        std::vector<double> radii(samples.size(),distance);
        std::vector<std::size_t> parent(samples.size());std::iota(parent.begin(),parent.end(),0);
        for(std::size_t i=0;i<samples.size();++i) {
            neighbors[i]=tree.nearest(i,16);
            if(!neighbors[i].empty())radii[i]=std::clamp(2*std::sqrt(neighbors[i][std::min<std::size_t>(5,neighbors[i].size()-1)].first),distance,maximumAdaptiveDistance);
        }
        auto root=[&](std::size_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
        for(std::size_t i=0;i<samples.size();++i)for(const auto& neighbor:neighbors[i]) {
            const auto j=neighbor.second;const double reach=std::min(radii[i],radii[j]);
            if(neighbor.first<=reach*reach)parent[root(j)]=root(i);
        }
        std::unordered_map<std::size_t,std::size_t> groups;
        for(std::size_t i=0;i<samples.size();++i) {
            auto inserted=groups.emplace(root(i),result.size());if(inserted.second)result.emplace_back();
            result[inserted.first->second].push_back(samples[i]->id);
        }
        return result;
    }
    Grid grid(samples,distance); std::vector<bool> visited(samples.size());
    for (std::size_t i=0;i<samples.size();++i) {
        if(visited[i]) continue;
        std::vector<std::size_t> queue{i}; visited[i]=true;
        std::vector<PointId> group;
        for(std::size_t head=0;head<queue.size();++head) {
            auto p=samples[queue[head]]; group.push_back(p->id);
            grid.near(p->position,[&](std::size_t next){if(!visited[next]) {visited[next]=true;queue.push_back(next);}});
        }
        result.push_back(std::move(group));
    }
    return result;
}

bool indexSurfaceEvidence(const PointStore& points, const std::vector<PrimitiveEvidence>& evidence,
                          const SurfaceAnalysisOptions& options, SurfaceAnalysisResult& output, std::string& error) {
    output = {}; error.clear();
    if (!valid(options)) { error = "Invalid surface analysis options"; return false; }
    output.pointFingerprint = pointStoreFingerprint(points); output.unit = points.unit(); output.options = options;
    output.surfaces = evidence;
    const auto map = pointMap(points);
    std::unordered_set<PointId> assigned;
    std::vector<RelationSurface> nodes;
    SurfaceId surface = 0;
    for (auto& e : output.surfaces) {
        std::visit([&](auto& s) { s.id = ++surface; }, e);
        std::vector<const PointSample*> samples;
        std::unordered_set<PointId> seen;
        Vec3 lower{INFINITY,INFINITY,INFINITY}, upper{-INFINITY,-INFINITY,-INFINITY};
        for (auto id : ids(e)) {
            const auto it = map.find(id);
            if (it == map.end()) { output = {}; error = "Surface support references a different point snapshot"; return false; }
            if (!seen.insert(id).second) continue;
            const auto p = it->second->position;
            lower = {std::min(lower.x,p.x),std::min(lower.y,p.y),std::min(lower.z,p.z)};
            upper = {std::max(upper.x,p.x),std::max(upper.y,p.y),std::max(upper.z,p.z)};
            samples.push_back(it->second); assigned.insert(id);
        }
        if (samples.empty()) continue;
        const auto d = subtract(upper,lower);
        const double area = 2*(d.x*d.y+d.y*d.z+d.z*d.x);
        const double radius = options.connectivityDistance > 0 ? options.connectivityDistance :
            std::max(options.relations.distanceTolerance*4, 2.5*std::sqrt(std::max(0.0,area)/samples.size()));
        Grid grid(samples, radius);
        std::vector<bool> visited(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) {
            if (visited[i]) continue;
            SurfacePatch patch; patch.id = output.patches.size()+1; patch.surfaceId = surface;
            patch.lower = patch.upper = samples[i]->position; patch.orientationKnown = true;
            std::vector<std::size_t> queue{i}; visited[i] = true;
            for (std::size_t head = 0; head < queue.size(); ++head) {
                const auto* p = samples[queue[head]]; const auto q = p->position;
                patch.supportPointIds.push_back(p->id); patch.centroid = add(patch.centroid,q);
                patch.lower = {std::min(patch.lower.x,q.x),std::min(patch.lower.y,q.y),std::min(patch.lower.z,q.z)};
                patch.upper = {std::max(patch.upper.x,q.x),std::max(patch.upper.y,q.y),std::max(patch.upper.z,q.z)};
                patch.orientationKnown &= p->normal.has_value() && p->normalOrientationKnown;
                if (p->normal) patch.materialNormal = add(patch.materialNormal,*p->normal);
                grid.near(q, [&](std::size_t next) { if (!visited[next]) { visited[next] = true; queue.push_back(next); } });
            }
            patch.centroid = scale(patch.centroid,1.0/queue.size());
            patch.materialNormal = scale(patch.materialNormal,1.0/queue.size());
            nodes.push_back({patch.id,e,patch.supportPointIds});
            output.patches.push_back(std::move(patch));
        }
    }
    for (const auto& p : points.points()) if (!assigned.count(p.id)) output.unassignedPointIds.push_back(p.id);
    return inferSurfaceRelations(points,nodes,options.relations,output.relations,error);
}
} // namespace reconstruction
