#include "detail/AnalysisInternal.h"

namespace reconstruction {
using namespace detail::analysis;
const char* localFailureReasonName(LocalFailureReason reason) noexcept {
    switch(reason) {
    case LocalFailureReason::InsufficientSupport:return "insufficient_support";
    case LocalFailureReason::MissingEndFace:return "missing_end_face";
    case LocalFailureReason::CandidateConflict:return "candidate_conflict";
    case LocalFailureReason::InvalidRelation:return "invalid_relation";
    }
    return "unknown";
}
bool recoverLocalSurfaces(const PointStore& working, const PointStore& source,
    const SurfaceAnalysisResult& analysis, LocalAnalysisFailure& failure,
    PointStore& local, SurfaceAnalysisResult& repaired, std::string& error) {
    error.clear(); repaired={}; failure.repaired=false;
    failure.recoveredPointIds.clear(); failure.repairedCandidates.clear();failure.candidates.clear();
    failure.repairedPatches.clear();failure.repairedRelations.clear();failure.rejectedRelations.clear();
    failure.sourceFingerprint=0;failure.localFingerprint=0;failure.detectionPasses=0;
    if(!analysisMatches(working,analysis) || working.unit()!=source.unit()) {
        error="Recovery requires matching working snapshot and source units"; return false;
    }
    const auto map=pointMap(working);
    Vec3 lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
    std::unordered_set<PointId> region;
    for(auto id:failure.pointIds) {
        auto it=map.find(id);
        if(it==map.end()) {error="Foreign recovery point ID";return false;}
        region.insert(id); const auto p=it->second->position;
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    if(region.empty()) {error="Empty recovery region";return false;}
    std::unordered_set<PointId> verified;
    for(const auto& p:source.points()) {
        if(region.count(p.id))verified.insert(p.id);
        auto it=map.find(p.id);
        if(it!=map.end() && length(subtract(p.position,it->second->position))>1e-8*std::max(1.0,length(p.position))) {
            error="Recovery source uses a different coordinate frame or point identity";return false;
        }
    }
    if(verified.size()!=region.size()) {error="Recovery source does not contain failed-region observations";return false;}
    failure.sourceFingerprint=pointStoreFingerprint(source);
    // The working observations (including estimated signed normals) win on ID.
    std::vector<PointSample> samples;
    for(const auto& p:working.points()) if(region.count(p.id)) samples.push_back(p);
    const double tolerance=analysis.options.detection.plane.distanceThreshold;
    const double halo=region.size()<20 ? std::max(tolerance*2,analysis.options.relations.distanceTolerance*16) : tolerance*2;
    std::vector<PlaneEvidence> stablePlanes;
    for(const auto& e:analysis.surfaces)if(const auto* plane=std::get_if<PlaneEvidence>(&e)) {
        std::size_t overlap=0;for(auto id:plane->supportPointIds)overlap+=region.count(id);
        if(overlap<plane->supportPointIds.size()*0.1)stablePlanes.push_back(*plane);
    }
    for(const auto& p:source.points()) {
        if(region.count(p.id)) continue;
        const auto q=p.position;
        if(q.x<lo.x-halo || q.x>hi.x+halo || q.y<lo.y-halo || q.y>hi.y+halo || q.z<lo.z-halo || q.z>hi.z+halo) continue;
        // Unsigned source normals cannot establish material orientation.
        const auto found=map.find(p.id);
        const auto& observation=found==map.end()?p:*found->second;
        if(!observation.normal || !observation.normalOrientationKnown) continue;
        bool stableObservation=false;
        for(const auto& plane:stablePlanes) {
            if(surfaceResidual(plane,q)<=tolerance*2 && std::abs(dot(normalized(*observation.normal),normalized(plane.plane.normal)))>=0.95) {stableObservation=true;break;}
        }
        if(stableObservation)continue;
        samples.push_back(observation); failure.recoveredPointIds.push_back(p.id);
    }
    if(region.size()<20 && samples.size()>region.size()) {
        PointStore neighborhood(samples,working.unit());std::vector<PointId> available;
        for(const auto& p:samples)available.push_back(p.id);
        std::unordered_set<PointId> connected;
        for(const auto& component:connectedPointComponents(neighborhood,available,analysis.options.relations.distanceTolerance*2,analysis.options.relations.distanceTolerance*4)) {
            if(std::any_of(component.begin(),component.end(),[&](auto id){return region.count(id)>0;}))connected.insert(component.begin(),component.end());
        }
        samples.erase(std::remove_if(samples.begin(),samples.end(),[&](const auto& p){return !connected.count(p.id);}),samples.end());
        auto& recovered=failure.recoveredPointIds;
        recovered.erase(std::remove_if(recovered.begin(),recovered.end(),[&](auto id){return !connected.count(id);}),recovered.end());
    }
    local=PointStore(std::move(samples),working.unit());
    failure.candidates.clear();
    std::vector<PrimitiveEvidence> retained;
    for(auto e:analysis.surfaces) {
        std::visit([&](auto& s) {
            auto& support=s.supportPointIds;
            support.erase(std::remove_if(support.begin(),support.end(),[&](auto id){return !region.count(id);}),support.end());
        },e);
        if(!ids(e).empty()) {retained.push_back(e);failure.candidates.push_back(e);}
    }
    auto options=analysis.options;
    auto& d=options.detection;
    d.plane.maximumPlanes=std::min<std::size_t>(24,d.plane.maximumPlanes);
    d.cylinder.maximumCylinders=std::min<std::size_t>(8,d.cylinder.maximumCylinders);
    d.plane.minimumSupportPoints=std::min<std::size_t>(20,d.plane.minimumSupportPoints);
    d.cylinder.minimumSupportPoints=std::min<std::size_t>(20,d.cylinder.minimumSupportPoints);
    d.cone.minimumSupportPoints=std::min<std::size_t>(20,d.cone.minimumSupportPoints);
    // Two bounded proposal streams, retaining alternatives rather than replacing
    // a previous family. No distance or boundary-completeness threshold is relaxed.
    for(unsigned pass=0;pass<2;++pass) {
        options.detection.randomSeed=analysis.options.detection.randomSeed+pass+1;
        SurfaceAnalysisResult detected;std::string detectionError;
        if(analyzeSurfaces(local,options,detected,detectionError)) {
            failure.detectionPasses+=detected.detectionPasses;
            for(auto& e:detected.surfaces) {
                auto support=ids(e);std::sort(support.begin(),support.end());
                const bool duplicate=std::any_of(retained.begin(),retained.end(),[&](const auto& old) {
                    if(primitiveKind(old)!=primitiveKind(e)) return false;
                    auto other=ids(old);std::sort(other.begin(),other.end());return other==support;
                });
                if(!duplicate) retained.push_back(std::move(e));
            }
        }
    }
    if(retained.empty()) {error="No observed local surface candidates";return false;}
    if(!indexSurfaceEvidence(local,retained,analysis.options,repaired,error)) return false;
    RelationOptimizationReport optimization;
    const auto proposals=repaired.relations;
    if(!optimizeSurfaceRelations(local,repaired,proposals,optimization,error)) return false;
    failure.rejectedRelations=std::move(optimization.rejected);
    failure.repairedCandidates=repaired.surfaces;
    failure.repairedPatches=repaired.patches;failure.repairedRelations=repaired.relations;
    failure.localFingerprint=repaired.pointFingerprint;
    return true; // Only the solid recognizer may mark the failure repaired.
}
} // namespace reconstruction
