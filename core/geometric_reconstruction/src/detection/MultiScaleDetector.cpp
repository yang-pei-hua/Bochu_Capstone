#include "reconstruction/detection/MultiScaleDetector.h"
#include "preprocessing/detail/KdTree.h"
#include "common/Math3.h"
#include "reconstruction/analysis/SurfaceAnalysis.h"
#include <numeric>
#include <unordered_set>
#include <unordered_map>

namespace reconstruction {
bool detectMultiScalePrimitiveEvidence(const PointStore& points,const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,MultiScaleDetectionReport& report,std::string& error,
    const std::function<bool()>& cancelled) {
    output={};report={};error.clear();
    const std::array<bool,5> enabled{options.detectPlanes,options.detectCylinders,options.detectSpheres,options.detectCones,options.detectTori};
    const std::array<std::size_t,5> support{options.plane.minimumSupportPoints,options.cylinder.minimumSupportPoints,
        options.sphere.minimumSupportPoints,options.cone.minimumSupportPoints,options.torus.minimumSupportPoints};
    std::size_t minimumSupport=points.size()+1;
    for(int family=0;family<5;++family)if(enabled[family])minimumSupport=std::min(minimumSupport,support[family]);
    minimumSupport=std::max<std::size_t>(3,minimumSupport);
    std::unordered_set<PointId> assigned;
    std::vector<std::vector<PointId>> coherentRegions;
    auto detect=[&](const PointStore& local,bool recovery) {
        for(int family=0;family<5;++family) {
            if(!enabled[family])continue;
            // Nearly planar regions admit enormous sphere/torus approximations.
            // Test these families inside finite local regions instead of running
            // a global lottery over the entire mechanical assembly.
            if(!recovery && (family==2 || family==4))continue;
            if(cancelled && cancelled())return false;
            auto o=options;o.detectPlanes=family==0;o.detectCylinders=family==1;o.detectSpheres=family==2;
            o.detectCones=family==3;o.detectTori=family==4;
            PrimitiveDetectionResult result;std::string ignored;++report.detectionPasses;
            if(!detectPrimitiveEvidence(local,o,result,ignored))continue;
            for(auto& e:result.evidence) {
                std::visit([&](const auto& s){assigned.insert(s.supportPointIds.begin(),s.supportPointIds.end());},e);
                output.evidence.push_back(std::move(e));report.recoveredSurfaces+=recovery;
            }
        }
        return true;
    };
    if(!detect(points,false)){output={};error="Cancelled";return false;}
    // Rebuild on residual observations at each level. Dense small surfaces
    // otherwise lose the global sampling lottery against a sparse large host.
    for(int round=0;round<3;++round) {
        if(cancelled && cancelled()){output={};error="Cancelled";return false;}
        std::vector<PointSample> residual;
        for(const auto& p:points.points())if(p.normal && std::isfinite(math3::length(*p.normal)) &&
            math3::length(*p.normal)>1e-12 && (round==0 || !assigned.count(p.id)))residual.push_back(p);
        if(residual.size()<minimumSupport)break;
        detail::preprocessing::KdTree tree(residual);
        std::vector<std::size_t> parent(residual.size());std::iota(parent.begin(),parent.end(),0);
        auto root=[&](std::size_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
        const double alignment=round==0?0.94:0.85;
        for(std::size_t i=0;i<residual.size();++i) {
            if(i%256==0 && cancelled && cancelled()){output={};error="Cancelled";return false;}
            const auto n=math3::normalized(*residual[i].normal);
            for(const auto& neighbor:tree.nearest(i,24)) {
                const auto j=neighbor.second;
                if(math3::dot(n,math3::normalized(*residual[j].normal))>=alignment)
                    parent[root(j)]=root(i);
            }
        }
        std::unordered_map<std::size_t,std::vector<PointSample>> groups;
        for(std::size_t i=0;i<residual.size();++i)groups[root(i)].push_back(residual[i]);
        // Stable order gives reproducible local random streams and outputs.
        std::vector<std::vector<PointSample>> regions;
        for(auto& group:groups)if(group.second.size()>=minimumSupport)regions.push_back(std::move(group.second));
        std::sort(regions.begin(),regions.end(),[](const auto& a,const auto& b){return a.front().id<b.front().id;});
        const auto before=assigned.size();
        std::unordered_map<PointId,std::vector<std::size_t>> membership;
        if(round==0)for(std::size_t i=0;i<output.evidence.size();++i)
            std::visit([&](const auto& s){for(auto id:s.supportPointIds)membership[id].push_back(i);},output.evidence[i]);
        for(auto& region:regions) {
            if(round==0) {
                std::vector<PointId> ids;ids.reserve(region.size());
                for(const auto& p:region)ids.push_back(p.id);
                coherentRegions.push_back(std::move(ids));
                std::unordered_map<std::size_t,std::size_t> counts;std::size_t strongest=0;
                for(const auto& p:region) {
                    auto found=membership.find(p.id);if(found==membership.end())continue;
                    for(auto surface:found->second)strongest=std::max(strongest,++counts[surface]);
                }
                // Several approximate families can collectively cover a narrow
                // cone. Only one coherent surface may suppress local recovery.
                if(strongest>=region.size()*0.98)continue;
            }
            ++report.localRegions;PointStore local(std::move(region),points.unit());
            if(!detect(local,true)){output={};error="Cancelled";return false;}
        }
        if(assigned.size()==before)break;
    }
    // Resolve family competition over a coherent region, rather than letting
    // incidental per-point residual minima turn cone bands into giant tori or
    // noisy cylinders into tangent planes. Retain same-family alternatives.
    std::unordered_map<PointId,const PointSample*> observations;
    for(const auto& p:points.points())observations.emplace(p.id,&p);
    std::unordered_map<PointId,std::vector<std::size_t>> membership;
    for(std::size_t i=0;i<output.evidence.size();++i)
        std::visit([&](const auto& s){for(auto id:s.supportPointIds)membership[id].push_back(i);},output.evidence[i]);
    std::unordered_map<PointId,PrimitiveKind> preferred;
    const std::array<int,5> parameters{3,5,4,6,7};
    const std::array<double,5> tolerance{options.plane.distanceThreshold,options.cylinder.distanceThreshold,
        options.sphere.distanceThreshold,options.cone.distanceThreshold,options.torus.distanceThreshold};
    for(const auto& region:coherentRegions) {
        if(cancelled && cancelled()){output={};error="Cancelled";return false;}
        std::unordered_map<std::size_t,std::size_t> counts;
        for(auto id:region)for(auto i:membership[id])++counts[i];
        double best=INFINITY;std::optional<PrimitiveKind> chosen;
        for(const auto& count:counts)if(count.second>=region.size()*0.98) {
            const auto& e=output.evidence[count.first];const auto kind=primitiveKind(e);const auto k=int(kind);
            double cost=0;
            for(auto id:region) {
                const auto* p=observations.at(id);
                cost+=std::min(surfaceResidual(e,p->position),tolerance[k]*4)+
                    tolerance[k]*(1-surfaceNormalAlignment(e,p->position,*p->normal));
            }
            cost=cost/region.size()+parameters[k]*tolerance[k]*0.002;
            if(cost<best){best=cost;chosen=kind;}
        }
        if(chosen)for(auto id:region)preferred.emplace(id,*chosen);
    }
    assigned.clear();
    output.evidence.erase(std::remove_if(output.evidence.begin(),output.evidence.end(),[&](auto& e) {
        const auto kind=primitiveKind(e);const auto k=int(kind);
        return std::visit([&](auto& s) {
            auto& ids=s.supportPointIds;
            ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){const auto i=preferred.find(id);return i!=preferred.end() && i->second!=kind;}),ids.end());
            if(ids.size()<support[k])return true;
            double sum=0;s.maxAbsoluteResidual=0;
            for(auto id:ids){const double r=surfaceResidual(e,observations.at(id)->position);sum+=r;s.maxAbsoluteResidual=std::max(s.maxAbsoluteResidual,r);assigned.insert(id);}
            s.meanAbsoluteResidual=sum/ids.size();s.coverage=double(ids.size())/points.size();
            return false;
        },e);
    }),output.evidence.end());
    SurfaceId id=0;
    for(auto& e:output.evidence)std::visit([&](auto& s){s.id=++id;},e);
    for(const auto& p:points.points())if(!assigned.count(p.id))output.unassignedPointIds.push_back(p.id);
    if(output.evidence.empty()){error="No supported multiscale surfaces";return false;}
    return true;
}
}
