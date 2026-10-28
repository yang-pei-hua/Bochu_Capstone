#include "reconstruction/recognition/CompoundRecognizer.h"
#include "mixed/detail/MixedInternal.h"
#include <unordered_set>
namespace reconstruction {
namespace {
std::vector<double> signature(const CompoundCandidate& c,double directionScale) {
    std::vector<double> v;
    auto scalar=[&](double x){v.push_back(x);};
    auto vec=[&](modeling::Vec3 p){v.insert(v.end(),{p.x,p.y,p.z});};
    auto direction=[&](modeling::Vec3 p){vec(math3::scale(p,directionScale));};
    scalar(double(c.boxes.size()));scalar(double(c.prisms.size()));scalar(double(c.throughHoles.size()));scalar(double(c.curvedSolids.size()));
    for(const auto& b:c.boxes){const auto& p=b.primitive;vec(p.pose.origin);direction(p.pose.xDirection);direction(p.pose.zDirection);scalar(p.sizeX);scalar(p.sizeY);scalar(p.sizeZ);scalar(p.join);}
    for(const auto& p:c.prisms){vec(p.plane.origin);direction(p.plane.normal);direction(p.plane.xDirection);scalar(p.depth);scalar(p.subtract);scalar(double(p.polygon.size()));for(const auto& q:p.polygon){scalar(q.x);scalar(q.y);}}
    for(const auto& h:c.throughHoles){vec(h.primitive.axisPoint);direction(h.primitive.axisDirection);scalar(h.primitive.radius);}
    for(const auto& curved:c.curvedSolids){scalar(double(curved.primitive.index()));std::visit([&](const auto& p){
        using T=std::decay_t<decltype(p)>;
        if constexpr(std::is_same_v<T,modeling::SpherePrimitiveParams>){vec(p.center);scalar(p.radius);}
        else {if constexpr(std::is_same_v<T,modeling::TorusPrimitiveParams>)vec(p.center);else vec(p.baseCenter);
            direction(p.axisDirection);
            if constexpr(std::is_same_v<T,modeling::TorusPrimitiveParams>){scalar(p.majorRadius);scalar(p.minorRadius);}
            else {if constexpr(std::is_same_v<T,modeling::ConePrimitiveParams>){scalar(p.baseRadius);scalar(p.topRadius);}else scalar(p.radius);scalar(p.height);}
        }scalar(p.join);
    },curved.primitive);}
    return v;
}
double boundaryCoverage(const PointStore& points,const CompoundCandidate& c,double tolerance) {
    using namespace math3;std::size_t count=0;
    for(const auto& p:points.points())if(p.normal && p.normalOrientationKnown){auto shift=scale(normalized(*p.normal),tolerance*3);
        count+=detail::mixed::material(c,subtract(p.position,shift)) && !detail::mixed::material(c,add(p.position,shift));}
    return double(count)/std::max<std::size_t>(1,points.size());
}
}
bool recognizeAndRepairParametricSolid(const PointStore& points,SurfaceAnalysisResult& analysis,
    CompoundCandidate& output,std::string& error,const PointStore* source,RepairTransactionReport* report,
    const std::vector<SurfaceRelationHint>& hints) {
    output={};error.clear();RepairTransactionReport status;
    std::vector<LocalAnalysisFailure> failures;
    auto reject=[&](std::string reason){error=std::move(reason);status.reason=error;status.failures=failures;if(report)*report=status;return false;};
    PointStore observed({},points.unit());
    if(!materializeAnalysisPoints(points,analysis,observed,error))return reject(error);
    if(analysis.baseFingerprint && source && analysis.recoverySourceFingerprint!=pointStoreFingerprint(*source))return reject("Changed recovery source invalidates repaired analysis");
    auto staged=analysis;
    auto requested=hints;
    for(const auto& r:analysis.relations)if(!r.automatic && r.first && r.second && r.first<=analysis.patches.size() && r.second<=analysis.patches.size())
        requested.push_back({analysis.patches[r.first-1].supportPointIds,analysis.patches[r.second-1].supportPointIds,r.kind});
    if(!applySurfaceRelationHints(observed,requested,staged,error))return reject(error);
    CompoundCandidate baseline;
    if(!recognizeParametricSolidFromAnalysis(observed,staged,baseline,error,&failures,source))return reject(error);
    if(analysis.baseFingerprint) {
        if(std::any_of(failures.begin(),failures.end(),[](const auto& f){return f.repaired;}))return reject("Repaired snapshot needs an additional recovery transaction");
        status.reused=true;status.beforeCoverage=status.afterCoverage=baseline.surfaceCoverage;
        analysis=std::move(staged);output=std::move(baseline);if(report)*report=status;return true;
    }
    auto samples=points.points();std::unordered_set<PointId> seen;
    for(const auto& p:samples)seen.insert(p.id);
    auto evidence=analysis.surfaces;std::vector<SurfaceIdMapping> mappings;
    for(const auto& f:failures)if(f.repaired) {
        for(const auto& p:f.observations)if(seen.insert(p.id).second)samples.push_back(p);
        for(const auto& e:f.repairedCandidates){
            const auto local=std::visit([](const auto& s){return s.id;},e);
            evidence.push_back(e);mappings.push_back({f.localFingerprint,local,SurfaceId(evidence.size())});
        }
    }
    if(mappings.empty()){staged.localFailures=std::move(failures);analysis=std::move(staged);output=std::move(baseline);if(report)*report=status;return true;}
    PointStore combined(std::move(samples),points.unit());SurfaceAnalysisResult merged;
    if(!indexSurfaceEvidence(combined,evidence,analysis.options,merged,error))return reject(error);
    // Rebind manual constraints using observation IDs, never stale patch numbers.
    std::vector<SurfaceRelationHint> manual=hints;
    for(const auto& r:staged.relations)if(!r.automatic && r.first && r.second && r.first<=staged.patches.size() && r.second<=staged.patches.size())
        manual.push_back({staged.patches[r.first-1].supportPointIds,staged.patches[r.second-1].supportPointIds,r.kind});
    if(!applySurfaceRelationHints(combined,manual,merged,error))return reject(error);
    CompoundCandidate checked;std::vector<LocalAnalysisFailure> retry;
    if(!recognizeParametricSolidFromAnalysis(combined,merged,checked,error,&retry,nullptr))return reject("Repair merge rejected: "+error);
    if(std::any_of(retry.begin(),retry.end(),[](const auto& f){return f.repaired;}))return reject("Merged analysis requires repeated recovery");
    status.beforeCoverage=boundaryCoverage(points,baseline,analysis.options.detection.plane.distanceThreshold);
    status.afterCoverage=boundaryCoverage(points,checked,analysis.options.detection.plane.distanceThreshold);
    if(baseline.boxes.size()!=checked.boxes.size() || baseline.prisms.size()!=checked.prisms.size() || baseline.throughHoles.size()!=checked.throughHoles.size() || baseline.curvedSolids.size()!=checked.curvedSolids.size())return reject("Repair changed feature counts");
    for(const auto& f:failures)if(f.repaired) {
        PointStore local(f.observations,points.unit());const double tolerance=analysis.options.detection.plane.distanceThreshold;
        if(boundaryCoverage(local,checked,tolerance)+1e-6<boundaryCoverage(local,baseline,tolerance))return reject("Repair degraded local boundary coverage");
    }
    for(std::size_t i=0;i<baseline.curvedSolids.size();++i)if(checked.curvedSolids[i].meanResidual>baseline.curvedSolids[i].meanResidual+analysis.options.detection.plane.distanceThreshold*0.15)return reject("Repair increased local model error");
    const double directionScale=analysis.options.detection.plane.distanceThreshold/analysis.options.relations.angularToleranceRadians;
    const auto a=signature(baseline,directionScale),b=signature(checked,directionScale);
    if(a.size()!=b.size())return reject("Repair changed feature structure");
    const double budget=analysis.options.detection.plane.distanceThreshold*0.25;
    for(std::size_t i=0;i<a.size();++i)if(!std::isfinite(b[i]) || std::abs(a[i]-b[i])>budget)return reject("Repair moved previously recognized geometry");
    if(status.afterCoverage+1e-6<status.beforeCoverage || checked.surfaceCoverage<0.98)return reject("Repair degraded boundary coverage");
    merged.baseFingerprint=pointStoreFingerprint(points);merged.recoverySourceFingerprint=pointStoreFingerprint(source?*source:points);
    std::unordered_set<PointId> original;for(const auto& p:points.points())original.insert(p.id);
    for(const auto& p:combined.points())if(!original.count(p.id))merged.supplementalPoints.push_back(p);
    for(const auto& f:failures)if(f.repaired)for(const auto& patch:f.repairedPatches) {
        const auto mapping=std::find_if(mappings.begin(),mappings.end(),[&](const auto& m){return m.localFingerprint==f.localFingerprint && m.local==patch.surfaceId;});
        if(mapping==mappings.end())return reject("Missing local surface mapping");
        std::unordered_set<PointId> localIds(patch.supportPointIds.begin(),patch.supportPointIds.end());
        for(const auto& global:merged.patches)if(global.surfaceId==mapping->global && std::any_of(global.supportPointIds.begin(),global.supportPointIds.end(),[&](auto id){return localIds.count(id)>0;}))
            merged.repairPatchMappings.push_back({f.localFingerprint,patch.id,global.id});
    }
    merged.repairMappings=std::move(mappings);merged.localFailures=std::move(failures);merged.detectionPasses=analysis.detectionPasses;
    analysis=std::move(merged);output=std::move(checked);status.committed=true;if(report)*report=status;return true;
}
}
