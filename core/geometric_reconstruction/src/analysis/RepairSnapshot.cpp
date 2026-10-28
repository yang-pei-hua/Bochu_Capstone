#include "detail/AnalysisInternal.h"
namespace reconstruction {
bool analysisInputMatches(const PointStore& points,const SurfaceAnalysisResult& analysis) noexcept {
    return points.unit()==analysis.unit && (analysis.baseFingerprint ? analysis.baseFingerprint==pointStoreFingerprint(points) : analysisMatches(points,analysis));
}
bool materializeAnalysisPoints(const PointStore& base,const SurfaceAnalysisResult& analysis,PointStore& output,std::string& error) {
    error.clear();
    if(!analysisInputMatches(base,analysis)){error="Stale repair input";return false;}
    auto samples=base.points();std::unordered_set<PointId> seen;
    for(const auto& p:samples)seen.insert(p.id);
    for(const auto& p:analysis.supplementalPoints) {
        if(!seen.insert(p.id).second){error="Duplicate supplemental point identity";return false;}
        samples.push_back(p);
    }
    PointStore combined(std::move(samples),base.unit());
    if(!analysisMatches(combined,analysis)){error="Corrupt augmented analysis snapshot";return false;}
    output=std::move(combined);return true;
}
}
