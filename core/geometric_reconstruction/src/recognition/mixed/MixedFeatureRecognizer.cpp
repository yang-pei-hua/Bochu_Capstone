#include "detail/MixedInternal.h"
#include "reconstruction/settings/ModuleConfig.h"

namespace reconstruction {
using namespace detail::mixed;

namespace detail::mixed {
bool recognizeLocalFeature(const PointStore& observed, const SurfaceAnalysisResult& candidates,
    const Frame& frame, double tolerance, bool subtractive,
    const MixedFeatureRecognitionOptions& options, const CompoundRecognitionOptions& compound,
    CompoundCandidate& output, std::string& error) {
    if(subtractive) {
        PrismCandidate cut;
        if(!rectangularCut(observed,candidates,frame,tolerance,options,cut)) {
            error="Unexplained host cavity; refusing incomplete mixed model";return false;
        }
        output.prisms.push_back(cut);return true;
    }
    CurvedSolidCandidate curved;
    if(attachedCurved(observed,candidates,frame,tolerance,options,curved)) {output.curvedSolids.push_back(curved);return true;}
    CompoundRecognitionOptions local = compound;
    local.distanceTolerance=tolerance;
    local.interfacePlanes.push_back({frame.z,-frame.c[1]});
    for(const auto& e:candidates.surfaces) if(auto plane=std::get_if<PlaneEvidence>(&e)) local.boundaryEvidence.push_back(*plane);
    CompoundCandidate planar;
    if(!reconstructCompound(observed,local,planar,error)) {
        error="Unsupported attached component: "+error;return false;
    }
    output.boxes.insert(output.boxes.end(),planar.boxes.begin(),planar.boxes.end());
    output.prisms.insert(output.prisms.end(),planar.prisms.begin(),planar.prisms.end());return true;
}
} // namespace detail::mixed

bool recognizeMixedFeatures(const PointStore& points,const SurfaceAnalysisResult& analysis,
                            CompoundCandidate& output,std::string& error,
                            std::vector<LocalAnalysisFailure>* failures,const PointStore* source,
                            const RecognitionSettings* settings) {
    output={};error.clear();
    if(failures)failures->clear();
    if(!analysisMatches(points,analysis)) {error="Stale mixed-feature analysis";return false;}
    const MixedFeatureRecognitionOptions& options = settings
        ? settings->mixed : MixedFeatureRecognitionOptions{};
    const CompoundRecognitionOptions compound = settings
        ? settings->compound : CompoundRecognitionOptions{};
    const ThroughHoleRecognitionOptions holeOptions = settings
        ? settings->throughHole : ThroughHoleRecognitionOptions{};
    const double tolerance=analysis.options.detection.plane.distanceThreshold;
    Frame frame;BoxCandidate host;
    if(!hostFrame(points,analysis,frame,host,options)) {error="No observed thin rectangular host";return false;}
    output.boxes.push_back(host);
    std::vector<CylinderEvidence> holes;
    std::unordered_map<PointId,const PointSample*> byId;
    for(const auto& p:points.points()) byId[p.id]=&p;
    for(const auto& e:analysis.surfaces) if(auto c=std::get_if<CylinderEvidence>(&e)) {
        double inward=0;std::size_t count=0;
        for(auto id:c->supportPointIds) {
            auto p=byId.at(id);if(!p->normal) continue;
            auto d=subtract(p->position,c->cylinder.axisPoint);auto axis=c->cylinder.axisDirection;
            auto radial=subtract(d,scale(axis,dot(d,axis)));
            if(length(radial)>1e-12) {inward+=dot(normalized(*p->normal),normalized(radial));++count;}
        }
        if(count && inward/count<-options.holeInwardNormalFraction) holes.push_back(*c);
    }
    if(!recognizeThroughHoles(host,holes,holeOptions,output.throughHoles,error)) return false;
    const double knownTolerance=tolerance*options.knownSurfaceToleranceMultiplier;
    std::vector<PointId> above,unknown;
    for(const auto& p:points.points()) {
        const double t=dot(p.position,frame.z);
        const bool sideAbove=t>frame.c[1]+tolerance*options.aboveSideToleranceFraction && p.normal &&
            std::abs(dot(normalized(*p.normal),frame.z))<options.aboveSideNormalCosine;
        if(t>frame.c[1]+knownTolerance || sideAbove) {above.push_back(p.id);continue;}
        if(t<frame.c[0]-knownTolerance || t>frame.c[1]+knownTolerance) continue;
        bool known=false;
        for(const auto& e:host.sourceSurfaces) if(surfaceResidual(e,p.position)<knownTolerance &&
            p.normal && std::abs(dot(normalized(*p.normal),e.plane.normal))>options.knownSurfaceNormalCosine) known=true;
        for(const auto& c:holes) if(surfaceResidual(c,p.position)<knownTolerance) known=true;
        if(!known) unknown.push_back(p.id);
    }
    const double spacing=std::sqrt(2*host.primitive.sizeX*host.primitive.sizeY/std::max<std::size_t>(1,points.size()));
    const double connectivity=std::max(tolerance*options.connectivityToleranceMultiplier,
        spacing*options.connectivitySpacingMultiplier);
    std::unordered_set<PointId> recoveredMembership;
    auto process=[&](const std::vector<PointId>& ids,bool subtractive) {
        for(const auto& group:connectedPointComponents(points,ids,connectivity,
                connectivity*options.connectivityAdaptiveMultiplier)) {
            if(std::all_of(group.begin(),group.end(),[&](auto id){return recoveredMembership.count(id)>0;}))continue;
            std::vector<PointSample> samples;for(auto id:group) samples.push_back(*byId.at(id));
            PointStore local(std::move(samples),points.unit());
            SurfaceAnalysisResult restricted;
            if(!indexSurfaceEvidence(local,restrictEvidence(analysis,group,options),analysis.options,restricted,error)) return false;
            auto attempt = [&](const PointStore& observed, const SurfaceAnalysisResult& candidates) {
                return recognizeLocalFeature(observed, candidates, frame, tolerance, subtractive,
                                             options, compound, output, error);
            };
            if(group.size()>=options.minimumLocalPoints && attempt(local,restricted)) continue;
            if(group.size()<options.minimumLocalPoints)error="Insufficient local observations";
            LocalAnalysisFailure failure;failure.pointIds=group;failure.message=error;
            failure.reason=(group.size()<options.minimumLocalPoints || restricted.surfaces.empty())
                ?LocalFailureReason::InsufficientSupport:LocalFailureReason::CandidateConflict;
            const bool axial=std::any_of(restricted.surfaces.begin(),restricted.surfaces.end(),[](const auto& e){return primitiveKind(e)==PrimitiveKind::Cylinder || primitiveKind(e)==PrimitiveKind::Cone;});
            if(axial) {
                std::size_t cap=0;for(const auto& p:local.points())cap+=p.normal && dot(normalized(*p.normal),frame.z)>options.capNormalCosine;
                if(cap<options.minimumCapPoints)failure.reason=LocalFailureReason::MissingEndFace;
                else if(std::none_of(restricted.relations.begin(),restricted.relations.end(),[](const auto& r){return r.kind==SurfaceRelationKind::AxisNormalToPlane;}))failure.reason=LocalFailureReason::InvalidRelation;
            }
            PointStore recovered({},points.unit());SurfaceAnalysisResult repaired;
            const bool detected=!analysis.baseFingerprint && recoverLocalSurfaces(points,source?*source:points,analysis,failure,recovered,repaired,error);
            if(detected && recovered.size()>=options.minimumLocalPoints) failure.repaired=attempt(recovered,repaired);
            if(!failure.repaired) error=failure.message+"; local recovery rejected: "+error;
            const bool success=failure.repaired;
            if(success) {recoveredMembership.insert(group.begin(),group.end());recoveredMembership.insert(failure.recoveredPointIds.begin(),failure.recoveredPointIds.end());}
            if(failures)failures->push_back(std::move(failure));
            // Tiny disconnected remnants retain diagnostics; they cannot alone
            // establish a feature. Substantial failed regions still reject the model.
            if(!success && group.size()>=options.minimumLocalPoints)return false;
        }
        return true;
    };
    if(!process(above,false) || !process(unknown,true)) {output={};return false;}
    std::size_t supported=0;
    for(const auto& p:points.points()) {
        if(!p.normal || !p.normalOrientationKnown) {error="Mixed features require outward normals";output={};return false;}
        const auto shift=scale(normalized(*p.normal),tolerance*options.coverageShiftToleranceMultiplier);
        supported+=material(output,subtract(p.position,shift)) && !material(output,add(p.position,shift));
    }
    output.surfaceCoverage=double(supported)/points.size();
    if(output.surfaceCoverage<options.minimumSurfaceCoverage) {
        error="Mixed boundary agreement below "+std::to_string(options.minimumSurfaceCoverage)+": "
            +std::to_string(output.surfaceCoverage);
        output={};return false;
    }
    output.confidence=output.surfaceCoverage;output.minimumBoxCountProven=false;error.clear();
    return true;
}
} // namespace reconstruction
