#include "detail/AnalysisInternal.h"
#include <Eigen/Dense>
#include <set>
#include <tuple>

namespace reconstruction {
using namespace detail::analysis;
namespace {
struct Constraint {std::size_t a,b; SurfaceRelation relation;};
bool supported(const PrimitiveEvidence& e) {
    return std::holds_alternative<PlaneEvidence>(e) || std::holds_alternative<CylinderEvidence>(e);
}
// Parameters are local displacements about the observed support centroid. This
// avoids conditioning the fit on the distance of the part from the world origin.
std::vector<PrimitiveEvidence> decode(const Eigen::VectorXd& x,
    const std::vector<PrimitiveEvidence>& original,const std::vector<Vec3>& origins) {
    auto result=original;
    for(std::size_t i=0;i<result.size();++i) {
        const auto k=Eigen::Index(i*7);Vec3 a{x[k],x[k+1],x[k+2]};
        if(length(a)<1e-8) continue;
        a=normalized(a);
        if(auto p=std::get_if<PlaneEvidence>(&result[i])) p->plane={a,x[k+3]-dot(a,origins[i])};
        if(auto c=std::get_if<CylinderEvidence>(&result[i])) {
            auto center=add(origins[i],Vec3{x[k+3],x[k+4],x[k+5]});
            c->cylinder={subtract(center,scale(a,dot(center,a))),a,x[k+6]};
        }
    }
    return result;
}
}
bool optimizeSurfaceRelations(const PointStore& points, SurfaceAnalysisResult& analysis,
    const std::vector<SurfaceRelation>& proposals, RelationOptimizationReport& report,std::string& error) {
    report={};error.clear();
    if(!analysisMatches(points,analysis)) {error="Stale optimization snapshot";return false;}
    const auto map=pointMap(points);
    const auto original=analysis.surfaces;
    std::vector<Vec3> origins(original.size());
    for(std::size_t i=0;i<original.size();++i) {
        for(auto id:ids(original[i])) origins[i]=add(origins[i],map.at(id)->position);
        if(!ids(original[i]).empty()) origins[i]=scale(origins[i],1.0/ids(original[i]).size());
    }
    auto equations=original;
    for(auto& e:equations)std::visit([](auto& s){s.supportPointIds.clear();},e);
    std::vector<Constraint> accepted;
    std::set<std::tuple<std::size_t,std::size_t,SurfaceRelationKind>> seen;
    std::size_t attempts=0;
    auto fitted=original;
    const double tolerance=analysis.options.detection.plane.distanceThreshold;
    const double angleTolerance=analysis.options.relations.angularToleranceRadians;
    auto meanError=[&](const PrimitiveEvidence& e) {
        double sum=0;for(auto id:ids(e))sum+=surfaceResidual(e,map.at(id)->position);
        return sum/std::max<std::size_t>(1,ids(e).size());
    };
    for(const auto& e:original)report.beforeError+=meanError(e);
    auto sorted=proposals;
    std::stable_sort(sorted.begin(),sorted.end(),[](const auto& a,const auto& b){return a.confidence>b.confidence;});
    for(const auto& r:sorted) {
        if(r.kind!=SurfaceRelationKind::Parallel && r.kind!=SurfaceRelationKind::Perpendicular && r.kind!=SurfaceRelationKind::Coaxial) continue;
        if(!r.first || !r.second || r.first>analysis.patches.size() || r.second>analysis.patches.size()) {report.rejected.push_back(r);continue;}
        const auto a=std::size_t(analysis.patches[r.first-1].surfaceId-1),b=std::size_t(analysis.patches[r.second-1].surfaceId-1);
        if(a>=original.size() || b>=original.size() || a==b || !supported(original[a]) || !supported(original[b]) ||
           (r.kind==SurfaceRelationKind::Coaxial && (!std::holds_alternative<CylinderEvidence>(original[a]) || !std::holds_alternative<CylinderEvidence>(original[b])))) {
            report.rejected.push_back(r);continue;
        }
        if(!seen.emplace(std::min(a,b),std::max(a,b),r.kind).second)continue;
        if(++attempts>32) {report.rejected.push_back(r);continue;}
        auto constraints=accepted;constraints.push_back({a,b,r});
        std::unordered_set<std::size_t> active;
        for(const auto& c:constraints){active.insert(c.a);active.insert(c.b);}
        if(active.size()>12) {report.rejected.push_back(r);continue;}
        Eigen::VectorXd initial=Eigen::VectorXd::Zero(Eigen::Index(original.size()*7));
        for(auto i:active) {
            auto k=Eigen::Index(i*7);const auto u=axis(fitted[i]);initial[k]=u.x;initial[k+1]=u.y;initial[k+2]=u.z;
            if(const auto* p=std::get_if<PlaneEvidence>(&fitted[i])) initial[k+3]=dot(u,origins[i])+p->plane.offset;
            else {const auto& c=std::get<CylinderEvidence>(fitted[i]);auto d=subtract(c.cylinder.axisPoint,origins[i]);d=subtract(d,scale(u,dot(d,u)));initial[k+3]=d.x;initial[k+4]=d.y;initial[k+5]=d.z;initial[k+6]=c.cylinder.radius;}
        }
        std::vector<Eigen::Index> variables;
        for(auto i:active)for(int j=0;j<(std::holds_alternative<PlaneEvidence>(original[i])?4:7);++j)variables.push_back(Eigen::Index(i*7+j));
        auto residual=[&](const Eigen::VectorXd& x) {
            const auto decoded=decode(x,equations,origins);std::vector<double> values;
            for(auto i:active) {
                const auto& support=ids(original[i]);const std::size_t stride=std::max<std::size_t>(1,(support.size()+255)/256);
                const double weight=1/std::sqrt(double((support.size()+stride-1)/stride));
                for(std::size_t j=0;j<support.size();j+=stride)values.push_back(surfaceResidual(decoded[i],map.at(support[j])->position)/tolerance*weight);
                // Retain the observed direction and remove parameter scale nullspace.
                const auto u=axis(decoded[i]),v=axis(original[i]);
                values.push_back((1-std::abs(dot(u,v)))/angleTolerance);
                const auto k=Eigen::Index(i*7);values.push_back((std::sqrt(x[k]*x[k]+x[k+1]*x[k+1]+x[k+2]*x[k+2])-1));
            }
            for(const auto& c:constraints) {
                const auto u=axis(decoded[c.a]),v=axis(decoded[c.b]);
                const auto d=c.relation.kind==SurfaceRelationKind::Perpendicular ? Vec3{dot(u,v),0,0} : cross(u,v);
                values.push_back(d.x/angleTolerance*5);values.push_back(d.y/angleTolerance*5);values.push_back(d.z/angleTolerance*5);
                if(c.relation.kind==SurfaceRelationKind::Coaxial) {
                    auto delta=subtract(center(decoded[c.a]),center(decoded[c.b]));delta=subtract(delta,scale(u,dot(delta,u)));
                    values.push_back(delta.x/tolerance*5);values.push_back(delta.y/tolerance*5);values.push_back(delta.z/tolerance*5);
                }
            }
            return Eigen::Map<Eigen::VectorXd>(values.data(),Eigen::Index(values.size())).eval();
        };
        auto x=initial;double damping=1e-3;
        for(int iteration=0;iteration<15;++iteration) {
            const auto base=residual(x);Eigen::MatrixXd jac(base.size(),Eigen::Index(variables.size()));
            for(std::size_t j=0;j<variables.size();++j) {
                auto shifted=x;const double step=1e-6*std::max(1.0,std::abs(x[variables[j]]));shifted[variables[j]]+=step;
                jac.col(Eigen::Index(j))=(residual(shifted)-base)/step;
            }
            Eigen::MatrixXd h=jac.transpose()*jac;h.diagonal().array()+=damping;
            const Eigen::VectorXd delta=h.ldlt().solve(-jac.transpose()*base);
            if(!delta.allFinite())break;
            auto trial=x;for(std::size_t j=0;j<variables.size();++j)trial[variables[j]]+=delta[Eigen::Index(j)];
            if(residual(trial).squaredNorm()<base.squaredNorm()) {x=trial;damping*=0.5;if(delta.norm()<1e-7)break;}
            else damping*=10;
        }
        auto trial=decode(x,original,origins);bool valid=true;
        for(auto i:active) {
            const auto& e=trial[i];std::size_t inliers=0;
            for(auto id:ids(e))inliers+=surfaceResidual(e,map.at(id)->position)<=tolerance*2;
            valid &= inliers>=ids(e).size()*0.9 && meanError(e)<=meanError(original[i])+tolerance*0.15 &&
                std::abs(dot(axis(e),axis(original[i])))>=std::cos(angleTolerance);
            if(const auto* c=std::get_if<CylinderEvidence>(&e))valid &= c->cylinder.radius>0;
        }
        for(const auto& c:constraints) {
            const double alignment=std::abs(dot(axis(trial[c.a]),axis(trial[c.b])));
            valid &= c.relation.kind==SurfaceRelationKind::Perpendicular ? alignment<=std::sin(angleTolerance)*0.25 : alignment>=std::cos(angleTolerance*0.25);
            if(c.relation.kind==SurfaceRelationKind::Coaxial) {
                auto d=subtract(center(trial[c.a]),center(trial[c.b]));auto u=axis(trial[c.a]);
                valid &= length(subtract(d,scale(u,dot(d,u))))<=analysis.options.relations.distanceTolerance*0.25;
            }
        }
        if(valid) {fitted=std::move(trial);accepted=std::move(constraints);report.accepted.push_back(r);}
        else report.rejected.push_back(r);
    }
    std::unordered_set<std::size_t> adjusted;
    for(const auto& c:accepted){adjusted.insert(c.a);adjusted.insert(c.b);}
    for(auto i:adjusted) {
        Vec3 lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
        for(auto id:ids(original[i])) {const auto p=map.at(id)->position;
            lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
            hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
        }
        std::vector<PointId> support;
        for(const auto& p:points.points()) {
            const auto q=p.position;
            if(q.x<lo.x-tolerance || q.x>hi.x+tolerance || q.y<lo.y-tolerance || q.y>hi.y+tolerance || q.z<lo.z-tolerance || q.z>hi.z+tolerance)continue;
            if(!p.normal || surfaceResidual(fitted[i],q)>tolerance*2)continue;
            auto n=axis(fitted[i]);
            if(std::holds_alternative<CylinderEvidence>(fitted[i])) {const auto d=subtract(q,center(fitted[i]));n=normalized(subtract(d,scale(n,dot(d,n))));}
            if(std::abs(dot(normalized(*p.normal),n))>=0.9)support.push_back(p.id);
        }
        double reassignedError=0;for(auto id:support)reassignedError+=surfaceResidual(fitted[i],map.at(id)->position);
        reassignedError/=std::max<std::size_t>(1,support.size());
        // Reassignment must meet the same observation budget as the fit.
        // Keep the already validated support if normals are unavailable.
        if(support.size()>=ids(original[i]).size()*0.9 && reassignedError<=meanError(original[i])+tolerance*0.15)std::visit([&](auto& e){e.supportPointIds=std::move(support);e.coverage=double(e.supportPointIds.size())/points.size();},fitted[i]);
        if(auto c=std::get_if<CylinderEvidence>(&fitted[i])) {
            c->axialMinimum=INFINITY;c->axialMaximum=-INFINITY;
            for(auto id:c->supportPointIds) {const double t=dot(subtract(map.at(id)->position,c->cylinder.axisPoint),c->cylinder.axisDirection);c->axialMinimum=std::min(c->axialMinimum,t);c->axialMaximum=std::max(c->axialMaximum,t);}
        }
    }
    for(auto& e:fitted) {
        const auto mean=meanError(e);report.afterError+=mean;
        std::visit([&](auto& s){s.meanAbsoluteResidual=mean;s.maxAbsoluteResidual=0;
            for(auto id:s.supportPointIds)s.maxAbsoluteResidual=std::max(s.maxAbsoluteResidual,surfaceResidual(e,map.at(id)->position));},e);
    }
    SurfaceAnalysisResult indexed;
    if(!indexSurfaceEvidence(points,fitted,analysis.options,indexed,error))return false;
    indexed.detectionPasses=analysis.detectionPasses;indexed.localFailures=analysis.localFailures;
    std::vector<SurfaceRelationHint> manual;
    for(const auto& r:analysis.relations)if(!r.automatic && r.first && r.second && r.first<=analysis.patches.size() && r.second<=analysis.patches.size())
        manual.push_back({analysis.patches[r.first-1].supportPointIds,analysis.patches[r.second-1].supportPointIds,r.kind});
    if(!applySurfaceRelationHints(points,manual,indexed,error))return false;
    analysis=std::move(indexed);return true;
}
} // namespace reconstruction
