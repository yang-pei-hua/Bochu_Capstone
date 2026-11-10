#include "detail/MixedInternal.h"

namespace reconstruction::detail::mixed {
bool attachedCurved(const PointStore& local,const SurfaceAnalysisResult& a,const Frame& host,
                    double tolerance,const MixedFeatureRecognitionOptions& options,CurvedSolidCandidate& output) {
    const double top=host.c[1]; double best=-INFINITY;
    bool found=false;
    for(const auto& e:a.surfaces) {
        const auto kind=primitiveKind(e);
        if(kind!=PrimitiveKind::Cylinder && kind!=PrimitiveKind::Cone) continue;
        const auto id=std::visit([](const auto& s){return s.id;},e);
        bool capRelation=false;
        for(const auto& other:a.surfaces) if(const auto* plane=std::get_if<PlaneEvidence>(&other))
            capRelation |= hasSurfaceRelation(a,id,plane->id,SurfaceRelationKind::AxisNormalToPlane);
        if(!capRelation) continue;
        Vec3 origin,axis; double radius,slope;
        if(auto c=std::get_if<CylinderEvidence>(&e)) {origin=c->cylinder.axisPoint;axis=c->cylinder.axisDirection;radius=c->cylinder.radius;slope=0;}
        else {const auto& cone=std::get<ConeEvidence>(e);origin=cone.cone.apex;axis=cone.cone.axisDirection;radius=0;slope=std::tan(cone.cone.openingAngleRadians);}
        if(std::abs(dot(axis,host.z))<options.bossAxisCosine) continue;
        if(dot(axis,host.z)<0) slope=-slope;
        axis=host.z;
        const double band=tolerance*options.bossBandToleranceMultiplier;
        const double start=(top-dot(origin,host.z))/dot(axis,host.z);
        const auto base=add(origin,scale(axis,start)); const double r0=radius+slope*start;
        if(r0<=tolerance*options.bossRadiusToleranceMultiplier) continue;
        std::vector<double> capHeights;
        for(const auto& p:local.points()) {
            if(!p.normal || dot(normalized(*p.normal),axis)<options.bossCapNormalCosine) continue;
            const auto d=subtract(p.position,base); const double h=dot(d,axis);
            const auto radial=subtract(d,scale(axis,h));
            if(length(radial)<=r0+slope*h+tolerance*options.bossRadiusToleranceMultiplier) capHeights.push_back(h);
        }
        if(capHeights.size()<options.bossMinimumCapPoints) continue;
        const double h=median(capHeights),r1=r0+slope*h;
        if(h<tolerance*options.bossMinimumHeightMultiplier ||
           r1<=tolerance*options.bossRadiusToleranceMultiplier) continue;
        const auto u=normalized(cross(axis,std::abs(axis.z)<options.bossReferenceAxisCosine?Vec3{0,0,1}:Vec3{1,0,0})),v=cross(axis,u);
        std::set<int> sideBins,capBins;
        std::size_t supported=0;
        double residualSum=0;
        const int depthBins=static_cast<int>(options.bossDepthBins);
        const int angularBins=static_cast<int>(options.bossAngularBins);
        const int capSectors=static_cast<int>(options.bossCapSectors);
        const int radialBins=static_cast<int>(options.bossRadialBins);
        const int angularPerSector=std::max(1,angularBins/std::max(1,capSectors));
        for(const auto& p:local.points()) {
            const auto d=subtract(p.position,base);const double t=dot(d,axis);
            const auto radial=subtract(d,scale(axis,t));const double rho=length(radial);
            if(!p.normal || rho<1e-12) continue;
            double angle=std::atan2(dot(radial,v),dot(radial,u));if(angle<0) angle+=6.283185307179586;
            const int bin=std::min(angularBins-1,int(angle/6.283185307179586*angularBins));
            bool match=false;
            double residual=INFINITY;
            if(t>=-band && t<=h+band &&
                std::abs(rho-(r0+slope*t))/std::sqrt(1+slope*slope)<=band &&
                dot(normalized(*p.normal),normalized(subtract(scale(radial,1/rho),scale(axis,slope))))>options.bossSideNormalCosine) {
                sideBins.insert(std::clamp(int(t/h*depthBins),0,depthBins-1)*angularBins+bin);match=true;
                residual=std::abs(rho-(r0+slope*t))/std::sqrt(1+slope*slope);
            }
            if(std::abs(t-h)<=band && rho<=r1+band && dot(normalized(*p.normal),axis)>options.bossCapNormalCosine) {
                capBins.insert(bin/angularPerSector+
                    capSectors*std::clamp(int(rho/r1*radialBins),0,radialBins-1));match=true;
                residual=std::min(residual,std::abs(t-h));
            }
            supported+=match;
            if(match) residualSum+=residual;
        }
        const double coverage=double(supported)/local.size();
        const double boundary=std::min(sideBins.size()/double(depthBins*angularBins),
                                       capBins.size()/double(capSectors*radialBins));
        if(coverage<options.minimumBossCoverage || boundary<options.minimumBossBoundary ||
           coverage+boundary<=best) continue;
        best=coverage+boundary;found=true;
        if(kind==PrimitiveKind::Cylinder) output.primitive=modeling::CylinderPrimitiveParams{base,axis,r0,h,true};
        else output.primitive=modeling::ConePrimitiveParams{base,axis,r0,r1,h,true};
        output.surfaceCoverage=coverage;output.boundaryCoverage=boundary;
        output.meanResidual=residualSum/supported;
    }
    return found;
}

bool rectangularCut(const PointStore& local,const SurfaceAnalysisResult& a,const Frame& host,
                    double tolerance,const MixedFeatureRecognitionOptions& options,PrismCandidate& cut) {
    std::vector<PlaneEvidence> walls;
    for(const auto& e:a.surfaces) if(auto p=std::get_if<PlaneEvidence>(&e))
        if(std::abs(dot(p->plane.normal,host.z))<options.coplanarNormalCosine) {
            auto equivalent=std::find_if(walls.begin(),walls.end(),[&](const auto& old) {
                const double alignment=dot(old.plane.normal,p->plane.normal);
                return std::abs(alignment)>options.coplanarAxisCosine &&
                    std::abs(old.plane.offset-(alignment<0?-p->plane.offset:p->plane.offset))
                        <tolerance*options.cutWallToleranceMultiplier;
            });
            if(equivalent==walls.end())walls.push_back(*p);
            else if(p->supportPointIds.size()>equivalent->supportPointIds.size() ||
                    (p->supportPointIds.size()==equivalent->supportPointIds.size() && p->meanAbsoluteResidual<equivalent->meanAbsoluteResidual))*equivalent=*p;
        }
    if(walls.size()<options.minimumWalls)return false;
    std::stable_sort(walls.begin(),walls.end(),[](const auto& a,const auto& b){return a.supportPointIds.size()>b.supportPointIds.size();});
    // Competing equations remain in the analysis. Evaluate bounded four-wall
    // hypotheses against every observed cavity point before choosing one.
    if(walls.size()>options.maximumWalls)walls.resize(options.maximumWalls);
    auto testWalls=[&](const std::vector<PlaneEvidence>& selected) {
    // Observed wall normals may tilt with noise. The cut frame must remain
    // exactly orthogonal to the host axis so its bottom and top coincide with
    // the observed host planes, independently of world-space XY coordinates.
    const auto wallNormal=selected.front().plane.normal;
    const auto x=normalized(subtract(wallNormal,scale(host.z,dot(wallNormal,host.z)))),y=normalized(cross(host.z,x));
    std::array<std::vector<double>,2> levels;
    for(const auto& p:selected) {
        const double dx=dot(p.plane.normal,x),dy=dot(p.plane.normal,y);
        const double offset=p.plane.offset+dot(p.plane.normal,host.z)*(host.c[0]+host.c[1])*0.5;
        if(std::abs(dx)>options.cutLevelAxisCosine) levels[0].push_back(-offset/dx);
        else if(std::abs(dy)>options.cutLevelAxisCosine) levels[1].push_back(-offset/dy);
        else return false;
    }
    if(levels[0].size()!=2 || levels[1].size()!=2) return false;
    for(auto& v:levels) std::sort(v.begin(),v.end());
    const double w=levels[0][1]-levels[0][0],h=levels[1][1]-levels[1][0];
    if(w<tolerance*options.cutMinimumSizeMultiplier ||
       h<tolerance*options.cutMinimumSizeMultiplier) return false;
    const auto origin=add(add(scale(x,levels[0][0]),scale(y,levels[1][0])),scale(host.z,host.c[0]));
    std::array<bool,4> seen{};
    double low=INFINITY,high=-INFINITY;
    const double band=tolerance*options.cutWallToleranceMultiplier;
    for(const auto& p:local.points()) {
        if(!p.normal) return false;
        const auto d=subtract(p.position,origin);const double u=dot(d,x),v=dot(d,y),z=dot(p.position,host.z);
        low=std::min(low,z);high=std::max(high,z);
        if(std::abs(u)<band && dot(*p.normal,x)>options.cutSideNormalCosine) seen[0]=true;
        else if(std::abs(u-w)<band && dot(*p.normal,x)<-options.cutSideNormalCosine) seen[1]=true;
        else if(std::abs(v)<band && dot(*p.normal,y)>options.cutSideNormalCosine) seen[2]=true;
        else if(std::abs(v-h)<band && dot(*p.normal,y)<-options.cutSideNormalCosine) seen[3]=true;
        else return false;
    }
    if(!std::all_of(seen.begin(),seen.end(),[](bool b){return b;}) ||
       low>host.c[0]+tolerance*options.cutSpanToleranceMultiplier ||
       high<host.c[1]-tolerance*options.cutSpanToleranceMultiplier) return false;
    cut.plane={origin,host.z,x};cut.polygon={{0,0},{w,0},{w,h},{0,h}};
    cut.depth=host.c[1]-host.c[0];cut.subtract=true;
    return true;
    };
    for(std::size_t i=0;i<walls.size();++i)for(std::size_t j=i+1;j<walls.size();++j)
        for(std::size_t k=j+1;k<walls.size();++k)for(std::size_t l=k+1;l<walls.size();++l)
            if(testWalls({walls[i],walls[j],walls[k],walls[l]}))return true;
    return false;
}
} // namespace reconstruction::detail::mixed
