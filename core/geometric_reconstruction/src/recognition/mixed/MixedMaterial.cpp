#include "detail/MixedInternal.h"

namespace reconstruction::detail::mixed {
double median(std::vector<double> values) {
    if(values.empty()) return NAN;
    auto at=values.begin()+values.size()/2; std::nth_element(values.begin(),at,values.end()); return *at;
}

std::vector<PrimitiveEvidence> restrictEvidence(const SurfaceAnalysisResult& a,const std::vector<PointId>& group) {
    std::unordered_set<PointId> ids(group.begin(),group.end());
    std::vector<PrimitiveEvidence> result;
    for(auto e : a.surfaces) {
        bool keep=std::visit([&](auto& s){
            auto& support=s.supportPointIds;
            support.erase(std::remove_if(support.begin(),support.end(),[&](auto id){return !ids.count(id);}),support.end());
            return support.size()>=12;
        },e);
        if(keep) result.push_back(std::move(e));
    }
    return result;
}

bool inBox(const modeling::BoxPrimitiveParams& b,Vec3 p) {
    const auto x=normalized(b.pose.xDirection),z=normalized(b.pose.zDirection),y=cross(z,x);
    const auto d=subtract(p,b.pose.origin);
    return dot(d,x)>=0 && dot(d,x)<=b.sizeX && dot(d,y)>=0 && dot(d,y)<=b.sizeY && dot(d,z)>=0 && dot(d,z)<=b.sizeZ;
}

bool inPrism(const PrismCandidate& prism,Vec3 p) {
    const auto n=normalized(prism.plane.normal), x=normalized(prism.plane.xDirection),y=cross(n,x);
    const auto d=subtract(p,prism.plane.origin); const double t=dot(d,n),u=dot(d,x),v=dot(d,y);
    if(t<0 || t>prism.depth) return false;
    bool inside=false;
    for(std::size_t i=0,j=prism.polygon.size()-1;i<prism.polygon.size();j=i++) {
        const auto a=prism.polygon[i],b=prism.polygon[j];
        if((a.y>v)!=(b.y>v) && u<(b.x-a.x)*(v-a.y)/(b.y-a.y)+a.x) inside=!inside;
    }
    return inside;
}

bool material(const CompoundCandidate& model,Vec3 p) {
    bool inside=false;
    for(const auto& b:model.boxes) inside |= inBox(b.primitive,p);
    for(const auto& prism:model.prisms) if(!prism.subtract) inside |= inPrism(prism,p);
    for(const auto& c:model.curvedSolids) inside |= std::visit([&](const auto& s) {
        using T=std::decay_t<decltype(s)>;
        if constexpr(std::is_same_v<T,modeling::CylinderPrimitiveParams> || std::is_same_v<T,modeling::ConePrimitiveParams>) {
            const auto axis=normalized(s.axisDirection),d=subtract(p,s.baseCenter);
            const double t=dot(d,axis); if(t<0 || t>s.height) return false;
            const double radius=[&](){if constexpr(std::is_same_v<T,modeling::CylinderPrimitiveParams>) return s.radius;
                else return s.baseRadius+(s.topRadius-s.baseRadius)*t/s.height;}();
            const auto radial=subtract(d,scale(axis,t)); return dot(radial,radial)<=radius*radius;
        } else return false;
    },c.primitive);
    if(!inside) return false;
    for(const auto& h:model.throughHoles) {
        const auto axis=normalized(h.primitive.axisDirection),d=subtract(p,h.primitive.axisPoint);
        const auto radial=subtract(d,scale(axis,dot(d,axis)));
        if(dot(radial,radial)<h.primitive.radius*h.primitive.radius) return false;
    }
    for(const auto& prism:model.prisms) if(prism.subtract && inPrism(prism,p)) return false;
    return true;
}
} // namespace reconstruction::detail::mixed
