#pragma once
#include <CGAL/Shape_detection/Efficient_RANSAC/Shape_base.h>
#include "common/Math3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace reconstruction::detail::cgal {
// Project-owned CGAL shape factory. Estimate the axis from the normal circle,
// which stays identifiable for a narrow axial band. Inferring it from three
// almost coplanar generatrices is ill-conditioned on countersink observations.
template<class Traits>
class StableCone final : public CGAL::Shape_detection::Shape_base<Traits> {
    using Point=typename Traits::Point_3;
    using Vector=typename Traits::Vector_3;
    using Vec=modeling::Vec3;
    Vec apex_{},axis_{0,0,1};double angle_=0,sine_=0,cosine_=1;
    static Vec vec(const Point& p){return {CGAL::to_double(p.x()),CGAL::to_double(p.y()),CGAL::to_double(p.z())};}
    static Vec vec(const Vector& p){return {CGAL::to_double(p.x()),CGAL::to_double(p.y()),CGAL::to_double(p.z())};}
    double distance(Vec p) const {
        const auto d=math3::subtract(p,apex_);const double t=math3::dot(d,axis_);
        const double r=math3::length(math3::subtract(d,math3::scale(axis_,t)));
        if(t*cosine_+r*sine_<0)return math3::dot(d,d);
        const double residual=r*cosine_-t*sine_;return residual*residual;
    }
  public:
    double angle() const{return angle_;}
    Point apex() const{return Point(apex_.x,apex_.y,apex_.z);}
    Vector axis() const{return Vector(axis_.x,axis_.y,axis_.z);}
    typename Traits::FT squared_distance(const Point& p) const override{return distance(vec(p));}
  protected:
    void create_shape(const std::vector<std::size_t>& indices) override {
        this->m_is_valid=false;
        std::array<Vec,3> p,n;
        for(int i=0;i<3;++i){p[i]=vec(this->point(indices[i]));n[i]=math3::normalized(vec(this->normal(indices[i])));}
        const double determinant=math3::dot(n[0],math3::cross(n[1],n[2]));
        if(std::abs(determinant)<1e-9)return;
        const double b=math3::dot(n[1],math3::subtract(p[1],p[0])),c=math3::dot(n[2],math3::subtract(p[2],p[0]));
        apex_=math3::add(p[0],math3::scale(math3::add(math3::scale(math3::cross(n[2],n[0]),b),
            math3::scale(math3::cross(n[0],n[1]),c)),1/determinant));
        const auto mean=math3::scale(math3::add(math3::add(p[0],p[1]),p[2]),1.0/3);
        double best=std::numeric_limits<double>::infinity();Vec bestAxis{};double bestAngle=0;
        for(int signs=0;signs<4;++signs) {
            auto a=math3::cross(math3::subtract(math3::scale(n[1],signs&1?-1:1),n[0]),
                math3::subtract(math3::scale(n[2],signs&2?-1:1),n[0]));
            if(math3::length(a)<1e-9)continue;
            a=math3::normalized(a);if(math3::dot(a,math3::subtract(mean,apex_))<0)a=math3::scale(a,-1);
            const double sine=std::clamp(std::abs(math3::dot(n[0],a)),0.0,1.0),angle=std::asin(sine);
            if(angle<1e-4 || angle>1.5706963267948966)continue;
            axis_=a;angle_=angle;sine_=sine;cosine_=std::cos(angle);
            double error=0;for(auto q:p)error+=distance(q);
            if(std::isfinite(error) && error<best){best=error;bestAxis=a;bestAngle=angle;}
        }
        if(best>this->m_epsilon*3)return;
        axis_=bestAxis;angle_=bestAngle;sine_=std::sin(angle_);cosine_=std::cos(angle_);
        this->m_is_valid=true;
    }
    void squared_distance(const std::vector<std::size_t>& indices,std::vector<typename Traits::FT>& values) const override {
        for(std::size_t i=0;i<indices.size();++i)values[i]=distance(vec(this->point(indices[i])));
    }
    void cos_to_normal(const std::vector<std::size_t>& indices,std::vector<typename Traits::FT>& values) const override {
        for(std::size_t i=0;i<indices.size();++i) {
            const auto d=math3::subtract(vec(this->point(indices[i])),apex_);
            const auto radial=math3::subtract(d,math3::scale(axis_,math3::dot(d,axis_)));
            const double r=math3::length(radial);
            values[i]=r<1e-12?0:std::abs(math3::dot(vec(this->normal(indices[i])),
                math3::subtract(math3::scale(radial,cosine_/r),math3::scale(axis_,sine_))));
        }
    }
    std::size_t minimum_sample_size() const override{return 3;}
};
}
