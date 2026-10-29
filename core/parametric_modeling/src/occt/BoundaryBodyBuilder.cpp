#include "modeling/BoundaryBody.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Builder.hxx>
#include <BRepLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <map>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace modeling {
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
template<class T,class A> A array(const std::vector<T>& values){A result(1,int(values.size()));for(int i=1;i<=int(values.size());++i)result.SetValue(i,values[i-1]);return result;}
Handle(Geom_Surface) surface(const BoundarySurface& s) {
    const auto o=s.pose.origin,x=s.pose.xDirection,z=s.pose.zDirection;
    const gp_Ax3 frame(gp_Pnt(o.x,o.y,o.z),gp_Dir(z.x,z.y,z.z),gp_Dir(x.x,x.y,x.z));
    switch(s.kind) {
    case BoundarySurface::Kind::Plane:return new Geom_Plane(frame);
    case BoundarySurface::Kind::Cylinder:return new Geom_CylindricalSurface(frame,s.radius);
    case BoundarySurface::Kind::Cone:return new Geom_ConicalSurface(frame,s.semiAngle,s.radius);
    case BoundarySurface::Kind::Sphere:return new Geom_SphericalSurface(frame,s.radius);
    case BoundarySurface::Kind::Torus:return new Geom_ToroidalSurface(frame,s.radius,s.minorRadius);
    case BoundarySurface::Kind::BSpline: {
        TColgp_Array2OfPnt poles(1,s.uCount,1,s.vCount);TColStd_Array2OfReal weights(1,s.uCount,1,s.vCount);
        for(int i=1;i<=s.uCount;++i)for(int j=1;j<=s.vCount;++j){const auto index=(i-1)*s.vCount+j-1;const auto p=s.poles[index];poles.SetValue(i,j,gp_Pnt(p.x,p.y,p.z));weights.SetValue(i,j,s.weights[index]);}
        return new Geom_BSplineSurface(poles,weights,array<double,TColStd_Array1OfReal>(s.uKnots),array<double,TColStd_Array1OfReal>(s.vKnots),
            array<int,TColStd_Array1OfInteger>(s.uMultiplicities),array<int,TColStd_Array1OfInteger>(s.vMultiplicities),s.uDegree,s.vDegree);
    }
    }
    throw std::runtime_error("Unknown boundary surface");
}
Handle(Geom2d_BSplineCurve) curve(const SplineCurve2D& c) {
    TColgp_Array1OfPnt2d poles(1,int(c.poles.size()));for(int i=1;i<=int(c.poles.size());++i)poles.SetValue(i,gp_Pnt2d(c.poles[i-1].x,c.poles[i-1].y));
    return new Geom2d_BSplineCurve(poles,array<double,TColStd_Array1OfReal>(c.weights),array<double,TColStd_Array1OfReal>(c.knots),array<int,TColStd_Array1OfInteger>(c.multiplicities),c.degree);
}
bool finite(Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
void spline(int degree,int count,const std::vector<double>& knots,const std::vector<int>& mult) {
    require(degree>=1 && degree<=25 && count>degree && knots.size()>=2 && knots.size()==mult.size(),"Invalid spline dimensions");
    int sum=0;for(size_t i=0;i<knots.size();++i){require(std::isfinite(knots[i]) && mult[i]>0 && mult[i]<=degree+1 && (!i || knots[i]>knots[i-1]),"Invalid spline knot sequence");sum+=mult[i];}
    require(sum==count+degree+1,"Invalid nonperiodic spline multiplicities");
}
}
bool validateBoundaryBody(const BoundaryBody& body,std::string& error) {
    try {
        require(!body.faces.empty(),"Boundary body has no faces");
        require(std::isfinite(body.tolerance)&&body.tolerance>0,"Invalid boundary assembly tolerance");
        for(const auto& face:body.faces) {
            require(!face.wires.empty(),"Boundary face has no trim wires");const auto& s=face.surface;
            require(finite(s.pose.origin)&&finite(s.pose.xDirection)&&finite(s.pose.zDirection),"Nonfinite surface frame");
            const auto x=s.pose.xDirection,z=s.pose.zDirection;
            require(std::abs(std::hypot(x.x,x.y,x.z)-1)<1e-7 && std::abs(std::hypot(z.x,z.y,z.z)-1)<1e-7
                && std::abs(x.x*z.x+x.y*z.y+x.z*z.z)<1e-7,"Surface frame must be orthonormal");
            for(double v:{s.radius,s.minorRadius,s.semiAngle})require(std::isfinite(v),"Nonfinite analytic surface parameter");
            if(s.kind==BoundarySurface::Kind::Cylinder || s.kind==BoundarySurface::Kind::Sphere)require(s.radius>1e-7,"Invalid fitted surface radius");
            if(s.kind==BoundarySurface::Kind::Cone)require(s.radius>=0 && std::abs(s.semiAngle)>1e-10 && std::abs(s.semiAngle)<std::acos(-1.0)*.5,"Invalid fitted cone parameters");
            if(s.kind==BoundarySurface::Kind::Torus)require(s.radius>s.minorRadius && s.minorRadius>1e-7,"Invalid fitted torus parameters");
            if(s.kind==BoundarySurface::Kind::BSpline) {
                spline(s.uDegree,s.uCount,s.uKnots,s.uMultiplicities);spline(s.vDegree,s.vCount,s.vKnots,s.vMultiplicities);
                require(s.poles.size()==size_t(s.uCount)*s.vCount && s.weights.size()==s.poles.size(),"Invalid surface control net");
                for(auto p:s.poles)require(finite(p),"Nonfinite surface pole");for(auto w:s.weights)require(std::isfinite(w)&&w>0,"Invalid surface weight");
            }
            for(const auto& wire:face.wires) {
                require(!wire.empty(),"Empty trim wire");
                for(const auto& use:wire){const auto& c=use.curve;require(use.edgeId>0,"Invalid trim edge identity");
                    spline(c.degree,int(c.poles.size()),c.knots,c.multiplicities);require(c.weights.size()==c.poles.size(),"Invalid trim weights");
                    for(auto p:c.poles)require(finite(p),"Nonfinite trim pole");for(auto w:c.weights)require(std::isfinite(w)&&w>0,"Invalid trim weight");}
            }
        }
        error.clear();return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
bool buildBoundaryBody(const BoundaryBody& body,TopoDS_Shape& result,std::string& error) {
    result.Nullify();if(!validateBoundaryBody(body,error))return false;
    try {
        const double tolerance=body.tolerance;
        BRepBuilderAPI_Sewing sewing(tolerance,true,true,true,false);
        std::map<int,TopoDS_Edge> edges;
        for(const auto& f:body.faces) {
            const auto support=surface(f.surface);BRep_Builder builder;
            std::map<int,Handle(Geom2d_BSplineCurve)> firstCurves;std::map<int,bool> firstReversed;
            for(const auto& wire:f.wires)for(const auto& use:wire) {
                const auto pc=curve(use.curve);auto found=edges.find(use.edgeId);
                if(found==edges.end()) {
                    BRepBuilderAPI_MakeEdge make(pc,support,pc->FirstParameter(),pc->LastParameter());
                    require(make.IsDone(),"Cannot construct surface trim edge");edges[use.edgeId]=make.Edge();firstCurves[use.edgeId]=pc;firstReversed[use.edgeId]=use.reversed;
                } else if(!firstCurves.count(use.edgeId)) {
                    builder.UpdateEdge(found->second,pc,support,TopLoc_Location(),tolerance);firstCurves[use.edgeId]=pc;firstReversed[use.edgeId]=use.reversed;
                } else if(firstReversed.at(use.edgeId))builder.UpdateEdge(found->second,pc,firstCurves.at(use.edgeId),support,TopLoc_Location(),tolerance);
                else builder.UpdateEdge(found->second,firstCurves.at(use.edgeId),pc,support,TopLoc_Location(),tolerance);
                builder.UpdateEdge(edges.at(use.edgeId),tolerance);
                for(TopExp_Explorer vi(edges.at(use.edgeId),TopAbs_VERTEX);vi.More();vi.Next())builder.UpdateVertex(TopoDS::Vertex(vi.Current()),tolerance);
            }
            for(const auto& pair:firstCurves)BRepLib::BuildCurves3d(edges.at(pair.first),tolerance);
            BRepBuilderAPI_MakeFace make;
            bool outer=true;
            for(const auto& loop:f.wires) {
                BRepBuilderAPI_MakeWire wire;
                for(const auto& use:loop){auto edge=edges.at(use.edgeId);if(use.reversed)edge.Reverse();wire.Add(edge);}
                require(wire.IsDone(),"Cannot close surface trim wire");
                if(outer){make.Init(support,false,tolerance);make.Add(wire.Wire());outer=false;}else make.Add(wire.Wire());
            }
            require(make.IsDone(),"Cannot construct fitted trimmed face");auto face=make.Face();
            BRepLib::SameParameter(face,tolerance,true);
            if(f.reversed)face.Reverse();sewing.Add(face);
        }
        sewing.Perform();const auto sewn=sewing.SewedShape();TopoDS_Shell shell;int shells=0;
        if(sewn.ShapeType()==TopAbs_SHELL){shell=TopoDS::Shell(sewn);shells=1;}
        else if(sewn.ShapeType()==TopAbs_FACE){BRep_Builder builder;builder.MakeShell(shell);builder.Add(shell,sewn);shells=1;}
        else for(TopExp_Explorer it(sewn,TopAbs_SHELL);it.More();it.Next()){shell=TopoDS::Shell(it.Current());++shells;}
        if(shells!=1 || sewing.NbFreeEdges()!=0) {
            double maximumGap=0;std::ostringstream details;details<<std::setprecision(12);
            for(int i=1;i<=sewing.NbFreeEdges();++i){BRepAdaptor_Curve c(sewing.FreeEdge(i));
                const auto a=c.Value(c.FirstParameter()),b=c.Value(c.LastParameter());
                details<<" edge "<<i<<" ("<<a.X()<<","<<a.Y()<<","<<a.Z()<<") -> ("<<b.X()<<","<<b.Y()<<","<<b.Z()<<")";
                const auto vertex=BRepBuilderAPI_MakeVertex(c.Value((c.FirstParameter()+c.LastParameter())*.5)).Vertex();double nearest=INFINITY;
                for(int j=1;j<=sewing.NbFreeEdges();++j)if(i!=j){BRepExtrema_DistShapeShape distance(vertex,sewing.FreeEdge(j));if(distance.IsDone())nearest=std::min(nearest,distance.Value());}
                maximumGap=std::max(maximumGap,nearest);}
            throw std::runtime_error("Fitted boundaries do not form one closed shell: shells="+std::to_string(shells)+", free edges="+std::to_string(sewing.NbFreeEdges())+", maximum nearest midpoint gap="+std::to_string(maximumGap)+details.str());
        }
        auto solid=BRepBuilderAPI_MakeSolid(shell).Solid();require(BRepLib::OrientClosedSolid(solid),"Cannot orient fitted closed solid");
        require(BRepCheck_Analyzer(solid).IsValid(),"Fitted boundary solid is invalid");result=solid;error.clear();return true;
    }catch(const Standard_Failure& e){error=e.what();return false;}catch(const std::exception& e){error=e.what();return false;}
}
}
