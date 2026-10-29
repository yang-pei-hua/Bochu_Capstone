#include "modeling/ProfileBody.h"
#include "modeling/PrimitiveEditing.h"
#include "occt/Builders.h"
#include "occt/OcctError.h"
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <TopTools_ListOfShape.hxx>
#include <Geom_Circle.hxx>
#include <gp_GTrsf.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <cmath>

namespace modeling {
bool validateProfileBody(const ProfileBodyParams& p,std::string& error) {
    for(double v:{p.depth,p.referenceSizeX,p.referenceSizeY,p.sizeX,p.sizeY,p.sizeZ})
        if(!std::isfinite(v) || v<=1e-7){error="Profile body dimensions must be finite and positive";return false;}
    const auto finite=[](Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    const auto x=p.pose.xDirection,z=p.pose.zDirection;
    if(!finite(p.pose.origin)||!finite(x)||!finite(z)||
       std::abs(std::hypot(x.x,x.y,x.z)-1)>1e-7 || std::abs(std::hypot(z.x,z.y,z.z)-1)>1e-7 ||
       std::abs(x.x*z.x+x.y*z.y+x.z*z.z)>1e-7){error="Profile body pose must be orthonormal";return false;}
    if(p.outline.empty() && p.envelope.empty() && !p.boundary){error="Profile body needs an analytic outline, envelope or fitted boundary";return false;}
    if(p.boundary && !validateBoundaryBody(*p.boundary,error))return false;
    const auto validateCurves=[&](const std::vector<ProfileCurve2D>& curves) {
      for(const auto& c:curves) {
        for(double v:{c.x1,c.y1,c.x2,c.y2,c.centerX,c.centerY,c.radius,c.startAngle,c.sweepAngle})
            if(!std::isfinite(v)){error="Nonfinite profile curve";return false;}
        if(c.circular && (c.radius<=1e-7 || std::abs(c.sweepAngle)<1e-10 || std::abs(c.sweepAngle)>2*std::acos(-1.0)+1e-7)) {
            error="Invalid circular profile arc";return false;
        }
      }
      return true;
    };
    if(!validateCurves(p.outline))return false;
    for(const auto& loop:p.profileVoids) {
        if(loop.empty()){error="Empty profile cutout";return false;}
        if(!validateCurves(loop))return false;
    }
    for(const auto* sections:{&p.envelope,&p.cavities})for(const auto& s:*sections) {
        for(double v:{s.centerX,s.centerY,s.centerZ,s.axis.x,s.axis.y,s.axis.z,s.start,s.end,s.startRadius,s.endRadius,s.innerStartRadius,s.innerEndRadius})if(!std::isfinite(v)){error="Nonfinite axial section";return false;}
        if(std::abs(std::hypot(s.axis.x,s.axis.y,s.axis.z)-1)>1e-7){error="Section axis must be a unit direction";return false;}
        if(s.end-s.start<=1e-7 || s.startRadius<0 || s.endRadius<0 || std::max(s.startRadius,s.endRadius)<=1e-7) {
            error="Invalid axial section dimensions";return false;
        }
        if(s.innerStartRadius<0 || s.innerEndRadius<0 || s.innerStartRadius>s.startRadius || s.innerEndRadius>s.endRadius ||
           (s.innerStartRadius==s.startRadius && s.innerEndRadius==s.endRadius)){error="Invalid annular section radii";return false;}
    }
    error.clear();return true;
}
bool buildProfileBody(const ProfileBodyParams& p,TopoDS_Shape& output,std::string& error) {
    output.Nullify();if(!validateProfileBody(p,error))return false;
    try {
        auto prism=[&](const std::vector<ProfileCurve2D>& curves)->TopoDS_Shape {
            BRepBuilderAPI_MakeWire wire;
            for(const auto& c:curves) {
                TopoDS_Edge edge;
                if(!c.circular)edge=BRepBuilderAPI_MakeEdge(gp_Pnt(c.x1,c.y1,0),gp_Pnt(c.x2,c.y2,0)).Edge();
                else {
                    const Handle(Geom_Circle) circle=new Geom_Circle(gp_Ax2(gp_Pnt(c.centerX,c.centerY,0),gp_Dir(0,0,1)),c.radius);
                    const double low=std::min(c.startAngle,c.startAngle+c.sweepAngle),high=std::max(c.startAngle,c.startAngle+c.sweepAngle);
                    edge=BRepBuilderAPI_MakeEdge(circle,low,high).Edge();if(c.sweepAngle<0)edge.Reverse();
                }
                wire.Add(edge);
            }
            if(!wire.IsDone())throw Standard_Failure("Analytic profile does not form a wire");
            const auto face=BRepBuilderAPI_MakeFace(wire.Wire(),true).Face();
            return BRepPrimAPI_MakePrism(face,gp_Vec(0,0,p.depth)).Shape();
        };
        auto section=[](const AxialSection& s)->TopoDS_Shape {
            const gp_Ax2 axis(gp_Pnt(s.centerX+s.axis.x*s.start,s.centerY+s.axis.y*s.start,s.centerZ+s.axis.z*s.start),gp_Dir(s.axis.x,s.axis.y,s.axis.z));
            auto cone=[&](double r0,double r1)->TopoDS_Shape {
                if(std::abs(r0-r1)<1e-8)return BRepPrimAPI_MakeCylinder(axis,(r0+r1)*0.5,s.end-s.start).Shape();
                return BRepPrimAPI_MakeCone(axis,r0,r1,s.end-s.start).Shape();
            };
            const auto outer=cone(s.startRadius,s.endRadius);
            if(std::max(s.innerStartRadius,s.innerEndRadius)<=1e-7)return outer;
            BRepAlgoAPI_Cut inner(outer,cone(s.innerStartRadius,s.innerEndRadius));
            if(!inner.IsDone())throw Standard_Failure("Annular section subtraction failed");
            return inner.Shape();
        };
        TopoDS_Shape body;
        if(p.boundary) {
            if(!buildBoundaryBody(*p.boundary,body,error))return false;
        } else if(!p.envelope.empty()) {
            for(const auto& s:p.envelope) {
                const auto solid=section(s);
                if(body.IsNull())body=solid;
                else {BRepAlgoAPI_Fuse fuse(body,solid);fuse.SetFuzzyValue(1e-7);fuse.Build();
                    if(!fuse.IsDone()){error="Profile envelope union failed";return false;}body=fuse.Shape();}
            }
        } else {
            body=prism(p.outline);
        }
        if(!p.cavities.empty() || !p.profileVoids.empty()) {
            BRepAlgoAPI_Cut cut;TopTools_ListOfShape arguments,tools;arguments.Append(body);
            for(const auto& s:p.cavities)tools.Append(section(s));
            for(const auto& loop:p.profileVoids)tools.Append(prism(loop));
            cut.SetArguments(arguments);cut.SetTools(tools);cut.SetFuzzyValue(1e-7);cut.SetRunParallel(false);cut.Build();
            if(!cut.IsDone()){error="Profile cavity subtraction failed";return false;}body=cut.Shape();
        }
        const double sx=p.sizeX/p.referenceSizeX,sy=p.sizeY/p.referenceSizeY,sz=p.sizeZ/p.depth;
        if(std::abs(sx-1)>1e-12 || std::abs(sy-1)>1e-12 || std::abs(sz-1)>1e-12) {
            gp_GTrsf scale;scale.SetValue(1,1,sx);scale.SetValue(2,2,sy);scale.SetValue(3,3,sz);
            scale.SetTranslationPart(gp_XYZ(0,0,p.depth*(1-sz)*0.5));
            body=BRepBuilderAPI_GTransform(body,scale,true).Shape();
        }
        const auto x=p.pose.xDirection,z=p.pose.zDirection;
        const Vec3 y{z.y*x.z-z.z*x.y,z.z*x.x-z.x*x.z,z.x*x.y-z.y*x.x};
        gp_Trsf placement;placement.SetValues(x.x,y.x,z.x,p.pose.origin.x,x.y,y.y,z.y,p.pose.origin.y,x.z,y.z,z.z,p.pose.origin.z);
        output=BRepBuilderAPI_Transform(body,placement,true).Shape();
        int solids=0;for(TopExp_Explorer it(output,TopAbs_SOLID);it.More();it.Next())++solids;
        if(solids!=1){error="Profile body must rebuild to exactly one solid";output.Nullify();return false;}
        if(!occt::isValidShape(output,error)){output.Nullify();return false;}return true;
    } catch(const Standard_Failure& failure) {error=occt::failureMessage(failure);output.Nullify();return false;}
}
bool refreshProfileBodyDimensions(ProfileBodyParams& p,std::string& error) {
    const double sx=p.sizeX/p.referenceSizeX,sy=p.sizeY/p.referenceSizeY,sz=p.sizeZ/p.depth,oldDepth=p.depth;
    auto unscaled=p;unscaled.pose={};unscaled.sizeX=p.referenceSizeX;unscaled.sizeY=p.referenceSizeY;unscaled.sizeZ=p.depth;
    TopoDS_Shape shape;if(!buildProfileBody(unscaled,shape,error))return false;
    Bnd_Box box;BRepBndLib::AddOptimal(shape,box,false,false);double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    const double cx=(x0+x1)*0.5,cy=(y0+y1)*0.5,depth=z1-z0;
    for(auto& c:p.outline){c.x1-=cx;c.x2-=cx;c.y1-=cy;c.y2-=cy;c.centerX-=cx;c.centerY-=cy;}
    for(auto& loop:p.profileVoids)for(auto& c:loop){c.x1-=cx;c.x2-=cx;c.y1-=cy;c.y2-=cy;c.centerX-=cx;c.centerY-=cy;}
    for(auto* sections:{&p.envelope,&p.cavities})for(auto& s:*sections){s.centerX-=cx;s.centerY-=cy;s.centerZ-=z0;}
    if(p.boundary)for(auto& face:p.boundary->faces){auto& s=face.surface;
        s.pose.origin.x-=cx;s.pose.origin.y-=cy;s.pose.origin.z-=z0;
        for(auto& pole:s.poles){pole.x-=cx;pole.y-=cy;pole.z-=z0;}}
    const auto x=p.pose.xDirection,z=p.pose.zDirection;const Vec3 y{z.y*x.z-z.z*x.y,z.z*x.x-z.x*x.z,z.x*x.y-z.y*x.x};
    const double dz=sz*z0+(oldDepth-depth)*(1-sz)*0.5;
    p.pose.origin.x+=x.x*sx*cx+y.x*sy*cy+z.x*dz;p.pose.origin.y+=x.y*sx*cx+y.y*sy*cy+z.y*dz;p.pose.origin.z+=x.z*sx*cx+y.z*sy*cy+z.z*dz;
    p.referenceSizeX=x1-x0;p.referenceSizeY=y1-y0;p.depth=depth;p.sizeX=p.referenceSizeX*sx;p.sizeY=p.referenceSizeY*sy;p.sizeZ=depth*sz;
    return validateProfileBody(p,error);
}
}
