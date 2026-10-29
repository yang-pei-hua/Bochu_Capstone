#pragma once
#include "modeling/PrimitiveParams.h"
#include "modeling/BoundaryBody.h"
#include <TopoDS_Shape.hxx>
#include <vector>
#include <optional>

namespace modeling {
// Explicit analytic profile data, not a stored/imported B-Rep. Angles use the
// body's local XY frame; a signed sweep preserves wire traversal direction.
struct ProfileCurve2D {
    bool circular=false;
    double x1=0,y1=0,x2=0,y2=0;
    double centerX=0,centerY=0,radius=0,startAngle=0,sweepAngle=0;
};
struct AxialSection {
    double centerX=0,centerY=0,start=0,end=0,startRadius=0,endRadius=0;
    double innerStartRadius=0,innerEndRadius=0;
    // Axis in the body frame. The centre plus axis*start/end locates the caps.
    double centerZ=0;
    Vec3 axis{0,0,1};
};
struct ProfileBodyParams {
    Pose3d pose;
    std::vector<ProfileCurve2D> outline;
    // Noncircular closed cutout profiles extruded through the complete depth.
    std::vector<std::vector<ProfileCurve2D>> profileVoids;
    // Nonempty for a stepped rotational envelope. Otherwise extrude outline.
    std::vector<AxialSection> envelope;
    std::vector<AxialSection> cavities;
    double depth=0;
    double referenceSizeX=0,referenceSizeY=0;
    double sizeX=0,sizeY=0,sizeZ=0;
    // For bodies with intersecting/mitered/freeform faces, rebuild from fitted
    // surface coefficients and explicit parameter-space trim loops.
    std::optional<BoundaryBody> boundary;
};
bool validateProfileBody(const ProfileBodyParams& params,std::string& error);
bool buildProfileBody(const ProfileBodyParams& params,TopoDS_Shape& output,std::string& error);
}
