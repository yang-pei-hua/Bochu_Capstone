#pragma once
#include "modeling/PrimitiveParams.h"
#include <TopoDS_Shape.hxx>
#include <vector>
#include <string>

namespace modeling {
// Explicit rational spline data. P-curves use XY poles; surfaces use XYZ poles
// in U-major order. Knots and multiplicities describe the parameter-space basis.
struct SplineCurve2D {
    int degree=1;
    std::vector<Vec3> poles;
    std::vector<double> weights,knots;
    std::vector<int> multiplicities;
};
struct BoundaryEdgeUse {
    int edgeId=0; // Shared topology identity across faces, including seams.
    bool reversed=false;
    SplineCurve2D curve;
};
struct BoundarySurface {
    enum class Kind { Plane, Cylinder, Cone, Sphere, Torus, BSpline };
    Kind kind=Kind::Plane;
    Pose3d pose;
    double radius=0,minorRadius=0,semiAngle=0;
    int uDegree=1,vDegree=1,uCount=0,vCount=0;
    std::vector<Vec3> poles;
    std::vector<double> weights,uKnots,vKnots;
    std::vector<int> uMultiplicities,vMultiplicities;
};
struct BoundaryFace {
    BoundarySurface surface;
    bool reversed=false;
    // First wire is the outer wire. Remaining wires bound holes.
    std::vector<std::vector<BoundaryEdgeUse>> wires;
};
struct BoundaryBody {
    std::vector<BoundaryFace> faces;
    double tolerance=1e-5;
};
bool validateBoundaryBody(const BoundaryBody& params,std::string& error);
bool buildBoundaryBody(const BoundaryBody& params,TopoDS_Shape& shape,std::string& error);
}
