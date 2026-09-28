#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>

namespace modeling::occt {

bool buildThroughHolePrimitive(
    const TopoDS_Shape& body,
    const ThroughHolePrimitiveParams& params,
    TopoDS_Shape& output,
    std::string& error) {
    try {
        if (body.IsNull()) {
            error = "Through hole requires an existing body";
            return false;
        }

        const gp_Dir direction(
            params.axisDirection.x,
            params.axisDirection.y,
            params.axisDirection.z);
        const double span = throughAllDistance(body);
        gp_Vec backwards(direction);
        backwards.Multiply(-span);
        const gp_Pnt center(
            params.axisPoint.x, params.axisPoint.y, params.axisPoint.z);
        const gp_Pnt start = center.Translated(backwards);

        BRepPrimAPI_MakeCylinder cylinder(
            gp_Ax2(start, direction), params.radius, span * 2.0);
        cylinder.Build();
        if (!cylinder.IsDone()) {
            error = "OpenCASCADE could not build the through-hole tool";
            return false;
        }

        BRepAlgoAPI_Cut cut(body, cylinder.Shape());
        cut.SetRunParallel(false);
        cut.Build();
        if (!cut.IsDone()) {
            error = "OpenCASCADE through-hole boolean cut failed";
            return false;
        }

        output = cut.Shape();
        if (!isValidShape(output, error)) {
            error = "Through hole produced invalid geometry: " + error;
            return false;
        }
        const double beforeVolume = shapeVolume(body);
        const double afterVolume = shapeVolume(output);
        const double tolerance = std::max(1.0, beforeVolume) * 1.0e-9;
        if (!(afterVolume < beforeVolume - tolerance)) {
            error = "Through-hole axis does not intersect the body";
            return false;
        }
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE through-hole failure: ") +
            failureMessage(failure);
        return false;
    }
}

}  // namespace modeling::occt
