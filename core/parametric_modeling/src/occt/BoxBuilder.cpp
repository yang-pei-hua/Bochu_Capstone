#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>

namespace modeling::occt {

bool buildBoxPrimitive(
    const BoxPrimitiveParams& params,
    TopoDS_Shape& output,
    std::string& error) {
    try {
        const gp_Ax2 axes(
            gp_Pnt(params.pose.origin.x, params.pose.origin.y, params.pose.origin.z),
            gp_Dir(params.pose.zDirection.x,
                   params.pose.zDirection.y,
                   params.pose.zDirection.z),
            gp_Dir(params.pose.xDirection.x,
                   params.pose.xDirection.y,
                   params.pose.xDirection.z));
        BRepPrimAPI_MakeBox builder(
            axes, params.sizeX, params.sizeY, params.sizeZ);
        builder.Build();
        if (!builder.IsDone()) {
            error = "OpenCASCADE could not build the box primitive";
            return false;
        }
        output = builder.Shape();
        if (!isValidShape(output, error)) {
            error = "Box primitive produced invalid geometry: " + error;
            return false;
        }
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE box primitive failure: ") +
            failureMessage(failure);
        return false;
    }
}

}  // namespace modeling::occt
