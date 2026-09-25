#include "occt/Builders.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace modeling::occt {

bool buildCut(
    const TopoDS_Shape& body,
    const TopoDS_Face& profile,
    const PlaneFrame& profileFrame,
    const CutFeatureParams& params,
    TopoDS_Shape& output,
    std::string& error) {
    try {
        if (body.IsNull()) {
            error = "Cut requires an existing body";
            return false;
        }
        if (!params.throughAll &&
            (!std::isfinite(params.depth) || params.depth <= Precision::Confusion())) {
            error = "Blind cut depth must be finite and positive";
            return false;
        }

        const double distance = params.throughAll
            ? throughAllDistance(body)
            : params.depth;
        gp_Vec cutVector(profileFrame.normal);
        cutVector.Reverse();
        cutVector.Multiply(distance);

        BRepPrimAPI_MakePrism prism(profile, cutVector, false, true);
        prism.Build();
        if (!prism.IsDone()) {
            error = "OpenCASCADE could not build the cut tool";
            return false;
        }

        BRepAlgoAPI_Cut cut(body, prism.Shape());
        cut.SetRunParallel(false);
        cut.Build();
        if (!cut.IsDone()) {
            error = "OpenCASCADE boolean cut failed";
            return false;
        }

        output = cut.Shape();
        if (!isValidShape(output, error)) {
            error = "Cut produced invalid geometry: " + error;
            return false;
        }

        const double beforeVolume = shapeVolume(body);
        const double afterVolume = shapeVolume(output);
        const double tolerance = std::max(1.0, beforeVolume) * 1.0e-9;
        if (!(afterVolume < beforeVolume - tolerance)) {
            error = "Cut tool does not intersect the body";
            return false;
        }
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE cut failure: ") + failure.what();
        return false;
    }
}

}  // namespace modeling::occt
