#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepPrimAPI_MakePrism.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace modeling::occt {

bool buildExtrude(
    const TopoDS_Face& profile,
    const PlaneFrame& profileFrame,
    const ExtrudeFeatureParams& params,
    ExtrudeResult& output,
    std::string& error) {
    try {
        if (!std::isfinite(params.depth) || params.depth <= Precision::Confusion()) {
            error = "Extrude depth must be finite and positive";
            return false;
        }

        gp_Dir extrusionDirection = profileFrame.normal;
        if (params.reverse) {
            extrusionDirection.Reverse();
        }
        gp_Vec prismVector(extrusionDirection);
        prismVector.Multiply(params.depth);

        BRepPrimAPI_MakePrism prism(profile, prismVector, false, true);
        prism.Build();
        if (!prism.IsDone()) {
            error = "OpenCASCADE could not build the extrusion";
            return false;
        }

        output.shape = prism.Shape();
        if (!isValidShape(output.shape, error)) {
            error = "Extrude produced invalid geometry: " + error;
            return false;
        }

        gp_Dir startNormal = extrusionDirection;
        startNormal.Reverse();
        output.startFace = translatedFrame(profileFrame, startNormal, 0.0);
        output.endFace = translatedFrame(
            profileFrame, extrusionDirection, params.depth);
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE extrude failure: ") +
            failureMessage(failure);
        return false;
    }
}

}  // namespace modeling::occt
