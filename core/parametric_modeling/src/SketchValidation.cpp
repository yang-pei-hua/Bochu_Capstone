#include "modeling/SketchValidation.h"

#include "occt/Builders.h"

#include <TopoDS_Face.hxx>

namespace modeling {

bool validateSketch(const SketchFeatureParams& params, std::string& error) {
    TopoDS_Face profile;
    return occt::buildSketchFace(
        params,
        occt::datumPlaneFrame(DatumPlane::XY),
        profile,
        error);
}

}  // namespace modeling
