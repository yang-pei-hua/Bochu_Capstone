#include "detail/CompoundInternal.h"

namespace reconstruction {
using namespace detail::compound;
bool reconstructCompound(const PointStore& store, const CompoundRecognitionOptions& options,
                         CompoundCandidate& output, std::string& error) {
    output = {};
    error.clear();
    if (store.unit() != LengthUnit::Millimeter || store.size() < 30 ||
        !(options.distanceTolerance > 0) ||
        !(options.normalAlignment > 0.9 && options.normalAlignment < 1) ||
        !(options.maximumCellError >= 0 && options.maximumCellError < 0.5)) {
        error = "Compound fitting requires millimetre points, outward normals and valid tolerances";
        return false;
    }
    const auto &points = store.points();
    Arrangement arrangement;
    return recoverPlanarFrame(points,options,arrangement,error)
        && classifyMaterialCells(store,options,arrangement,output,error)
        && selectBoxCover(arrangement,options,output,error)
        && validateCompoundBoundary(points,options,arrangement.axes,output,error);
}
} // namespace reconstruction
