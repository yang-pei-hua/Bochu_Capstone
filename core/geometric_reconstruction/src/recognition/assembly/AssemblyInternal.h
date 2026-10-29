#pragma once
#include "reconstruction/recognition/AssemblyReconstruction.h"
#include "common/Math3.h"
#include <vector>
namespace reconstruction::detail::assembly {
using namespace math3;
using modeling::Vec3;
using Neighbor=std::pair<double,std::size_t>;
struct WorkingData {
    std::vector<PointSample> points;
    std::vector<std::vector<Neighbor>> neighbors;
    std::vector<double> spacing;
    std::vector<int> labels;
    std::vector<std::vector<std::size_t>> groups;
    double tolerance=0;
};
void recoverBoundaries(const WorkingData&,const AssemblyOptions&,AssemblyReconstructionResult&,
    const std::function<bool()>&);
void closeParts(const WorkingData&,const AssemblyOptions&,AssemblyReconstructionResult&,
    const std::function<bool()>&);
}
