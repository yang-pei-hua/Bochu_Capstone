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
struct PatchExtraction {
    WorkingData work;
    std::vector<FiniteSurfacePatch> patches;
    std::vector<PointId> unassignedPointIds;
};
struct PatchGrouping {
    std::vector<PatchConnection> connections;
    std::vector<AssemblyPart> parts;
    std::vector<std::size_t> patchParts; // Patch index to part index.
};
PatchExtraction extractPatches(const PointStore&, const AssemblyOptions&, const std::function<bool()>&);
PatchGrouping groupPatches(const WorkingData&, const std::vector<FiniteSurfacePatch>&, const AssemblyOptions&);
std::vector<PointId> assignSourceObservations(const PointStore&, const WorkingData&, const AssemblyOptions&,
    std::vector<FiniteSurfacePatch>&, PatchGrouping&, const std::function<bool()>&);
void recoverBoundaries(const WorkingData&, const AssemblyOptions&, const std::vector<PatchConnection>&,
    std::vector<FiniteSurfacePatch>&, const std::function<bool()>&);
CompoundCandidate closeParts(const WorkingData&, const AssemblyOptions&, std::vector<FiniteSurfacePatch>&,
    std::vector<AssemblyPart>&, std::size_t inputPoints, const std::function<bool()>&);
}
