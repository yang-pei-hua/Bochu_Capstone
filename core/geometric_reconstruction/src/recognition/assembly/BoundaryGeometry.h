#pragma once
#include "reconstruction/recognition/AssemblyReconstruction.h"

namespace reconstruction::detail::assembly {
using BoundaryLoop = std::vector<modeling::Point2D>;
struct BoundaryLine {
    double a=0,b=0,c=0;
    int identity=0;
    modeling::Point2D anchor{};
};

double boundaryArea(const BoundaryLoop& loop);
bool simpleBoundary(const BoundaryLoop& loop);
// A triangle-boundary walk can touch the same vertex several times. Split at
// those vertices rather than publishing a figure-eight as one polygon.
std::vector<BoundaryLoop> splitBoundaryCycles(const BoundaryLoop& walk);
BoundaryLoop simplifyBoundary(const BoundaryLoop& loop,double tolerance,
    BoundaryRecoveryDiagnostics& diagnostics);
BoundaryLoop snapBoundary(const BoundaryLoop& loop,const std::vector<BoundaryLine>& constraints,
    double spacing,double reach,BoundaryRecoveryDiagnostics& diagnostics,std::size_t& snapped);
bool sameBoundaryTopology(const std::vector<BoundaryLoop>& original,
                          const std::vector<BoundaryLoop>& candidate);
}
