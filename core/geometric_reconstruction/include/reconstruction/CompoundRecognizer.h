#pragma once
#include "modeling/FeatureParams.h"
#include "reconstruction/PrimitiveDetector.h"
#include "reconstruction/ReconstructionCandidate.h"
#include <string>

namespace reconstruction {
struct PrismCandidate {
    modeling::Plane3d plane{};
    std::vector<modeling::Point2D> polygon;
    double depth = 0;
};
struct CompoundRecognitionOptions {
    double distanceTolerance = 0.1;
    double normalAlignment = 0.98;
    double maximumCellError = 0.12;
    std::size_t maximumGridCells = 512;
    std::size_t maximumSearchNodes = 200000;
};
struct CompoundCandidate {
    std::vector<BoxCandidate> boxes;
    std::vector<PrismCandidate> prisms;
    std::vector<ThroughHoleCandidate> throughHoles;
    double occupancyError = 0;
    double surfaceCoverage = 0;
    double confidence = 0;
    std::size_t gridCells = 0;
    std::size_t searchNodes = 0;
    bool minimumBoxCountProven = false;
};
// Closed, metrically scaled surface cloud with outward normals. Selects the
// smallest exact box cover of observed material cells; oblique cells become
// polygonal extrusions rather than stair-step boxes.
bool reconstructCompound(const PointStore &points, const CompoundRecognitionOptions &options,
                         CompoundCandidate &output, std::string &error);
// Client and validator share this entry point. Curved-feature fallback is
// accepted only if the complete surface agrees with its predicted boundary.
bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error);
} // namespace reconstruction
