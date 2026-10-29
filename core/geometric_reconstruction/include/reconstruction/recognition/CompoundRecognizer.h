#pragma once
#include "modeling/FeatureParams.h"
#include "reconstruction/detection/PrimitiveDetector.h"
#include "reconstruction/data/ReconstructionCandidate.h"
#include "reconstruction/recognition/CurvedSolidRecognizer.h"
#include <string>

namespace reconstruction {
struct PrismCandidate {
    modeling::Plane3d plane{};
    std::vector<modeling::Point2D> polygon;
    double depth = 0;
    bool subtract = false;
};
struct CompoundRecognitionOptions {
    double distanceTolerance = 0.1;
    double normalAlignment = 0.98;
    double maximumCellError = 0.12;
    std::size_t maximumGridCells = 512;
    std::size_t maximumSearchNodes = 200000;
    // Observed equations are shared; interface planes are explicit inferred
    // contacts with an observed host, not fabricated source observations.
    std::vector<PlaneEvidence> boundaryEvidence;
    std::vector<PlaneEquation> interfacePlanes;
};
struct CompoundCandidate {
    std::vector<BoxCandidate> boxes;
    std::vector<PrismCandidate> prisms;
    std::vector<ThroughHoleCandidate> throughHoles;
    std::vector<CurvedSolidCandidate> curvedSolids;
    std::vector<modeling::ProfileBodyParams> profileBodies;
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
struct RepairTransactionReport {
    bool committed = false, reused = false;
    double beforeCoverage = 0, afterCoverage = 0;
    std::string reason;
    std::vector<LocalAnalysisFailure> failures;
};
// Publishes an augmented analysis only after complete recognition and stability checks.
// Failure leaves the supplied analysis unchanged and clears the output candidate.
bool recognizeAndRepairParametricSolid(const PointStore& points, SurfaceAnalysisResult& analysis,
    CompoundCandidate& output, std::string& error, const PointStore* source = nullptr,
    RepairTransactionReport* report = nullptr,
    const std::vector<SurfaceRelationHint>& hints = {});
// Client and validator share this entry point. Curved-feature fallback is
// accepted only if the complete surface agrees with its predicted boundary.
bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error,
                                const std::vector<PrimitiveEvidence>& seeds = {});
bool recognizeParametricSolidFromAnalysis(const PointStore& points, const SurfaceAnalysisResult& analysis,
                                        CompoundCandidate& output, std::string& error,
                                        std::vector<LocalAnalysisFailure>* failures = nullptr,
                                        const PointStore* source = nullptr);
} // namespace reconstruction
