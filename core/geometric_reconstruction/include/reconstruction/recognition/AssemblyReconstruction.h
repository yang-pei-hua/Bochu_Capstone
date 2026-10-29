#pragma once
#include "reconstruction/data/PointStore.h"
#include "reconstruction/recognition/CompoundRecognizer.h"
#include <functional>
#include <string>

namespace reconstruction {
struct AssemblyOptions {
    double distanceTolerance=0; // Millimetres. Zero estimates noise from observations.
    std::size_t maximumWorkingPoints=200000;
    std::size_t neighbors=32,minimumPatchPoints=24;
    std::size_t maximumPrimitiveAttempts=32;
    double normalAngleRadians=.174532925199433;
    double connectivityFactor=2.5,alphaFactor=2.5;
    bool buildSolids=true;
};
struct FiniteSurfacePatch {
    std::size_t id=0;
    modeling::Vec3 origin{},normal{},xDirection{},lower{},upper{};
    double spacing=0,residual=0;
    bool planar=false,orientationKnown=false;
    std::vector<PointId> pointIds;
    // Ordered finite boundaries in world coordinates. First loop encloses the
    // largest area; additional loops can describe holes or disconnected islands.
    std::vector<std::vector<modeling::Vec3>> loops;
    std::size_t snappedVertices=0;
};
struct PatchConnection {
    std::size_t first=0,second=0;
    bool convex=false;
    std::size_t observations=0;
    double distance=0;
    bool accepted=false,orientationKnown=false;
};
struct AssemblyPart {
    std::size_t id=0;
    std::vector<std::size_t> patchIds;
    std::vector<PointId> pointIds;
    modeling::Vec3 lower{},upper{};
    bool closed=false,orientationKnown=false;
    bool normalOrientationInferred=false;
    std::size_t openEdges=0,nonmanifoldEdges=0;
    double confidence=0;
    std::string status,reason;
};
struct AssemblyReconstructionResult {
    std::size_t inputPoints=0,workingPoints=0;
    double distanceTolerance=0;
    std::vector<FiniteSurfacePatch> patches;
    std::vector<PatchConnection> connections;
    std::vector<AssemblyPart> parts;
    std::vector<PointId> unassignedPointIds;
    // CAD contains only parts that pass closed-shell and B-Rep checks.
    // candidate.surfaceCoverage counts observations assigned to those parts;
    // it is not a source completeness score or an independent surface distance.
    CompoundCandidate candidate;
};
// Observation-only convex-patch decomposition. Candidates are not asserted to
// be original assembly instances; concave parts/contacts can remain ambiguous.
// No source shape, file path, reference labels or topology input is accepted.
bool reconstructAssembly(const PointStore& points,const AssemblyOptions& options,
    AssemblyReconstructionResult& output,std::string& error,
    const std::function<bool()>& cancelled={});
}
