#pragma once

#include "modeling/FeatureParams.h"
#include "reconstruction/data/PrimitiveEvidence.h"

#include <functional>
#include <string>

namespace modeling { class ModelingCore; }

namespace reconstruction {

struct ProjectionExtrusionOptions {
    double distanceTolerance = 0; // mm; zero estimates scale from the part.
    std::size_t maximumAxes = 12;
    std::size_t maximumFitPoints = 24000;
    std::size_t maximumScorePoints = 1800;
    std::size_t maximumOperations = 3;
    bool includeAreaDirections = false;
};

struct ProjectionOperation {
    modeling::SketchFeatureParams sketch;
    modeling::ExtrudeOperation operation = modeling::ExtrudeOperation::NewBody;
    double depth = 0;
    bool reverse = false;
    std::vector<PointId> supportPointIds;
};

struct ProjectionFitQuality {
    std::size_t scoredPoints = 0;
    double medianDistance = 0;
    double p95Distance = 0;
    double surfaceInlierFraction = 0;
    double unsupportedFaceAreaFraction = 0;
    double meanNormalErrorDegrees = 0;
    std::size_t normalComparisons = 0;
    double score = 0;
};

struct ProjectionAxisReport {
    modeling::Vec3 axis{};
    std::string source;
    double convexProjectionArea = 0;
    double convexEnvelopeVolume = 0;
    std::string status;
};

struct ProjectionExtrusionResult {
    double distanceTolerance = 0;
    std::vector<ProjectionAxisReport> axes;
    std::vector<ProjectionOperation> operations;
    ProjectionFitQuality quality;
    std::vector<PointId> residualPointIds;
    std::string reason;
};

// Fits one metrically scaled component. Evidence, if supplied, must use the
// same point IDs and frame. Failure retains diagnostic axes and reason.
bool fitProjectionExtrusion(const PointStore& points,
    const ProjectionExtrusionOptions& options, ProjectionExtrusionResult& result,
    const std::vector<PrimitiveEvidence>& evidence = {},
    const std::function<bool()>& cancelled = {});

// Replays a complete program in isolation and publishes it atomically.
bool buildProjectionProgram(const std::vector<ProjectionOperation>& operations,
    modeling::ModelingCore& model, std::string& error);

} // namespace reconstruction
