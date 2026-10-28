#include "reconstruction/recognition/CompoundRecognizer.h"
#include "common/Math3.h"
#include "reconstruction/recognition/ThroughHoleRecognizer.h"
#include "reconstruction/recognition/MixedFeatureRecognizer.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace reconstruction {
using math3::add;
using math3::dot;
using math3::scale;
using math3::subtract;
using modeling::Vec3;
using Coord = std::array<double, 3>;

bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error,
                                const std::vector<PrimitiveEvidence>& seeds) {
    output={};
    SurfaceAnalysisResult analysis;
    if (!analyzeSurfaces(points, recognitionAnalysisOptions(points, planes, cylinders), analysis, error, seeds)) return false;
    return recognizeParametricSolidFromAnalysis(points, analysis, output, error);
}

bool recognizeParametricSolidFromAnalysis(const PointStore& points, const SurfaceAnalysisResult& analysis,
                                        CompoundCandidate& output, std::string& error,
                                        std::vector<LocalAnalysisFailure>* failures, const PointStore* source) {
    output = {};
    if (!analysisMatches(points, analysis)) { error = "Stale surface analysis"; return false; }
    const auto& planes = analysis.options.detection.plane;
    if (std::any_of(points.points().begin(), points.points().end(), [](const PointSample& p) {
            return p.normal && !p.normalOrientationKnown;
        })) {
        output = {};
        error = "Surface normal directions lack material orientation; provide oriented normals "
                "or additional observations for disconnected surface patches";
        return false;
    }
    CompoundRecognitionOptions options;
    options.distanceTolerance = planes.distanceThreshold;
    for(const auto& e : analysis.surfaces)
        if(const auto* plane=std::get_if<PlaneEvidence>(&e)) options.boundaryEvidence.push_back(*plane);
    std::string mixedError;
    CompoundCandidate mixed;
    const bool hasMixed = recognizeMixedFeatures(points, analysis, mixed, mixedError, failures, source);
    if(!hasMixed && mixedError!="No observed thin rectangular host") {
        error=mixedError;output={};return false;
    }
    const bool hasCuts = std::any_of(mixed.prisms.begin(),mixed.prisms.end(),[](const auto& p){return p.subtract;});
    if(hasMixed && (!mixed.curvedSolids.empty() || !mixed.throughHoles.empty() || hasCuts)) {
        output=std::move(mixed);return true;
    }
    if (reconstructCompound(points, options, output, error)) {
        if(hasMixed && output.surfaceCoverage<0.995) output=std::move(mixed);
        return true;
    }
    const auto compoundError = error;
    if(hasMixed) {output=std::move(mixed);error.clear();return true;}
    auto tryCurved = [&](std::string previous) {
        CurvedSolidRecognitionOptions curvedOptions;
        curvedOptions.distanceTolerance = planes.distanceThreshold;
        curvedOptions.randomSeed = planes.randomSeed;

        CurvedSolidCandidate curved;
        output = {};
        if (!recognizeCurvedSolidFromAnalysis(points, analysis, curvedOptions, curved, error)) {
            error = compoundError + "; mixed: " + mixedError + "; box/hole: " + previous + "; curved solid: " + error;
            return false;
        }
        output.curvedSolids.push_back(curved);
        output.surfaceCoverage = curved.surfaceCoverage;
        output.confidence = curved.surfaceCoverage;
        error.clear();
        return true;
    };
    BoxWithThroughHolesCandidate legacy;
    if (!recognizeBoxWithThroughHolesFromAnalysis(points, analysis, BoxRecognitionOptions{},
                                        ThroughHoleRecognitionOptions{}, legacy, error)) {
        return tryCurved(error);
    }
    const auto &box = legacy.box.primitive;
    const auto x = math3::normalized(box.pose.xDirection),
               z = math3::normalized(box.pose.zDirection), y = math3::cross(z, x);
    auto contains = [&](const Vec3 &point) {
        auto delta = subtract(point, box.pose.origin);
        const Coord local{{dot(delta, x), dot(delta, y), dot(delta, z)}};
        if (local[0] < 0 || local[0] > box.sizeX || local[1] < 0 || local[1] > box.sizeY ||
            local[2] < 0 || local[2] > box.sizeZ)
            return false;
        for (const auto &hole : legacy.throughHoles) {
            const auto direction = math3::normalized(hole.primitive.axisDirection);
            auto d = subtract(point, hole.primitive.axisPoint);
            auto radial = subtract(d, scale(direction, dot(d, direction)));
            if (dot(radial, radial) < hole.primitive.radius * hole.primitive.radius)
                return false;
        }
        return true;
    };
    std::size_t support = 0;
    for (const auto &p : points.points())
        if (p.normal) {
            const auto shift = scale(math3::normalized(*p.normal), planes.distanceThreshold * 4);
            support += contains(subtract(p.position, shift)) && !contains(add(p.position, shift));
        }
    const double coverage = double(support) / points.size();
    // Curved fused scan normals have larger boundary uncertainty than planar
    // normals. Still require the whole-body hypothesis to explain most points.
    if (coverage < 0.80) {
        return tryCurved("single-box boundary coverage=" + std::to_string(coverage));
    }
    output = {};
    output.boxes.push_back(legacy.box);
    output.throughHoles = legacy.throughHoles;
    output.surfaceCoverage = coverage;
    output.confidence = legacy.confidence;
    output.minimumBoxCountProven = true;
    error.clear();
    return true;
}
} // namespace reconstruction
