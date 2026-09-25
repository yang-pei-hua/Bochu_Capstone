#include "RebuildEngine.h"

#include "modeling/PartDocument.h"
#include "occt/Builders.h"

#include <TopoDS_Face.hxx>

#include <string>
#include <unordered_map>
#include <variant>

namespace modeling {
namespace {

struct SketchBuildState {
    TopoDS_Face profile;
    occt::PlaneFrame frame;
};

struct ExtrudeBuildState {
    occt::ExtrudeResult result;
};

using DerivedFeatureState = std::variant<SketchBuildState, ExtrudeBuildState>;
using DerivedStateMap = std::unordered_map<FeatureId, DerivedFeatureState>;

bool resolvePlane(
    const SketchPlaneReference& reference,
    const DerivedStateMap& states,
    occt::PlaneFrame& frame,
    std::string& error) {
    if (const auto* datum = std::get_if<DatumPlaneReference>(&reference)) {
        frame = occt::datumPlaneFrame(datum->plane);
        return true;
    }

    const FaceReference& faceReference = std::get<FaceReference>(reference);
    const auto owner = states.find(faceReference.ownerFeature);
    if (owner == states.end()) {
        error = "ReferenceLost: owner feature " +
            std::to_string(faceReference.ownerFeature) +
            " has no geometry in the current rebuild";
        return false;
    }
    const auto* extrude = std::get_if<ExtrudeBuildState>(&owner->second);
    if (extrude == nullptr) {
        error = "ReferenceLost: referenced owner is not an Extrude feature";
        return false;
    }

    switch (faceReference.role) {
    case FaceRole::StartFace:
        frame = extrude->result.startFace;
        return true;
    case FaceRole::EndFace:
        frame = extrude->result.endFace;
        return true;
    case FaceRole::Unknown:
        error = "ReferenceLost: V0 requires semantic StartFace or EndFace";
        return false;
    }
    error = "ReferenceLost: unsupported face role";
    return false;
}

}  // namespace

bool RebuildEngine::rebuild(PartDocument& document) const {
    document.lastError_.clear();
    document.bodyShape_.Nullify();

    for (Feature& feature : document.features_) {
        feature.valid = true;
        feature.errorText.clear();
    }

    DerivedStateMap states;
    TopoDS_Shape currentBody;

    for (std::size_t index = 0; index < document.features_.size(); ++index) {
        Feature& feature = document.features_[index];
        if (feature.suppressed) {
            continue;
        }

        std::string error;
        switch (feature.type) {
        case FeatureType::Sketch: {
            const auto* params = std::get_if<SketchFeatureParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match Sketch";
                break;
            }
            occt::PlaneFrame frame;
            if (!resolvePlane(params->plane, states, frame, error)) {
                break;
            }
            TopoDS_Face profile;
            if (!occt::buildSketchFace(*params, frame, profile, error)) {
                break;
            }
            states.emplace(feature.id, SketchBuildState{profile, frame});
            continue;
        }
        case FeatureType::Extrude: {
            const auto* params = std::get_if<ExtrudeFeatureParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match Extrude";
                break;
            }
            if (!currentBody.IsNull()) {
                error = "V0 supports only one New Body Extrude";
                break;
            }
            const auto sketch = states.find(params->sketchId);
            if (sketch == states.end() ||
                !std::holds_alternative<SketchBuildState>(sketch->second)) {
                error = "Missing or invalid upstream sketch " +
                    std::to_string(params->sketchId);
                break;
            }
            const SketchBuildState& sketchState =
                std::get<SketchBuildState>(sketch->second);
            occt::ExtrudeResult result;
            if (!occt::buildExtrude(
                    sketchState.profile, sketchState.frame, *params, result, error)) {
                break;
            }
            currentBody = result.shape;
            states.emplace(feature.id, ExtrudeBuildState{result});
            continue;
        }
        case FeatureType::Cut: {
            const auto* params = std::get_if<CutFeatureParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match Cut";
                break;
            }
            const auto sketch = states.find(params->sketchId);
            if (sketch == states.end() ||
                !std::holds_alternative<SketchBuildState>(sketch->second)) {
                error = "Missing or invalid upstream sketch " +
                    std::to_string(params->sketchId);
                break;
            }
            const SketchBuildState& sketchState =
                std::get<SketchBuildState>(sketch->second);
            TopoDS_Shape cutResult;
            if (!occt::buildCut(
                    currentBody,
                    sketchState.profile,
                    sketchState.frame,
                    *params,
                    cutResult,
                    error)) {
                break;
            }
            currentBody = cutResult;
            continue;
        }
        }

        feature.valid = false;
        feature.errorText = error;
        document.lastError_ = feature.name + ": " + error;
        document.bodyShape_ = currentBody;

        for (std::size_t later = index + 1;
             later < document.features_.size();
             ++later) {
            if (!document.features_[later].suppressed) {
                document.features_[later].valid = false;
                document.features_[later].errorText =
                    "Skipped because rebuild stopped after " + feature.name;
            }
        }
        return false;
    }

    document.bodyShape_ = currentBody;
    return true;
}

}  // namespace modeling
