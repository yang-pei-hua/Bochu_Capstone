#include "RebuildEngine.h"

#include "modeling/PartDocument.h"
#include "modeling/SemanticFaces.h"
#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>

#include <cmath>
#include <string>
#include <unordered_map>
#include <variant>

namespace modeling {
namespace {

struct SketchBuildState {
    TopoDS_Face profile;
    occt::PlaneFrame frame;
    std::string profileError;
};

struct ExtrudeBuildState {
    occt::ExtrudeResult result;
};

struct PrimitiveBuildState { FeatureParams params; };
using DerivedFeatureState = std::variant<SketchBuildState, ExtrudeBuildState, PrimitiveBuildState>;
using DerivedStateMap = std::unordered_map<FeatureId, DerivedFeatureState>;

void appendIndependentSolid(const TopoDS_Shape& solid, TopoDS_Shape& currentBody) {
    if (currentBody.IsNull()) {
        currentBody = solid;
        return;
    }
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    builder.Add(compound, currentBody);
    builder.Add(compound, solid);
    currentBody = compound;
}

bool resolvePlane(
    const SketchPlaneReference& reference,
    const DerivedStateMap& states,
    occt::PlaneFrame& frame,
    std::string& error) {
    if (!std::holds_alternative<FaceReference>(reference)) {
        Plane3d plane;
        if (!resolveSketchPlane({},reference,plane,error)) return false;
        frame.origin = gp_Pnt(plane.origin.x,plane.origin.y,plane.origin.z);
        frame.normal = gp_Dir(plane.normal.x,plane.normal.y,plane.normal.z);
        frame.xDirection = gp_Dir(plane.xDirection.x,plane.xDirection.y,plane.xDirection.z);
        frame.yDirection = gp_Dir(
            plane.normal.y*plane.xDirection.z-plane.normal.z*plane.xDirection.y,
            plane.normal.z*plane.xDirection.x-plane.normal.x*plane.xDirection.z,
            plane.normal.x*plane.xDirection.y-plane.normal.y*plane.xDirection.x);
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
    if (const auto* primitive = std::get_if<PrimitiveBuildState>(&owner->second)) {
        Plane3d plane;
        return primitiveFacePlane(primitive->params, faceReference.role, plane, error)
            && resolvePlane(plane, states, frame, error);
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
    default:
        error = "ReferenceLost: extrusion requires semantic StartFace or EndFace";
        return false;
    }
    error = "ReferenceLost: unsupported face role";
    return false;
}

bool applyExtrudeOperation(
    const ExtrudeFeatureParams& params,
    const TopoDS_Shape& tool,
    TopoDS_Shape& currentBody,
    std::string& error) {
    try {
        if (params.operation == ExtrudeOperation::NewBody) {
            appendIndependentSolid(tool, currentBody);
            return true;
        }
        if (currentBody.IsNull()) {
            error = "Extrude boolean operation requires an existing body";
            return false;
        }

        TopoDS_Shape result;
        switch (params.operation) {
        case ExtrudeOperation::Join: {
            BRepAlgoAPI_Fuse operation(currentBody, tool);
            operation.SetRunParallel(false);
            operation.Build();
            if (!operation.IsDone()) {
                error = "OpenCASCADE boolean join failed";
                return false;
            }
            result = operation.Shape();
            break;
        }
        case ExtrudeOperation::Cut: {
            BRepAlgoAPI_Cut operation(currentBody, tool);
            operation.SetRunParallel(false);
            operation.Build();
            if (!operation.IsDone()) {
                error = "OpenCASCADE boolean cut failed";
                return false;
            }
            result = operation.Shape();
            break;
        }
        case ExtrudeOperation::Intersect: {
            BRepAlgoAPI_Common operation(currentBody, tool);
            operation.SetRunParallel(false);
            operation.Build();
            if (!operation.IsDone()) {
                error = "OpenCASCADE boolean intersection failed";
                return false;
            }
            result = operation.Shape();
            break;
        }
        case ExtrudeOperation::NewBody:
            break;
        }
        if (!occt::isValidShape(result, error)) {
            error = "Extrude boolean produced invalid geometry: " + error;
            return false;
        }
        currentBody = result;
        return true;
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE extrude boolean failure: ") +
            occt::failureMessage(failure);
        return false;
    }
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
            std::string profileError;
            // An unfinished sketch is valid persistent authoring state. Keep its
            // plane and diagnostics; only a consuming feature requires a face.
            occt::buildSketchFace(*params, frame, profile, profileError);
            states.emplace(feature.id, SketchBuildState{profile, frame, profileError});
            continue;
        }
        case FeatureType::Extrude: {
            const auto* params = std::get_if<ExtrudeFeatureParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match Extrude";
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
            if (sketchState.profile.IsNull()) {
                error = "Upstream sketch " + std::to_string(params->sketchId) +
                    " has no closed profile: " + sketchState.profileError;
                break;
            }
            occt::ExtrudeResult result;
            if (!occt::buildExtrude(
                    sketchState.profile, sketchState.frame, *params, result, error)) {
                break;
            }
            if (!applyExtrudeOperation(
                    *params, result.shape, currentBody, error)) {
                break;
            }
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
            if (sketchState.profile.IsNull()) {
                error = "Upstream sketch " + std::to_string(params->sketchId) +
                    " has no closed profile: " + sketchState.profileError;
                break;
            }
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
        case FeatureType::BoxPrimitive: {
            const auto* params = std::get_if<BoxPrimitiveParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match BoxPrimitive";
                break;
            }
            TopoDS_Shape box;
            if (!occt::buildBoxPrimitive(*params, box, error)) {
                break;
            }
            if (params->join && !currentBody.IsNull()) {
                ExtrudeFeatureParams operation;
                operation.operation = ExtrudeOperation::Join;
                if (!applyExtrudeOperation(operation, box, currentBody, error)) break;
            } else {
                appendIndependentSolid(box, currentBody);
            }
            states.emplace(feature.id, PrimitiveBuildState{feature.params});
            continue;
        }
        case FeatureType::ThroughHolePrimitive: {
            const auto* params =
                std::get_if<ThroughHolePrimitiveParams>(&feature.params);
            if (params == nullptr) {
                error = "Feature parameter type does not match ThroughHolePrimitive";
                break;
            }
            TopoDS_Shape cut;
            if (!occt::buildThroughHolePrimitive(
                    currentBody, *params, cut, error)) {
                break;
            }
            currentBody = cut;
            continue;
        }
        case FeatureType::SpherePrimitive:
        case FeatureType::CylinderPrimitive:
        case FeatureType::ConePrimitive:
        case FeatureType::TorusPrimitive: {
            TopoDS_Shape solid;
            const bool built = std::visit([&](const auto& params) {
                using P = std::decay_t<decltype(params)>;
                if constexpr (std::is_same_v<P, SpherePrimitiveParams> ||
                              std::is_same_v<P, CylinderPrimitiveParams> ||
                              std::is_same_v<P, ConePrimitiveParams> ||
                              std::is_same_v<P, TorusPrimitiveParams>) {
                    return occt::buildCurvedPrimitive(CurvedPrimitiveParams{params}, solid, error);
                }
                error = "Feature parameter type does not match curved solid";
                return false;
            }, feature.params);
            if (!built) break;
            appendIndependentSolid(solid, currentBody);
            if (feature.type == FeatureType::CylinderPrimitive)
                states.emplace(feature.id, PrimitiveBuildState{feature.params});
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
