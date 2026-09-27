#include "RebuildEngine.h"

#include "modeling/PartDocument.h"
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

using DerivedFeatureState = std::variant<SketchBuildState, ExtrudeBuildState>;
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
    if (const auto* datum = std::get_if<DatumPlaneReference>(&reference)) {
        frame = occt::datumPlaneFrame(datum->plane);
        return true;
    }
    if (const auto* offset = std::get_if<OffsetDatumPlane>(&reference)) {
        if (!std::isfinite(offset->offset)) {
            error = "Offset datum plane distance must be finite";
            return false;
        }
        frame = occt::datumPlaneFrame(offset->base);
        frame = occt::translatedFrame(frame, frame.normal, offset->offset);
        return true;
    }
    if (const auto* plane = std::get_if<Plane3d>(&reference)) {
        constexpr double kDegenerate = 1.0e-12;
        const auto finite = [](const Vec3& value) {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                std::isfinite(value.z);
        };
        if (!finite(plane->origin) || !finite(plane->normal) ||
            !finite(plane->xDirection)) {
            error = "Explicit sketch plane must be finite and non-degenerate";
            return false;
        }

        const double normalLength = std::sqrt(
            plane->normal.x * plane->normal.x +
            plane->normal.y * plane->normal.y +
            plane->normal.z * plane->normal.z);
        if (normalLength <= kDegenerate) {
            error = "Explicit sketch plane must be finite and non-degenerate";
            return false;
        }
        const Vec3 normal{
            plane->normal.x / normalLength,
            plane->normal.y / normalLength,
            plane->normal.z / normalLength};

        const double projection =
            plane->xDirection.x * normal.x +
            plane->xDirection.y * normal.y +
            plane->xDirection.z * normal.z;
        Vec3 xDirection{
            plane->xDirection.x - projection * normal.x,
            plane->xDirection.y - projection * normal.y,
            plane->xDirection.z - projection * normal.z};
        const double xLength = std::sqrt(
            xDirection.x * xDirection.x +
            xDirection.y * xDirection.y +
            xDirection.z * xDirection.z);
        if (xLength <= kDegenerate) {
            error = "Explicit sketch plane must be finite and non-degenerate";
            return false;
        }
        xDirection = Vec3{
            xDirection.x / xLength,
            xDirection.y / xLength,
            xDirection.z / xLength};

        frame.origin = gp_Pnt(plane->origin.x, plane->origin.y, plane->origin.z);
        frame.normal = gp_Dir(normal.x, normal.y, normal.z);
        frame.xDirection = gp_Dir(xDirection.x, xDirection.y, xDirection.z);
        frame.yDirection = gp_Dir(
            normal.y * xDirection.z - normal.z * xDirection.y,
            normal.z * xDirection.x - normal.x * xDirection.z,
            normal.x * xDirection.y - normal.y * xDirection.x);
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
            appendIndependentSolid(box, currentBody);
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
