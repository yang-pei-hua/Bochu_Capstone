#include "modeling/SemanticFaces.h"
#include "modeling/PrimitiveEditing.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace modeling {
namespace {
Vec3 add(Vec3 a, Vec3 b, double factor = 1) {
    return {a.x + factor*b.x, a.y + factor*b.y, a.z + factor*b.z};
}
double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
bool finite(Vec3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
Vec3 unit(Vec3 a) {
    const double length = std::hypot(a.x,a.y,a.z);
    if (!finite(a) || !std::isfinite(length) || length <= 1e-12)
        throw std::runtime_error("Sketch plane axes must be finite and non-degenerate");
    return {a.x/length,a.y/length,a.z/length};
}
Plane3d normalized(Plane3d plane) {
    if (!finite(plane.origin)) throw std::runtime_error("Sketch plane origin must be finite");
    plane.normal = unit(plane.normal);
    plane.xDirection = unit(add(plane.xDirection,plane.normal,-dot(plane.xDirection,plane.normal)));
    return plane;
}
bool resolve(const std::vector<Feature>& features, const SketchPlaneReference& reference,
             Plane3d& plane, std::string& error, int depth) {
    if (depth > 32) { error = "ReferenceLost: cyclic or excessively deep plane reference"; return false; }
    if (const auto* explicitPlane = std::get_if<Plane3d>(&reference)) {
        plane = normalized(*explicitPlane); return true;
    }
    DatumPlane base = DatumPlane::XY;
    double distance = 0;
    if (const auto* datum = std::get_if<DatumPlaneReference>(&reference)) base = datum->plane;
    else if (const auto* offset = std::get_if<OffsetDatumPlane>(&reference)) {
        base = offset->base; distance = offset->offset;
        if (!std::isfinite(distance)) { error = "Offset datum plane distance must be finite"; return false; }
    } else {
        const auto& face = std::get<FaceReference>(reference);
        const auto owner = std::find_if(features.begin(),features.end(),[&](const Feature& f) { return f.id == face.ownerFeature; });
        if (owner == features.end() || owner->suppressed || !owner->valid) {
            error = "ReferenceLost: owner feature " + std::to_string(face.ownerFeature) + " has no available geometry";
            return false;
        }
        if (const auto* extrude = std::get_if<ExtrudeFeatureParams>(&owner->params)) {
            if (face.role != FaceRole::StartFace && face.role != FaceRole::EndFace) {
                error = "ReferenceLost: unsupported extrusion face role"; return false;
            }
            const auto sketch = std::find_if(features.begin(),owner,[&](const Feature& f) { return f.id == extrude->sketchId; });
            if (sketch == owner || sketch->suppressed || !sketch->valid) {
                error = "ReferenceLost: extrusion has no available upstream sketch"; return false;
            }
            const auto* params = std::get_if<SketchFeatureParams>(&sketch->params);
            if (!params || !resolve(features,params->plane,plane,error,depth+1)) return false;
            const auto direction = add(Vec3{},plane.normal,extrude->reverse ? -1 : 1);
            if (face.role == FaceRole::EndFace) plane.origin = add(plane.origin,direction,extrude->depth);
            plane.normal = face.role == FaceRole::EndFace ? direction : add(Vec3{},direction,-1);
            return true;
        }
        return primitiveFacePlane(owner->params,face.role,plane,error);
    }
    if (base == DatumPlane::XY) plane = {{},{0,0,1},{1,0,0}};
    else if (base == DatumPlane::YZ) plane = {{},{1,0,0},{0,1,0}};
    else if (base == DatumPlane::XZ) plane = {{},{0,-1,0},{1,0,0}};
    else { error = "Invalid datum plane"; return false; }
    plane.origin = add(plane.origin,plane.normal,distance);
    return true;
}
}

const char* faceRoleName(FaceRole role) {
    switch (role) {
    case FaceRole::StartFace: return "Start";
    case FaceRole::EndFace: return "End";
    case FaceRole::BoxXMin: return "Box -X";
    case FaceRole::BoxXMax: return "Box +X";
    case FaceRole::BoxYMin: return "Box -Y";
    case FaceRole::BoxYMax: return "Box +Y";
    case FaceRole::BoxZMin: return "Box -Z";
    case FaceRole::BoxZMax: return "Box +Z";
    case FaceRole::CylinderBase: return "Cylinder base";
    case FaceRole::CylinderTop: return "Cylinder top";
    default: return "Unknown";
    }
}

bool primitiveFacePlane(const FeatureParams& params, FaceRole role, Plane3d& plane, std::string& error) {
    try {
        if (const auto* box = std::get_if<BoxPrimitiveParams>(&params)) {
            if (role < FaceRole::BoxXMin || role > FaceRole::BoxZMax) {
                error = "ReferenceLost: unsupported box face role"; return false;
            }
            const auto axes = boxAxes(*box);
            const double sizes[3]{box->sizeX,box->sizeY,box->sizeZ};
            const int side = static_cast<int>(role)-static_cast<int>(FaceRole::BoxXMin);
            const int axis = side/2;
            plane = {add(box->pose.origin,axes[axis],side%2 ? sizes[axis] : 0),
                     add(Vec3{},axes[axis],side%2 ? 1 : -1),axes[(axis+1)%3]};
        } else if (const auto* cylinder = std::get_if<CylinderPrimitiveParams>(&params)) {
            if (role != FaceRole::CylinderBase && role != FaceRole::CylinderTop) {
                error = "ReferenceLost: unsupported cylinder face role"; return false;
            }
            const auto axis = unit(cylinder->axisDirection);
            const auto tangent = unit(cross(std::abs(axis.z) < .9 ? Vec3{0,0,1} : Vec3{1,0,0},axis));
            plane = {add(cylinder->baseCenter,axis,role == FaceRole::CylinderTop ? cylinder->height : 0),
                     add(Vec3{},axis,role == FaceRole::CylinderTop ? 1 : -1),tangent};
        } else { error = "ReferenceLost: owner does not support primitive face references"; return false; }
        plane = normalized(plane); error.clear(); return true;
    } catch (const std::exception& failure) { error = failure.what(); return false; }
}

bool resolveSketchPlane(const std::vector<Feature>& features, const SketchPlaneReference& reference,
                        Plane3d& plane, std::string& error) {
    try { error.clear(); return resolve(features,reference,plane,error,0); }
    catch (const std::exception& failure) { error = failure.what(); return false; }
}

std::optional<FaceReference> identifyPrimitiveFace(const std::vector<Feature>& features,
    const Plane3d& pickedPlane, Vec3 point, std::string& error, double tolerance) {
    std::optional<FaceReference> found;
    try {
        const auto picked = normalized(pickedPlane);
        if (!finite(point) || !std::isfinite(tolerance) || tolerance <= 0) {
            error = "Invalid face pick"; return {};
        }
        for (const auto& feature : features) {
            if (feature.suppressed || !feature.valid) continue;
            const auto* box = std::get_if<BoxPrimitiveParams>(&feature.params);
            const auto* cylinder = std::get_if<CylinderPrimitiveParams>(&feature.params);
            if (!box && !cylinder) continue;
            const int first = static_cast<int>(box ? FaceRole::BoxXMin : FaceRole::CylinderBase);
            const int last = static_cast<int>(box ? FaceRole::BoxZMax : FaceRole::CylinderTop);
            for (int role = first; role <= last; ++role) {
                Plane3d candidate; std::string ignored;
                if (!primitiveFacePlane(feature.params,static_cast<FaceRole>(role),candidate,ignored)) continue;
                if (dot(candidate.normal,picked.normal) < 1-1e-9
                    || std::abs(dot(add(picked.origin,candidate.origin,-1),candidate.normal)) > tolerance
                    || std::abs(dot(add(point,candidate.origin,-1),candidate.normal)) > tolerance) continue;
                bool contains = true;
                if (box) {
                    const auto axes = boxAxes(*box);
                    const double sizes[3]{box->sizeX,box->sizeY,box->sizeZ};
                    for (int a = 0; a < 3; ++a) {
                        const double value = dot(add(point,box->pose.origin,-1),axes[a]);
                        contains = contains && value >= -tolerance && value <= sizes[a]+tolerance;
                    }
                } else {
                    const auto offset = add(point,candidate.origin,-1);
                    contains = std::hypot(offset.x,offset.y,offset.z) <= cylinder->radius+tolerance;
                }
                if (!contains) continue;
                if (found) { error = "Ambiguous coplanar primitive owners; using a fixed plane"; return {}; }
                found = std::get<FaceReference>(featureFace(feature.id,static_cast<FaceRole>(role)));
            }
        }
        error = found ? "" : "No supported primitive face; using a fixed plane";
        return found;
    } catch (const std::exception& failure) { error = failure.what(); return {}; }
}
} // namespace modeling
