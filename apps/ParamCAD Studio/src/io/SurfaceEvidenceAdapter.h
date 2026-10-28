#pragma once
#include "core/PointCloudSelection.h"
#include <reconstruction/analysis/SurfaceAnalysis.h>
#include <QJsonArray>
#include <cmath>

// Only the application edge knows JSON/region IDs. The geometry core receives
// typed, validated equations in the same units as its PointStore snapshot.
inline std::optional<reconstruction::PrimitiveEvidence> regionEvidence(const PointCloudRegion& region)
{
    using namespace reconstruction;
    if (region.fitStale || region.parameters.isEmpty()) return {};
    const auto& p = region.parameters;
    auto scalar = [&](const char* key) { return p[key].isDouble() ? p[key].toDouble() : NAN; };
    auto vector = [&](const char* key) -> modeling::Vec3 {
        const auto a = p[key].toArray();
        if (a.size()!=3 || !a[0].isDouble() || !a[1].isDouble() || !a[2].isDouble()) return {NAN,NAN,NAN};
        return {a[0].toDouble(),a[1].toDouble(),a[2].toDouble()};
    };
    auto finite = [](modeling::Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); };
    auto unit = [&](const char* key) -> std::optional<modeling::Vec3> {
        auto v = vector(key); const double len = std::hypot(v.x,v.y,v.z);
        if (!finite(v) || len < 1e-12) return {};
        return modeling::Vec3{v.x/len,v.y/len,v.z/len};
    };
    auto finish = [&](auto e) -> std::optional<PrimitiveEvidence> {
        e.id = region.id; e.confidence = region.confidence; e.meanAbsoluteResidual = region.residual;
        return PrimitiveEvidence{e};
    };
    if (region.geometry == "plane") {
        const auto n = unit("normal"); const double offset=scalar("offset");
        if (!n || !std::isfinite(offset)) return {};
        const auto raw=vector("normal");
        PlaneEvidence e; e.plane={*n,offset/std::hypot(raw.x,raw.y,raw.z)}; return finish(e);
    }
    const auto c=vector("center"); if (!finite(c)) return {};
    if (region.geometry == "sphere") {
        const double r=scalar("radius"); if (!(r>0) || !std::isfinite(r)) return {};
        SphereEvidence e; e.sphere={c,r}; return finish(e);
    }
    const auto a=unit("axis"); if (!a) return {};
    if (region.geometry == "cylinder") {
        const double r=scalar("radius"); if (!(r>0) || !std::isfinite(r)) return {};
        CylinderEvidence e; e.cylinder={c,*a,r};
        e.axialMinimum=scalar("axial_min"); e.axialMaximum=scalar("axial_max"); return finish(e);
    }
    if (region.geometry == "cone") {
        const double angle=scalar("angle"); if (!(angle>0 && angle<1.5707963267948966)) return {};
        ConeEvidence e; e.cone={c,*a,angle}; return finish(e);
    }
    if (region.geometry == "ring") {
        const double major=scalar("major_radius"), minor=scalar("minor_radius");
        if (!(major>minor && minor>0) || !std::isfinite(major)) return {};
        TorusEvidence e; e.torus={c,*a,major,minor}; return finish(e);
    }
    return {};
}
