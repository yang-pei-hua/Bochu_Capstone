#include "reconstruction/recognition/GuidedSolidRecognizer.h"
#include "reconstruction/recognition/BoxRecognizer.h"
#include "reconstruction/modeling/ModelingAdapter.h"
#include "common/Math3.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "detail/GuidedInternal.h"

namespace reconstruction::detail::guided {
GeomAbs_SurfaceType faceKind(TemplateKind kind)
{
    switch (kind) {
    case TemplateKind::Plane:
    case TemplateKind::Box:
        return GeomAbs_Plane;
    case TemplateKind::Cylinder:
        return GeomAbs_Cylinder;
    case TemplateKind::Sphere:
        return GeomAbs_Sphere;
    case TemplateKind::Cone:
        return GeomAbs_Cone;
    case TemplateKind::Torus:
        return GeomAbs_Torus;
    }
    return GeomAbs_OtherSurface;
}

bool verifyHints(const PointStore& points, const std::vector<TemplateHint>& hints,
                 const CompoundCandidate& candidate, const std::vector<TemplateFit>& fits,
                 double tolerance, int& failed, std::string& error,
                 const std::function<bool()>& cancelled)
{
    modeling::ModelingCore model;
    const auto built = commitCompoundCandidate(candidate, model);
    if (!built.success) {
        error = built.error;
        return false;
    }
    std::map<GeomAbs_SurfaceType, std::vector<TopoDS_Face>> faces;
    for (TopExp_Explorer e(model.bodyShape(), TopAbs_FACE); e.More(); e.Next()) {
        const auto face = TopoDS::Face(e.Current());
        faces[BRepAdaptor_Surface(face).GetType()].push_back(face);
    }
    for (std::size_t h = 0; h < hints.size(); ++h) {
        if (cancelled && cancelled()) {
            error = "Cancelled";
            return false;
        }
        std::set<PointId> selected(hints[h].points.begin(), hints[h].points.end());
        std::size_t total = 0, matched = 0;
        for (const auto& point : points.points())
            if (selected.count(point.id)) {
                if (cancelled && cancelled()) {
                    error = "Cancelled";
                    return false;
                }
                ++total;
                if (fits[h].solid &&
                    matchesCurvedBoundary(fits[h].solid->primitive, point, tolerance * 3)) {
                    ++matched;
                    continue;
                }
                const auto vertex =
                    BRepBuilderAPI_MakeVertex(
                        gp_Pnt(point.position.x, point.position.y, point.position.z))
                        .Vertex();
                for (const auto& face : faces[faceKind(hints[h].kind)]) {
                    BRepExtrema_DistShapeShape distance(vertex, face);
                    if (distance.IsDone() && distance.Value() <= tolerance * 3) {
                        ++matched;
                        break;
                    }
                }
            }
        if (!total || matched < total * 0.9) {
            failed = hints[h].regionId;
            error = "The complete CAD boundary conflicts with the selected template (support " +
                    std::to_string(matched) + "/" + std::to_string(total) + ")";
            return false;
        }
    }
    return true;
}
} // namespace reconstruction::detail::guided
