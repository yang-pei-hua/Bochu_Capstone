#include "ProjectionInternal.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <algorithm>
#include <cmath>

namespace reconstruction::detail::projection {
double volume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

std::optional<Fit> scoreShape(const TopoDS_Shape& body, const std::vector<PointSample>& observations,
                            double tolerance, std::size_t operations, std::size_t curves,
                            const std::function<bool()>& cancelled) {
    if (body.IsNull() || observations.empty() || !std::isfinite(tolerance) || tolerance <= 0) return {};
    TopoDS_Compound faces;
    BRep_Builder builder;
    builder.MakeCompound(faces);
    NCollection_IndexedMap<TopoDS_Shape,TopTools_ShapeMapHasher> faceIndex;
    TopExp::MapShapes(body, TopAbs_FACE, faceIndex);
    for (int i = 1; i <= faceIndex.Extent(); ++i) builder.Add(faces, faceIndex(i));
    std::vector<std::size_t> support(faceIndex.Extent()+1);
    Fit fit;
    fit.volume = volume(body);
    std::vector<double> distances;
    double loss = 0, normalError = 0;
    BRepExtrema_DistShapeShape distance;
    distance.LoadS2(faces);
    for (const auto& p : observations) {
        if (cancelled && distances.size()%64 == 0 && cancelled()) return {};
        const auto vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(p.position.x,p.position.y,p.position.z)).Vertex();
        distance.LoadS1(vertex);
        distance.Perform();
        if (!distance.IsDone() || !distance.NbSolution()) return {};
        const double error = distance.Value();
        distances.push_back(error);
        loss += std::min(error/tolerance, 8.0);
        if (error > tolerance) fit.residuals.push_back(p.id);
        if (distance.SupportTypeShape2(1) != BRepExtrema_IsInFace) continue;
        const auto face = TopoDS::Face(distance.SupportOnShape2(1));
        const auto faceId = faceIndex.FindIndex(face);
        if (!p.normal) {
            if (error <= tolerance) ++support[faceId];
            continue;
        }
        double u, v;
        distance.ParOnFaceS2(1, u, v);
        gp_Pnt center;
        gp_Vec du, dv;
        BRepAdaptor_Surface(face).D1(u, v, center, du, dv);
        const auto crossProduct = du.Crossed(dv);
        if (crossProduct.SquareMagnitude() < 1e-24) continue;
        const auto normal = crossProduct.Normalized();
        double alignment = dot(*p.normal, {normal.X(),normal.Y(),normal.Z()});
        if (face.Orientation() == TopAbs_REVERSED) alignment = -alignment;
        if (!p.normalOrientationKnown) alignment = std::abs(alignment);
        alignment = std::clamp(alignment, -1.0, 1.0);
        if (error <= tolerance && alignment >= .8) ++support[faceId];
        normalError += std::acos(alignment)*180/pi;
        loss += .5*(1-alignment);
        ++fit.quality.normalComparisons;
    }
    std::sort(distances.begin(), distances.end());
    const auto count = distances.size();
    fit.quality.scoredPoints = count;
    fit.quality.medianDistance = distances[count/2];
    fit.quality.p95Distance = distances[std::size_t(.95*(count-1))];
    fit.quality.surfaceInlierFraction = 1-double(fit.residuals.size())/count;
    if (fit.quality.normalComparisons)
        fit.quality.meanNormalErrorDegrees = normalError/fit.quality.normalComparisons;
    double area = 0, unsupported = 0;
    for (int i = 1; i <= faceIndex.Extent(); ++i) {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(faceIndex(i),properties);
        area += properties.Mass();
        if (support[i] < 3) unsupported += properties.Mass();
    }
    fit.quality.unsupportedFaceAreaFraction = area > 0 ? unsupported/area : 1;
    fit.quality.score = loss/count + fit.quality.unsupportedFaceAreaFraction +
        .015*operations + .001*curves;
    return fit;
}

} // namespace reconstruction::detail::projection
