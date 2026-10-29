#include "io/FeatureCodec.h"
#include <QJsonObject>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <map>

namespace {
using namespace modeling;
void check(bool value)
{
    if (!value)
        throw std::runtime_error("Invalid project feature data");
}
QJsonArray values(std::initializer_list<double> numbers)
{
    QJsonArray result;
    for (double number : numbers) {
        check(std::isfinite(number));
        result.append(number);
    }
    return result;
}
std::vector<double> numbers(const QJsonValue& value, size_t count)
{
    check(value.isArray());
    const auto array = value.toArray();
    check(static_cast<size_t>(array.size()) == count);
    std::vector<double> result;
    for (const auto& item : array) {
        check(item.isDouble() && std::isfinite(item.toDouble()));
        result.push_back(item.toDouble());
    }
    return result;
}
int integer(const QJsonValue& value, int maximum)
{
    check(value.isDouble());
    const double number = value.toDouble();
    check(std::isfinite(number) && number >= 0 && number <= maximum &&
          std::floor(number) == number);
    return static_cast<int>(number);
}
bool boolean(const QJsonValue& value)
{
    check(value.isBool());
    return value.toBool();
}
FeatureId id(const QJsonValue& value)
{
    check(value.isString());
    bool ok = false;
    const auto result = value.toString().toULongLong(&ok);
    check(ok && result != 0 && result != std::numeric_limits<FeatureId>::max());
    return result;
}
Vec3 vec(const std::vector<double>& v, size_t start)
{
    return {v[start], v[start + 1], v[start + 2]};
}
template<class T> QJsonArray sequence(const std::vector<T>& list){QJsonArray a;for(auto x:list)a.append(x);return a;}
QJsonArray polesJson(const std::vector<Vec3>& list){QJsonArray a;for(auto p:list)a.append(values({p.x,p.y,p.z}));return a;}
std::vector<Vec3> polesFromJson(const QJsonValue& value){check(value.isArray());std::vector<Vec3> result;for(auto p:value.toArray())result.push_back(vec(numbers(p,3),0));return result;}
std::vector<double> reals(const QJsonValue& value){check(value.isArray());return numbers(value,value.toArray().size());}
std::vector<int> integers(const QJsonValue& value){check(value.isArray());std::vector<int> result;for(auto p:value.toArray())result.push_back(integer(p,1000000));return result;}
QJsonObject boundaryJson(const BoundaryBody& body) {
    QJsonArray faces;
    for(const auto& face:body.faces) {
        const auto& s=face.surface;
        QJsonObject surface{{"kind",int(s.kind)},{"frame",values({s.pose.origin.x,s.pose.origin.y,s.pose.origin.z,s.pose.xDirection.x,s.pose.xDirection.y,s.pose.xDirection.z,s.pose.zDirection.x,s.pose.zDirection.y,s.pose.zDirection.z})},
            {"radius",s.radius},{"minorRadius",s.minorRadius},{"semiAngle",s.semiAngle},
            {"uDegree",s.uDegree},{"vDegree",s.vDegree},{"uCount",s.uCount},{"vCount",s.vCount},
            {"poles",polesJson(s.poles)},{"weights",sequence(s.weights)},{"uKnots",sequence(s.uKnots)},{"vKnots",sequence(s.vKnots)},
            {"uMultiplicities",sequence(s.uMultiplicities)},{"vMultiplicities",sequence(s.vMultiplicities)}};
        QJsonArray wires;for(const auto& loop:face.wires){QJsonArray wire;for(const auto& edge:loop){const auto& c=edge.curve;
            wire.append(QJsonObject{{"edgeId",edge.edgeId},{"reversed",edge.reversed},{"degree",c.degree},{"poles",polesJson(c.poles)},
                {"weights",sequence(c.weights)},{"knots",sequence(c.knots)},{"multiplicities",sequence(c.multiplicities)}});}wires.append(wire);}
        faces.append(QJsonObject{{"surface",surface},{"reversed",face.reversed},{"wires",wires}});
    }
    return {{"faces",faces},{"edge_identity_scope","body"},{"tolerance",body.tolerance}};
}
BoundaryBody boundaryFromJson(const QJsonValue& value) {
    check(value.isObject());const auto object=value.toObject();check(object["faces"].isArray());BoundaryBody body;
    if(object.contains("tolerance")){check(object["tolerance"].isDouble());body.tolerance=object["tolerance"].toDouble();}
    const auto scope=object["edge_identity_scope"].toString("face");check(scope=="face" || scope=="body");
    std::map<std::pair<size_t,int>,int> legacyEdges;
    for(auto item:object["faces"].toArray()) {
        check(item.isObject());const auto f=item.toObject();check(f["surface"].isObject());const auto s=f["surface"].toObject();BoundaryFace face;
        face.reversed=boolean(f["reversed"]);auto& surface=face.surface;surface.kind=static_cast<BoundarySurface::Kind>(integer(s["kind"],5));
        const auto frame=numbers(s["frame"],9);surface.pose={vec(frame,0),vec(frame,3),vec(frame,6)};
        const auto dimensions=numbers(QJsonArray{s["radius"],s["minorRadius"],s["semiAngle"]},3);surface.radius=dimensions[0];surface.minorRadius=dimensions[1];surface.semiAngle=dimensions[2];
        surface.uDegree=integer(s["uDegree"],25);surface.vDegree=integer(s["vDegree"],25);surface.uCount=integer(s["uCount"],100000);surface.vCount=integer(s["vCount"],100000);
        surface.poles=polesFromJson(s["poles"]);surface.weights=reals(s["weights"]);surface.uKnots=reals(s["uKnots"]);surface.vKnots=reals(s["vKnots"]);
        surface.uMultiplicities=integers(s["uMultiplicities"]);surface.vMultiplicities=integers(s["vMultiplicities"]);
        check(f["wires"].isArray());for(auto loop:f["wires"].toArray()){check(loop.isArray());std::vector<BoundaryEdgeUse> wire;
            for(auto use:loop.toArray()){check(use.isObject());const auto edge=use.toObject();BoundaryEdgeUse e;e.edgeId=integer(edge["edgeId"],1000000);e.reversed=boolean(edge["reversed"]);
                if(scope=="face"){const auto key=std::make_pair(body.faces.size(),e.edgeId);const auto found=legacyEdges.find(key);
                    if(found==legacyEdges.end()){const int next=int(legacyEdges.size()+1);legacyEdges[key]=next;e.edgeId=next;}else e.edgeId=found->second;}
                e.curve.degree=integer(edge["degree"],25);e.curve.poles=polesFromJson(edge["poles"]);e.curve.weights=reals(edge["weights"]);e.curve.knots=reals(edge["knots"]);e.curve.multiplicities=integers(edge["multiplicities"]);wire.push_back(std::move(e));}
            face.wires.push_back(std::move(wire));}
        body.faces.push_back(std::move(face));
    }
    std::string error;check(validateBoundaryBody(body,error));return body;
}
QJsonObject planeJson(const SketchPlaneReference& plane)
{
    auto result = std::visit(
        [](const auto& p) -> QJsonObject {
            using P = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<P, DatumPlaneReference>)
                return {{"base", static_cast<int>(p.plane)}};
            else if constexpr (std::is_same_v<P, OffsetDatumPlane>)
                return {{"base", static_cast<int>(p.base)}, {"values", values({p.offset})}};
            else if constexpr (std::is_same_v<P, FaceReference>)
                return {{"owner", QString::number(p.ownerFeature)},
                        {"role", static_cast<int>(p.role)},
                        {"hint", p.transientFaceIndexHint},
                        {"values", values({p.normal.x, p.normal.y, p.normal.z, p.centroid.x,
                                           p.centroid.y, p.centroid.z, p.area})}};
            else
                return {{"values",
                         values({p.origin.x, p.origin.y, p.origin.z, p.normal.x, p.normal.y,
                                 p.normal.z, p.xDirection.x, p.xDirection.y, p.xDirection.z})}};
        },
        plane);
    result["kind"] = static_cast<int>(plane.index());
    return result;
}
SketchPlaneReference planeFromJson(const QJsonObject& json)
{
    const int kind = integer(json["kind"], 3);
    if (kind == 0)
        return DatumPlaneReference{static_cast<DatumPlane>(integer(json["base"], 2))};
    if (kind == 1)
        return OffsetDatumPlane{static_cast<DatumPlane>(integer(json["base"], 2)),
                                numbers(json["values"], 1)[0]};
    if (kind == 2) {
        const auto v = numbers(json["values"], 7);
        check(json["hint"].isDouble() && json["hint"].toDouble() >= -1);
        const int hint =
            integer(QJsonValue(json["hint"].toDouble() + 1), std::numeric_limits<int>::max() - 1) -
            1;
        return FaceReference{id(json["owner"]),
                             static_cast<FaceRole>(integer(json["role"], static_cast<int>(FaceRole::CylinderTop))),
                             vec(v, 0),
                             vec(v, 3),
                             v[6],
                             hint};
    }
    const auto v = numbers(json["values"], 9);
    return Plane3d{vec(v, 0), vec(v, 3), vec(v, 6)};
}
QJsonObject paramsJson(const FeatureParams& params)
{
    auto json = std::visit(
        [](const auto& p) -> QJsonObject {
            using P = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<P, SketchFeatureParams>) {
                QJsonArray entities;
                for (const auto& entity : p.entities) {
                    const auto data = std::visit(
                        [](const auto& g) -> QJsonArray {
                            using G = std::decay_t<decltype(g)>;
                            if constexpr (std::is_same_v<G, Point2D>)
                                return values({g.x, g.y});
                            else if constexpr (std::is_same_v<G, Line2D>)
                                return values({g.x1, g.y1, g.x2, g.y2});
                            else if constexpr (std::is_same_v<G, Rectangle2D>)
                                return values({g.x, g.y, g.width, g.height});
                            else
                                return values({g.x, g.y, g.radius});
                        },
                        entity.geometry);
                    entities.append(QJsonObject{{"id", QString::number(entity.id)},
                                                {"kind", static_cast<int>(entity.geometry.index())},
                                                {"values", data}});
                }
                return {{"plane", planeJson(p.plane)}, {"entities", entities}};
            } else if constexpr (std::is_same_v<P, ExtrudeFeatureParams>)
                return {{"sketch", QString::number(p.sketchId)},
                        {"values", values({p.depth})},
                        {"reverse", p.reverse},
                        {"operation", static_cast<int>(p.operation)}};
            else if constexpr (std::is_same_v<P, CutFeatureParams>)
                return {{"sketch", QString::number(p.sketchId)},
                        {"values", values({p.depth})},
                        {"reverse", p.reverse},
                        {"throughAll", p.throughAll}};
            else if constexpr (std::is_same_v<P, BoxPrimitiveParams>)
                return {{"join", p.join},
                        {"values",
                         values({p.pose.origin.x, p.pose.origin.y, p.pose.origin.z,
                                 p.pose.xDirection.x, p.pose.xDirection.y, p.pose.xDirection.z,
                                 p.pose.zDirection.x, p.pose.zDirection.y, p.pose.zDirection.z,
                                 p.sizeX, p.sizeY, p.sizeZ})}};
            else if constexpr (std::is_same_v<P, ThroughHolePrimitiveParams>)
                return {{"values",
                         values({p.axisPoint.x, p.axisPoint.y, p.axisPoint.z, p.axisDirection.x,
                                 p.axisDirection.y, p.axisDirection.z, p.radius})}};
            else if constexpr (std::is_same_v<P, SpherePrimitiveParams>)
                return {{"join", p.join}, {"values", values({p.center.x, p.center.y, p.center.z, p.radius})}};
            else if constexpr (std::is_same_v<P, CylinderPrimitiveParams>)
                return {{"join", p.join}, {"values",
                         values({p.baseCenter.x, p.baseCenter.y, p.baseCenter.z, p.axisDirection.x,
                                 p.axisDirection.y, p.axisDirection.z, p.radius, p.height})}};
            else if constexpr (std::is_same_v<P, ConePrimitiveParams>)
                return {{"join", p.join}, {"values", values({p.baseCenter.x, p.baseCenter.y, p.baseCenter.z,
                                           p.axisDirection.x, p.axisDirection.y, p.axisDirection.z,
                                           p.baseRadius, p.topRadius, p.height})}};
            else if constexpr (std::is_same_v<P, TorusPrimitiveParams>)
                return {{"join", p.join}, {"values", values({p.center.x, p.center.y, p.center.z, p.axisDirection.x,
                                           p.axisDirection.y, p.axisDirection.z, p.majorRadius,
                                           p.minorRadius})}};
            else {
                QJsonArray envelope,cavities,voids;
                auto curves=[](const auto& list){QJsonArray result;for(const auto& c:list)result.append(QJsonObject{{"circular",c.circular},{"values",values({c.x1,c.y1,c.x2,c.y2,c.centerX,c.centerY,c.radius,c.startAngle,c.sweepAngle})}});return result;};
                for(const auto& loop:p.profileVoids)voids.append(curves(loop));
                auto sections=[](const auto& list,QJsonArray& array){for(const auto& s:list)array.append(values({s.centerX,s.centerY,s.start,s.end,s.startRadius,s.endRadius,s.innerStartRadius,s.innerEndRadius,s.centerZ,s.axis.x,s.axis.y,s.axis.z}));};
                sections(p.envelope,envelope);sections(p.cavities,cavities);
                QJsonObject result{{"values",values({p.pose.origin.x,p.pose.origin.y,p.pose.origin.z,p.pose.xDirection.x,p.pose.xDirection.y,p.pose.xDirection.z,
                    p.pose.zDirection.x,p.pose.zDirection.y,p.pose.zDirection.z,p.depth,p.referenceSizeX,p.referenceSizeY,p.sizeX,p.sizeY,p.sizeZ})},
                    {"outline",curves(p.outline)},{"profileVoids",voids},{"envelope",envelope},{"cavities",cavities}};
                if(p.boundary)result["boundary"]=boundaryJson(*p.boundary);
                return result;
            }
        },
        params);
    json["kind"] = static_cast<int>(params.index());
    return json;
}
FeatureParams paramsFromJson(const QJsonObject& p)
{
    switch (integer(p["kind"], 9)) {
    case 0: {
        check(p["plane"].isObject() && p["entities"].isArray());
        SketchFeatureParams sketch;
        sketch.plane = planeFromJson(p["plane"].toObject());
        for (const auto& value : p["entities"].toArray()) {
            check(value.isObject());
            const auto entity = value.toObject();
            const int kind = integer(entity["kind"], 3);
            const auto v = numbers(entity["values"], kind == 0 ? 2 : kind == 3 ? 3 : 4);
            SketchGeometry geometry;
            if (kind == 0)
                geometry = Point2D{v[0], v[1]};
            else if (kind == 1)
                geometry = Line2D{v[0], v[1], v[2], v[3]};
            else if (kind == 2)
                geometry = Rectangle2D{v[0], v[1], v[2], v[3]};
            else
                geometry = Circle2D{v[0], v[1], v[2]};
            sketch.entities.push_back({id(entity["id"]), geometry});
        }
        return sketch;
    }
    case 1:
        return ExtrudeFeatureParams{id(p["sketch"]), numbers(p["values"], 1)[0],
                                    boolean(p["reverse"]),
                                    static_cast<ExtrudeOperation>(integer(p["operation"], 3))};
    case 2:
        return CutFeatureParams{id(p["sketch"]), numbers(p["values"], 1)[0],
                                boolean(p["throughAll"]), boolean(p["reverse"])};
    case 3: {
        const auto v = numbers(p["values"], 12);
        return BoxPrimitiveParams{
            {vec(v, 0), vec(v, 3), vec(v, 6)}, v[9], v[10], v[11], boolean(p["join"])};
    }
    case 4: {
        const auto v = numbers(p["values"], 7);
        return ThroughHolePrimitiveParams{vec(v, 0), vec(v, 3), v[6]};
    }
    case 5: {
        const auto v = numbers(p["values"], 4);
        return SpherePrimitiveParams{vec(v, 0), v[3], p.contains("join") ? boolean(p["join"]) : false};
    }
    case 6: {
        const auto v = numbers(p["values"], 8);
        return CylinderPrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7], p.contains("join") ? boolean(p["join"]) : false};
    }
    case 7: {
        const auto v = numbers(p["values"], 9);
        return ConePrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7], v[8], p.contains("join") ? boolean(p["join"]) : false};
    }
    case 8: {
        const auto v = numbers(p["values"], 8);
        return TorusPrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7], p.contains("join") ? boolean(p["join"]) : false};
    }
    default: {
        const auto v=numbers(p["values"],15);ProfileBodyParams body;
        body.pose={vec(v,0),vec(v,3),vec(v,6)};body.depth=v[9];body.referenceSizeX=v[10];body.referenceSizeY=v[11];body.sizeX=v[12];body.sizeY=v[13];body.sizeZ=v[14];
        check(p["outline"].isArray()&&p["envelope"].isArray()&&p["cavities"].isArray());
        auto curves=[](const QJsonValue& value){check(value.isArray());std::vector<ProfileCurve2D> result;
            for(auto item:value.toArray()){check(item.isObject());const auto curve=item.toObject();const auto c=numbers(curve["values"],9);
                result.push_back({boolean(curve["circular"]),c[0],c[1],c[2],c[3],c[4],c[5],c[6],c[7],c[8]});}return result;};
        body.outline=curves(p["outline"]);
        if(p.contains("profileVoids")){check(p["profileVoids"].isArray());for(auto loop:p["profileVoids"].toArray())body.profileVoids.push_back(curves(loop));}
        auto sections=[&](const char* key,auto& output){for(auto value:p[key].toArray()){
            check(value.isArray());const auto n=value.toArray().size();check(n==8 || n==12);const auto s=numbers(value,int(n));
            AxialSection section{s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7]};
            if(n==12){section.centerZ=s[8];section.axis=vec(s,9);}output.push_back(section);}};
        sections("envelope",body.envelope);sections("cavities",body.cavities);
        if(p.contains("boundary"))body.boundary=boundaryFromJson(p["boundary"]);
        std::string error;check(validateProfileBody(body,error));return body;
    }
    }
}
} // namespace

QJsonArray FeatureCodec::encode(const std::vector<modeling::Feature>& features)
{
    QJsonArray result;
    for (const auto& feature : features)
        result.append(QJsonObject{{"id", QString::number(feature.id)},
                                  {"name", QString::fromStdString(feature.name)},
                                  {"suppressed", feature.suppressed},
                                  {"reconstructed", feature.reconstructed},
                                  {"manuallyEdited", feature.manuallyEdited},
                                  {"resizeAnchor", static_cast<int>(feature.resizeAnchor)},
                                  {"params", paramsJson(feature.params)}});
    return result;
}
bool FeatureCodec::decode(const QJsonArray& json, std::vector<modeling::Feature>& features,
                          QString& error)
{
    try {
        std::vector<modeling::Feature> result;
        for (const auto& item : json) {
            check(item.isObject());
            const auto feature = item.toObject();
            check(feature["params"].isObject() && feature["name"].isString());
            auto params = paramsFromJson(feature["params"].toObject());
            const auto anchor = feature.contains("resizeAnchor")
                ? static_cast<ResizeAnchor>(integer(feature["resizeAnchor"],8))
                : std::holds_alternative<CylinderPrimitiveParams>(params) ? ResizeAnchor::Base : ResizeAnchor::Center;
            result.push_back({id(feature["id"]),
                              feature["name"].toString().toStdString(),
                              modeling::featureTypeOf(params),
                              std::move(params),
                              boolean(feature["suppressed"]),
                              true,
                              {}, feature.contains("reconstructed") ? boolean(feature["reconstructed"]) : false,
                              feature.contains("manuallyEdited") ? boolean(feature["manuallyEdited"]) : false,
                              anchor});
        }
        features = std::move(result);
        error.clear();
        return true;
    } catch (const std::exception& failure) {
        error = QString::fromUtf8(failure.what());
        return false;
    }
}
