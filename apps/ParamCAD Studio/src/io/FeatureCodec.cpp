#include "io/FeatureCodec.h"
#include <QJsonObject>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

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
                             static_cast<FaceRole>(integer(json["role"], 2)),
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
                return {{"values", values({p.center.x, p.center.y, p.center.z, p.radius})}};
            else if constexpr (std::is_same_v<P, CylinderPrimitiveParams>)
                return {{"values",
                         values({p.baseCenter.x, p.baseCenter.y, p.baseCenter.z, p.axisDirection.x,
                                 p.axisDirection.y, p.axisDirection.z, p.radius, p.height})}};
            else if constexpr (std::is_same_v<P, ConePrimitiveParams>)
                return {{"values", values({p.baseCenter.x, p.baseCenter.y, p.baseCenter.z,
                                           p.axisDirection.x, p.axisDirection.y, p.axisDirection.z,
                                           p.baseRadius, p.topRadius, p.height})}};
            else
                return {{"values", values({p.center.x, p.center.y, p.center.z, p.axisDirection.x,
                                           p.axisDirection.y, p.axisDirection.z, p.majorRadius,
                                           p.minorRadius})}};
        },
        params);
    json["kind"] = static_cast<int>(params.index());
    return json;
}
FeatureParams paramsFromJson(const QJsonObject& p)
{
    switch (integer(p["kind"], 8)) {
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
        return SpherePrimitiveParams{vec(v, 0), v[3]};
    }
    case 6: {
        const auto v = numbers(p["values"], 8);
        return CylinderPrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7]};
    }
    case 7: {
        const auto v = numbers(p["values"], 9);
        return ConePrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7], v[8]};
    }
    default: {
        const auto v = numbers(p["values"], 8);
        return TorusPrimitiveParams{vec(v, 0), vec(v, 3), v[6], v[7]};
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
            result.push_back({id(feature["id"]),
                              feature["name"].toString().toStdString(),
                              modeling::featureTypeOf(params),
                              std::move(params),
                              boolean(feature["suppressed"]),
                              true,
                              {}, feature.contains("reconstructed") ? boolean(feature["reconstructed"]) : false});
        }
        features = std::move(result);
        error.clear();
        return true;
    } catch (const std::exception& failure) {
        error = QString::fromUtf8(failure.what());
        return false;
    }
}
