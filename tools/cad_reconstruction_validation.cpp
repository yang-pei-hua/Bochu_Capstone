// Repository-owned validation: generate exact STEP fixtures, then exercise the
// same cloud -> semantic features -> ModelingCore path as the desktop client.
#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"
#include "io/StepModelLoader.h"
#include "StepCloudSampler.h"
#include "SurfaceCoverageValidation.h"
#include "PointCloudCadValidation.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <gp_Ax2.hxx>
#include <gp_Ax1.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <fstream>
#include <iostream>
#include <functional>
#include <limits>
#include <memory>
#include <modeling/StepExporter.h>
#include <random>
#include <reconstruction/modeling/ModelingAdapter.h>
#include <reconstruction/detection/PrimitiveDetector.h>
#include <reconstruction/recognition/ThroughHoleRecognizer.h>
#include <stdexcept>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#endif

namespace {
#ifdef _WIN32
QString nativeCrashPath;
LONG WINAPI recordNativeCrash(EXCEPTION_POINTERS *exception) {
    QJsonArray frames;
    const auto process = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
    if (SymInitialize(process, nullptr, TRUE)) {
        CONTEXT context = *exception->ContextRecord;
        STACKFRAME64 frame{};
        frame.AddrPC.Offset = context.Rip;
        frame.AddrPC.Mode = AddrModeFlat;
        frame.AddrFrame.Offset = context.Rbp;
        frame.AddrFrame.Mode = AddrModeFlat;
        frame.AddrStack.Offset = context.Rsp;
        frame.AddrStack.Mode = AddrModeFlat;
        for (int i = 0; i < 16; ++i) {
            alignas(SYMBOL_INFOW) char storage[sizeof(SYMBOL_INFOW) + 1024 * sizeof(wchar_t)]{};
            auto *symbol = reinterpret_cast<SYMBOL_INFOW *>(storage);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFOW);
            symbol->MaxNameLen = 1023;
            DWORD64 offset = 0;
            DWORD lineOffset = 0;
            IMAGEHLP_LINE64 line{};
            line.SizeOfStruct = sizeof(line);
            QJsonObject item{{"address", QString::number(frame.AddrPC.Offset, 16)}};
            if (SymFromAddrW(process, frame.AddrPC.Offset, &offset, symbol))
                item["function"] = QString::fromWCharArray(symbol->Name).left(500);
            if (SymGetLineFromAddr64(process, frame.AddrPC.Offset, &lineOffset, &line)) {
                item["file"] = QString::fromLocal8Bit(line.FileName);
                item["line"] = int(line.LineNumber);
            }
            frames.append(item);
            if (!StackWalk64(IMAGE_FILE_MACHINE_AMD64, process, GetCurrentThread(), &frame,
                             &context, nullptr, SymFunctionTableAccess64, SymGetModuleBase64,
                             nullptr))
                break;
        }
        SymCleanup(process);
    }
    QFile file(nativeCrashPath);
    if (file.open(QIODevice::WriteOnly))
        file.write(QJsonDocument(
                       QJsonObject{{"schema", "paramcad.native-crash"},
                                   {"exception_code",
                                    QString::number(exception->ExceptionRecord->ExceptionCode, 16)},
                                   {"stack", frames}})
                       .toJson());
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif
void require(bool condition, const std::string &error) {
    if (!condition)
        throw std::runtime_error(error);
}
QJsonObject readObject(const QString &path) {
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "Cannot read JSON: " + path.toStdString());
    QJsonParseError error;
    const auto doc = QJsonDocument::fromJson(file.readAll(), &error);
    require(error.error == QJsonParseError::NoError && doc.isObject(), "Invalid JSON object");
    return doc.object();
}
void writeObject(const QString &path, const QJsonObject &report) {
    QSaveFile file(path);
    require(file.open(QIODevice::WriteOnly), "Cannot create report: " + path.toStdString());
    const auto bytes = QJsonDocument(report).toJson(QJsonDocument::Indented);
    require(file.write(bytes) == bytes.size() && file.commit(), "Cannot save report");
}
double volume(const TopoDS_Shape &shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}
QJsonObject sampledDifference(const TopoDS_Shape &reference, const TopoDS_Shape &actual,
                              double referenceVolume,
                              const std::function<bool(const gp_Pnt &)> &referenceInside = {},
                              const std::function<bool(const gp_Pnt &)> &actualInside = {}) {
    Bnd_Box box;
    BRepBndLib::AddOptimal(reference, box, false, false);
    BRepBndLib::AddOptimal(actual, box, false, false);
    double lo[3], hi[3];
    box.Get(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
    using Classifiers = std::vector<std::unique_ptr<BRepClass3d_SolidClassifier>>;
    auto classifiers = [](const TopoDS_Shape &shape) {
        Classifiers result;
        for (TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next())
            result.push_back(std::make_unique<BRepClass3d_SolidClassifier>(it.Current()));
        return result;
    };
    auto first = classifiers(reference), second = classifiers(actual);
    auto inside = [](Classifiers &solid, const gp_Pnt &p) {
        for (auto &classifier : solid) {
            classifier->Perform(p, 1e-7);
            if (classifier->State() == TopAbs_IN || classifier->State() == TopAbs_ON)
                return true;
        }
        return false;
    };
    constexpr int samples = 32768;
    std::mt19937 engine(4500);
    std::uniform_real_distribution<double> uniform(0, 1);
    int mismatches = 0;
    for (int i = 0; i < samples; ++i) {
        gp_Pnt p(lo[0] + (hi[0] - lo[0]) * uniform(engine),
                 lo[1] + (hi[1] - lo[1]) * uniform(engine),
                 lo[2] + (hi[2] - lo[2]) * uniform(engine));
        mismatches += (referenceInside ? referenceInside(p) : inside(first, p)) !=
                      (actualInside ? actualInside(p) : inside(second, p));
    }
    const double ratio = (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]) / referenceVolume;
    const double probability = double(mismatches) / samples, z = 3.0;
    const double upper = (probability + z * z / (2 * samples) +
                          z * std::sqrt(probability * (1 - probability) / samples +
                                        z * z / (4.0 * samples * samples))) /
                         (1 + z * z / samples);
    return {{"samples", samples},
            {"seed", 4500},
            {"mismatches", mismatches},
            {"estimate", probability * ratio},
            {"upper_confidence_bound", upper * ratio},
            {"confidence_z", z}};
}
QJsonArray vector(const modeling::Vec3 &v) { return {v.x, v.y, v.z}; }
std::function<bool(const gp_Pnt &)> analyticInside(const modeling::CurvedPrimitiveParams &params) {
    return [params](const gp_Pnt &point) {
        return std::visit(
            [&](const auto &p) {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>)
                    return point.SquareDistance(gp_Pnt(p.center.x, p.center.y, p.center.z)) <=
                           p.radius * p.radius;
                else {
                    modeling::Vec3 center;
                    if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                        center = p.center;
                    else
                        center = p.baseCenter;
                    const gp_Vec axis(
                        gp_Dir(p.axisDirection.x, p.axisDirection.y, p.axisDirection.z));
                    const gp_Vec d(gp_Pnt(center.x, center.y, center.z), point);
                    const double z = d.Dot(axis), rho2 = std::max(0.0, d.SquareMagnitude() - z * z);
                    if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                        return std::pow(std::sqrt(rho2) - p.majorRadius, 2) + z * z <=
                               p.minorRadius * p.minorRadius;
                    else {
                        double radius;
                        if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>)
                            radius = p.radius;
                        else
                            radius = p.baseRadius + (p.topRadius - p.baseRadius) * z / p.height;
                        return z >= 0 && z <= p.height && rho2 <= radius * radius;
                    }
                }
            },
            params);
    };
}
modeling::CurvedPrimitiveParams referencePrimitive(const QJsonObject &definition) {
    const auto p = definition["parameters"].toObject();
    const auto a = p["axis"].toArray();
    const modeling::Vec3 axis =
        a.size() == 3 ? modeling::Vec3{a[0].toDouble(), a[1].toDouble(), a[2].toDouble()}
                      : modeling::Vec3{0, 0, 1};
    const auto geometry = definition["geometry"].toString();
    if (geometry == "sphere")
        return modeling::SpherePrimitiveParams{{}, p["radius"].toDouble()};
    if (geometry == "torus")
        return modeling::TorusPrimitiveParams{
            {}, axis, p["major_radius"].toDouble(), p["minor_radius"].toDouble()};
    const double h = p["height"].toDouble();
    double offset = h / 2;
    if (geometry == "cone") {
        const double r0 = p["base_radius"].toDouble(), r1 = p["top_radius"].toDouble();
        offset = h * (r0 * r0 + 2 * r0 * r1 + 3 * r1 * r1) / (4 * (r0 * r0 + r0 * r1 + r1 * r1));
        return modeling::ConePrimitiveParams{
            {-axis.x * offset, -axis.y * offset, -axis.z * offset}, axis, r0, r1, h};
    }
    return modeling::CylinderPrimitiveParams{
        {-axis.x * offset, -axis.y * offset, -axis.z * offset}, axis, p["radius"].toDouble(), h};
}
// Independently sample the source primitive's boundary and measure distance to
// actual CAD faces. This checks the builder, not just reported fitted parameters.
double sourceBoundaryDistance(const modeling::CurvedPrimitiveParams &params,
                              const TopoDS_Shape &target) {
    BRep_Builder builder;
    TopoDS_Compound faces;
    builder.MakeCompound(faces);
    for (TopExp_Explorer it(target, TopAbs_FACE); it.More(); it.Next())
        builder.Add(faces, it.Current());
    double maximum = 0;
    std::mt19937 random(9221);
    std::uniform_real_distribution<double> uniform(0, 1);
    for (int i = 0; i < 128; ++i) {
        const double u = uniform(random), v = uniform(random), phi = 2 * std::acos(-1.0) * u;
        const auto point = std::visit(
            [&](const auto &p) -> gp_Pnt {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>) {
                    const double z = 2 * v - 1, r = std::sqrt(1 - z * z);
                    return gp_Pnt(p.center.x + p.radius * r * std::cos(phi),
                                  p.center.y + p.radius * r * std::sin(phi),
                                  p.center.z + p.radius * z);
                } else {
                    modeling::Vec3 center;
                    if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                        center = p.center;
                    else
                        center = p.baseCenter;
                    const gp_Ax2 frame(
                        gp_Pnt(center.x, center.y, center.z),
                        gp_Dir(p.axisDirection.x, p.axisDirection.y, p.axisDirection.z));
                    double radius, z;
                    if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>) {
                        const double theta = 2 * std::acos(-1.0) * v;
                        radius = p.majorRadius + p.minorRadius * std::cos(theta);
                        z = p.minorRadius * std::sin(theta);
                    } else {
                        double r0, r1;
                        if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>)
                            r0 = r1 = p.radius;
                        else {
                            r0 = p.baseRadius;
                            r1 = p.topRadius;
                        }
                        z = v * p.height;
                        radius = r0 + (r1 - r0) * v;
                        if (i % 3 == 1) {
                            z = 0;
                            radius = r0 * std::sqrt(v);
                        } else if (i % 3 == 2) {
                            z = p.height;
                            radius = r1 * std::sqrt(v);
                        }
                    }
                    return frame.Location().Translated(
                        gp_Vec(frame.XDirection()) * (radius * std::cos(phi)) +
                        gp_Vec(frame.YDirection()) * (radius * std::sin(phi)) +
                        gp_Vec(frame.Direction()) * z);
                }
            },
            params);
        BRepExtrema_DistShapeShape distance(BRepBuilderAPI_MakeVertex(point).Vertex(), faces);
        require(distance.IsDone(), "CAD boundary distance verification failed");
        maximum = std::max(maximum, distance.Value());
    }
    return maximum;
}
QJsonObject curvedParameters(const modeling::CurvedPrimitiveParams &params) {
    return std::visit(
        [](const auto &p) -> QJsonObject {
            using P = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<P, modeling::SpherePrimitiveParams>)
                return {
                    {"kind", "sphere"}, {"center_mm", vector(p.center)}, {"radius_mm", p.radius}};
            else if constexpr (std::is_same_v<P, modeling::TorusPrimitiveParams>)
                return {{"kind", "torus"},
                        {"center_mm", vector(p.center)},
                        {"axis_direction", vector(p.axisDirection)},
                        {"major_radius_mm", p.majorRadius},
                        {"minor_radius_mm", p.minorRadius}};
            else {
                QJsonObject result{{"base_center_mm", vector(p.baseCenter)},
                                   {"axis_direction", vector(p.axisDirection)},
                                   {"height_mm", p.height}};
                if constexpr (std::is_same_v<P, modeling::CylinderPrimitiveParams>) {
                    result["kind"] = "cylinder";
                    result["radius_mm"] = p.radius;
                } else {
                    result["kind"] = "cone";
                    result["base_radius_mm"] = p.baseRadius;
                    result["top_radius_mm"] = p.topRadius;
                }
                return result;
            }
        },
        params);
}
QJsonObject cloudBounds(const reconstruction::PointStore &cloud) {
    if (cloud.empty())
        return {};
    auto low = cloud.points().front().position, high = low;
    for (const auto &point : cloud.points()) {
        const auto &p = point.position;
        low.x = std::min(low.x, p.x);
        low.y = std::min(low.y, p.y);
        low.z = std::min(low.z, p.z);
        high.x = std::max(high.x, p.x);
        high.y = std::max(high.y, p.y);
        high.z = std::max(high.z, p.z);
    }
    return {{"minimum", vector(low)},
            {"maximum", vector(high)},
            {"dimensions", QJsonArray{high.x - low.x, high.y - low.y, high.z - low.z}}};
}
QJsonArray bounds(const TopoDS_Shape &shape) {
    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    return {x1 - x0, y1 - y0, z1 - z0};
}
TopoDS_Shape cut(const TopoDS_Shape &a, const TopoDS_Shape &b, double fuzzyTolerance = 0.0) {
    BRepAlgoAPI_Cut operation;
    TopTools_ListOfShape arguments, tools;
    arguments.Append(a);
    tools.Append(b);
    operation.SetArguments(arguments);
    operation.SetTools(tools);
    operation.SetFuzzyValue(fuzzyTolerance);
    operation.Build();
    require(operation.IsDone(), "Boolean cut failed");
    return operation.Shape();
}
TopoDS_Shape fuse(const TopoDS_Shape &a, const TopoDS_Shape &b) {
    BRepAlgoAPI_Fuse operation(a, b);
    require(operation.IsDone(), "Boolean fusion failed");
    operation.SimplifyResult();
    return operation.Shape();
}
std::filesystem::path nativePath(const QString &path) {
    return std::filesystem::path(path.toStdWString());
}
TopoDS_Shape readPlacedStep(const QString& path) {
    // Display imports intentionally center models. Direct analytic clouds retain
    // their source coordinates, so accuracy comparison must retain STEP placement.
    STEPControl_Reader reader;std::ifstream input(nativePath(path),std::ios::binary);
    require(input.good() && reader.ReadStream("reference.step",input)==IFSelect_RetDone,"Cannot read placed reference STEP");
    require(reader.TransferRoots()>0,"Reference STEP has no transferable roots");
    return reader.OneShape();
}
// Independent source construction from dimensions, never from fitted labels.
TopoDS_Shape mixedBoardSource(const QJsonObject& definition, double& analyticVolume) {
    const auto base=definition["base_size"].toArray();
    const double x=base[0].toDouble(),y=base[1].toDouble(),z=base[2].toDouble();
    auto shape=BRepPrimAPI_MakeBox(x,y,z).Shape();analyticVolume=x*y*z;
    for(const auto value:definition["features"].toArray()) {
        const auto f=value.toObject();const auto kind=f["kind"].toString();
        const bool subtract=kind.endsWith("through_hole");
        const auto origin=f.contains("center_xy")?f["center_xy"].toArray():f["origin_xy"].toArray();
        TopoDS_Shape tool;double volume=0;const double pi=std::acos(-1.0);
        if(kind=="round_through_hole") {
            const double r=f["radius"].toDouble();
            tool=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,-1),gp_Dir(0,0,1)),r,z+2).Shape();volume=pi*r*r*z;
        } else if(kind=="square_through_hole") {
            const auto s=f["size_xy"].toArray();const double w=s[0].toDouble(),d=s[1].toDouble();
            tool=BRepPrimAPI_MakeBox(gp_Pnt(0,0,-1),w,d,z+2).Shape();volume=w*d*z;
        } else if(kind=="cube" || kind=="rectangular_boss") {
            const auto s=f["size"].toArray();const double w=s[0].toDouble(),d=s[1].toDouble(),h=s[2].toDouble();
            tool=BRepPrimAPI_MakeBox(w,d,h).Shape();volume=w*d*h;
        } else if(kind=="cylinder_boss") {
            const double r=f["radius"].toDouble(),h=f["height"].toDouble();
            tool=BRepPrimAPI_MakeCylinder(r,h).Shape();volume=pi*r*r*h;
        } else if(kind=="frustum_boss") {
            const double a=f["base_radius"].toDouble(),b=f["top_radius"].toDouble(),h=f["height"].toDouble();
            tool=BRepPrimAPI_MakeCone(a,b,h).Shape();volume=pi*h*(a*a+a*b+b*b)/3;
        } else if(kind=="two_step_boss") {
            const auto s=f["size_xy"].toArray();const double w=s[0].toDouble(),d=s[1].toDouble();
            const double h=f["lower_height"].toDouble(),u=f["upper_width"].toDouble(),v=f["upper_height"].toDouble();
            tool=fuse(BRepPrimAPI_MakeBox(w,d,h).Shape(),BRepPrimAPI_MakeBox(gp_Pnt(0,0,h),u,d,v).Shape());
            volume=w*d*h+u*d*v;
        } else if(kind=="l_bracket_gusset") {
            const auto p=f["parameters"].toObject();
            const double w=p["width"].toDouble(),d=p["base_depth"].toDouble(),b=p["base_thickness"].toDouble();
            const double t=p["upright_thickness"].toDouble(),h=p["upright_height"].toDouble();
            const double r=p["rib_thickness"].toDouble(),run=p["rib_run"].toDouble(),rise=p["rib_rise"].toDouble();
            BRepBuilderAPI_MakePolygon polygon;
            polygon.Add(gp_Pnt((w-r)/2,t,b));polygon.Add(gp_Pnt((w-r)/2,t+run,b));polygon.Add(gp_Pnt((w-r)/2,t,b+rise));polygon.Close();
            const auto rib=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(r,0,0)).Shape();
            tool=fuse(fuse(BRepPrimAPI_MakeBox(w,d,b).Shape(),BRepPrimAPI_MakeBox(gp_Pnt(0,0,b),w,t,h).Shape()),rib);
            volume=w*d*b+w*t*h+0.5*r*run*rise;
        } else throw std::runtime_error("Unsupported mixed source feature");
        gp_Trsf pose;pose.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),f["yaw_degrees"].toDouble()*pi/180);
        pose.SetTranslationPart(gp_Vec(origin[0].toDouble(),origin[1].toDouble(),subtract?0:z));
        tool=BRepBuilderAPI_Transform(tool,pose,true).Shape();
        shape=subtract?cut(shape,tool):fuse(shape,tool);analyticVolume+=subtract?-volume:volume;
    }
    return shape;
}
void generate(const QStringList &args, QJsonObject &report) {
    require(args.size() == 5, "Usage: generate case.json model.step report.json");
    const auto definition = readObject(args[2]);
    require(definition["unit"] == "mm", "Standard cases must use millimeters");
    const auto p = definition["parameters"].toObject();
    const auto number = [&p](const char *key) {
        const double value = p[key].toDouble(-1);
        require(std::isfinite(value) && value > 0, std::string("Invalid dimension: ") + key);
        return value;
    };
    TopoDS_Shape shape;
    double analyticVolume = 0;
    if (definition["geometry"] == "mixed_template_board") {
        shape=mixedBoardSource(definition,analyticVolume);
    } else if (definition["geometry"] == "box_through_hole") {
        const double x = number("size_x"), y = number("size_y"), z = number("size_z");
        const double radius = number("hole_diameter") / 2;
        require(2 * radius < std::min(x, y), "Hole must stay within the box");
        shape = cut(BRepPrimAPI_MakeBox(x, y, z).Shape(),
                    BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x / 2, y / 2, -1), gp_Dir(0, 0, 1)),
                                             radius, z + 2)
                        .Shape());
        analyticVolume = x * y * z - std::acos(-1.0) * radius * radius * z;
    } else if (definition["geometry"] == "l_bracket_gusset") {
        const double w = number("width"), d = number("base_depth"), b = number("base_thickness");
        const double t = number("upright_thickness"), h = number("upright_height");
        const double r = number("rib_thickness"), run = number("rib_run"),
                     rise = number("rib_rise");
        require(t + run <= d && rise <= h && r < w, "Rib must fit inside the bracket");
        auto base = BRepPrimAPI_MakeBox(w, d, b).Shape();
        auto upright = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, b), w, t, h).Shape();
        BRepBuilderAPI_MakePolygon polygon;
        polygon.Add(gp_Pnt((w - r) / 2, t, b));
        polygon.Add(gp_Pnt((w - r) / 2, t + run, b));
        polygon.Add(gp_Pnt((w - r) / 2, t, b + rise));
        polygon.Close();
        const auto rib =
            BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(), gp_Vec(r, 0, 0))
                .Shape();
        shape = fuse(fuse(base, upright), rib);
        analyticVolume = w * d * b + w * t * h + 0.5 * r * run * rise;
    } else if (definition["geometry"] == "sphere" || definition["geometry"] == "cylinder" ||
               definition["geometry"] == "cone" || definition["geometry"] == "torus") {
        const auto center = p["center"].toArray(), direction = p["axis"].toArray();
        const gp_Pnt origin(center[0].toDouble(), center[1].toDouble(), center[2].toDouble());
        const gp_Dir axis =
            direction.size() == 3
                ? gp_Dir(direction[0].toDouble(), direction[1].toDouble(), direction[2].toDouble())
                : gp_Dir(0, 0, 1);
        const double pi = std::acos(-1.0);
        const auto geometry = definition["geometry"].toString();
        if (geometry == "sphere") {
            const double r = number("radius");
            shape = BRepPrimAPI_MakeSphere(origin, r).Shape();
            analyticVolume = 4 * pi * r * r * r / 3;
        } else if (geometry == "torus") {
            const double R = number("major_radius"), r = number("minor_radius");
            require(R > r, "Ring torus requires R > r");
            shape = BRepPrimAPI_MakeTorus(gp_Ax2(origin, axis), R, r).Shape();
            analyticVolume = 2 * pi * pi * R * r * r;
        } else {
            const double h = number("height");
            double offset = h / 2;
            if (geometry == "cone") {
                const double a = p["base_radius"].toDouble(), b = p["top_radius"].toDouble();
                require(a >= 0 && b >= 0 && std::max(a, b) > 0 && a != b, "Invalid cone radii");
                offset = h * (a * a + 2 * a * b + 3 * b * b) / (4 * (a * a + a * b + b * b));
                auto base = origin.Translated(gp_Vec(axis) * (-offset));
                shape = BRepPrimAPI_MakeCone(gp_Ax2(base, axis), a, b, h).Shape();
                analyticVolume = pi * h * (a * a + a * b + b * b) / 3;
            } else {
                const double r = number("radius");
                auto base = origin.Translated(gp_Vec(axis) * (-offset));
                shape = BRepPrimAPI_MakeCylinder(gp_Ax2(base, axis), r, h).Shape();
                analyticVolume = pi * r * r * h;
            }
        }
    } else if (definition["geometry"] == "box_union") {
        for (const auto value : p["boxes"].toArray()) {
            const auto box = value.toObject();
            const auto origin = box["origin"].toArray(), size = box["size"].toArray();
            auto solid =
                BRepPrimAPI_MakeBox(
                    gp_Pnt(origin[0].toDouble(), origin[1].toDouble(), origin[2].toDouble()),
                    size[0].toDouble(), size[1].toDouble(), size[2].toDouble())
                    .Shape();
            shape = shape.IsNull() ? solid : fuse(shape, solid);
        }
        const auto rib = p["rib"].toObject();
        if (!rib.isEmpty()) {
            const auto vertices = rib["vertices"].toArray();
            BRepBuilderAPI_MakePolygon polygon;
            for (const auto v : vertices) {
                auto q = v.toArray();
                polygon.Add(gp_Pnt(q[0].toDouble(), q[1].toDouble(), q[2].toDouble()));
            }
            polygon.Close();
            shape =
                fuse(shape, BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),
                                                  gp_Vec(rib["depth"].toDouble(), 0, 0))
                                .Shape());
        }
        const auto rotation = p["rotation"].toArray();
        if (rotation.size() == 3) {
            gp_Trsf transform;
            auto a = rotation[0].toArray(), b = rotation[1].toArray(), c = rotation[2].toArray();
            transform.SetValues(a[0].toDouble(), a[1].toDouble(), a[2].toDouble(), 0,
                                b[0].toDouble(), b[1].toDouble(), b[2].toDouble(), 0,
                                c[0].toDouble(), c[1].toDouble(), c[2].toDouble(), 0);
            shape = BRepBuilderAPI_Transform(shape, transform, true).Shape();
        }
        analyticVolume = definition["expected"].toObject()["volume_mm3"].toDouble();
    } else
        throw std::runtime_error("Unsupported standard-case geometry");
    require(BRepCheck_Analyzer(shape).IsValid(), "Generated B-Rep is invalid");
    require(std::abs(volume(shape) - analyticVolume) < analyticVolume * 1e-8,
            "Analytic volume mismatch");
    std::string error;
    require(modeling::exportStep(shape, nativePath(args[3]), &error), error);
    LoadedModel loaded;
    QString loadError;
    StepModelLoader loader;
    require(loader.load(args[3], loaded, loadError), loadError.toStdString());
    require(loaded.solidCount == definition["expected"].toObject()["solids"].toInt(1) &&
                BRepCheck_Analyzer(loaded.shape).IsValid(),
            "STEP solid count or validity mismatch");
    require(std::abs(volume(loaded.shape) - analyticVolume) < analyticVolume * 1e-8,
            "STEP volume mismatch");
    const auto actualBounds = bounds(loaded.shape);
    const auto expectedBounds = definition["expected"].toObject()["bounding_box_mm"].toArray();
    require(expectedBounds.size() == 3, "Missing expected bounding box");
    for (int i = 0; i < 3; ++i)
        require(std::abs(actualBounds[i].toDouble() - expectedBounds[i].toDouble()) < 1e-6,
                "STEP dimension mismatch");
    report["status"] = "generated_and_round_trip_verified";
    report["case_id"] = definition["id"];
    report["unit"] = "mm";
    report["volume_mm3"] = volume(loaded.shape);
    report["analytic_volume_mm3"] = analyticVolume;
    report["bounding_box_mm"] = actualBounds;
    report["solids"] = loaded.solidCount;
    report["faces"] = loaded.faceCount;
    report["edges"] = loaded.edgeCount;
    report["vertices"] = loaded.vertexCount;
    report["brep_valid"] = true;
}
void fit(const QStringList &args, QJsonObject &report) {
    require(args.size() == 5 || args.size() == 7,
            "Usage: fit cloud.ply report.json result.step [reference.step "
            "case.json]");
    const auto checkpoint = [&](const char *stage) {
        report["stage"] = stage;
        report["status"] = "running";
        report["complete_cad_fit"] = false;
        writeObject(args[3], report);
    };
    checkpoint("point_cloud_loading");
    PlyCloud cloud;
    QString error;
    require(PlyReader::read(args[2], cloud, error), error.toStdString());
    const auto unit = PointCloudAdapter::lengthUnitFor(args[2]);
    require(unit != reconstruction::LengthUnit::Arbitrary,
            "Metric cloud metadata is required; no implicit millimeter assumption");
    const auto source = PointCloudAdapter::toPointStore(cloud, unit);
    reconstruction::PointStore processed({}, unit);
    reconstruction::PointCloudPreprocessingReport pre;
    std::string text;
    const auto preprocessingOptions = PointCloudAdapter::preprocessingOptionsFor(cloud);
    checkpoint("preprocessing");
    require(
        reconstruction::preprocessPointCloud(source, preprocessingOptions, processed, pre, text),
        text);
    report["preprocessing"] =
        QJsonObject{{"input_points", double(pre.inputPointCount)},
                    {"output_points", double(pre.outputPointCount)},
                    {"voxel_duplicates_removed", double(pre.removedByVoxel)},
                    {"statistical_outliers_removed", double(pre.removedAsOutliers)},
                    {"normals_estimated", double(pre.normalsEstimated)},
                    {"input_normals", double(pre.normalsFromInput)},
                    {"estimated_normals_oriented", double(pre.estimatedNormalsOriented)},
                    {"normals_without_orientation", double(pre.normalsWithoutOrientation)},
                    {"degenerate_neighborhoods", double(pre.degenerateNeighborhoods)},
                    {"outlier_median_neighbor_distance_mm", pre.outlierMedianNeighborDistance},
                    {"outlier_distance_threshold_mm", pre.outlierDistanceThreshold},
                    {"outlier_neighbor_count", double(preprocessingOptions.outlierNeighborCount)},
                    {"normal_neighbor_count", double(preprocessingOptions.normalNeighborCount)},
                    {"outlier_robust_median_multiplier", preprocessingOptions.outlierRobustMedianMultiplier},
                    {"normal_orientation_minimum_alignment", preprocessingOptions.normalOrientationMinimumAlignment},
                    {"normal_orientation_maximum_tangent_deviation", preprocessingOptions.normalOrientationMaximumTangentDeviation},
                    {"normal_orientation_maximum_spacing_ratio", preprocessingOptions.normalOrientationMaximumSpacingRatio},
                    {"voxel_size", preprocessingOptions.voxelSize},
                    {"scale_to_mm", pre.appliedScaleToMillimeters}};
    auto planes = PointCloudAdapter::planeOptionsFor(processed);
    auto cylinders = PointCloudAdapter::cylinderOptionsFor(processed);
    const auto seedText = qEnvironmentVariable("PCAD_RANSAC_SEED");
    bool validSeed = true;
    const auto seed = seedText.isEmpty() ? 4500 : seedText.toULongLong(&validSeed);
    require(validSeed && seed <= std::numeric_limits<std::uint32_t>::max(),
            "Invalid PCAD_RANSAC_SEED");
    planes.randomSeed = cylinders.randomSeed = static_cast<std::uint32_t>(seed);
    report["ransac_seed"] = double(seed);
    report["processed_bounds_mm"] = cloudBounds(processed);
    reconstruction::SurfaceAnalysisResult analysis;
    checkpoint("surface_analysis");
    const bool analyzed = reconstruction::analyzeSurfaces(processed,
        reconstruction::recognitionAnalysisOptions(processed, planes, cylinders), analysis, text);
    report["surface_analysis"] = QJsonObject{{"surfaces", double(analysis.surfaces.size())},
        {"patches", double(analysis.patches.size())}, {"relations", double(analysis.relations.size())},
        {"detection_passes", double(analysis.detectionPasses)},
        {"point_fingerprint", QString::number(analysis.pointFingerprint)}};
    reconstruction::CompoundCandidate compound;
    checkpoint("compound_reconstruction");
    auto recoverySamples=source.points();
    for(auto& p:recoverySamples) {p.position.x*=pre.appliedScaleToMillimeters;p.position.y*=pre.appliedScaleToMillimeters;p.position.z*=pre.appliedScaleToMillimeters;}
    const reconstruction::PointStore recoverySource(std::move(recoverySamples),processed.unit());
    reconstruction::RepairTransactionReport repairTransaction;
    const bool compoundRecognized = analyzed &&
        reconstruction::recognizeAndRepairParametricSolid(processed, analysis, compound, text, &recoverySource, &repairTransaction);
    if(!compoundRecognized)analysis.localFailures=repairTransaction.failures;
    report["repair_transaction"]=QJsonObject{{"committed",repairTransaction.committed},{"reused",repairTransaction.reused},
        {"reason",QString::fromStdString(repairTransaction.reason)},{"before_coverage",repairTransaction.beforeCoverage},
        {"after_coverage",repairTransaction.afterCoverage},{"supplemental_points",double(analysis.supplementalPoints.size())},
        {"surface_mappings",double(analysis.repairMappings.size())}};
    QJsonArray surfaceMappings,patchMappings;
    for(const auto& m:analysis.repairMappings)surfaceMappings.append(QJsonObject{{"local_fingerprint",QString::number(m.localFingerprint)},{"local",QString::number(m.local)},{"global",QString::number(m.global)}});
    for(const auto& m:analysis.repairPatchMappings)patchMappings.append(QJsonObject{{"local_fingerprint",QString::number(m.localFingerprint)},{"local",QString::number(m.local)},{"global",QString::number(m.global)}});
    report["effective_surface_analysis"]=QJsonObject{{"point_fingerprint",QString::number(analysis.pointFingerprint)},
        {"base_fingerprint",QString::number(analysis.baseFingerprint)},{"surfaces",double(analysis.surfaces.size())},
        {"patches",double(analysis.patches.size())},{"relations",double(analysis.relations.size())},
        {"surface_mappings",surfaceMappings},{"patch_mappings",patchMappings}};
    auto serializeCandidates=[](const std::vector<reconstruction::PrimitiveEvidence>& candidates) {
        QJsonArray values;
        for(const auto& candidate:candidates)std::visit([&](const auto& e) {
            QJsonArray support;for(auto id:e.supportPointIds)support.append(QString::number(id));
            QJsonObject v{{"kind",int(reconstruction::primitiveKind(candidate))},{"support_point_ids",support},
                {"mean_residual_mm",e.meanAbsoluteResidual},{"max_residual_mm",e.maxAbsoluteResidual},{"confidence",e.confidence}};
            using E=std::decay_t<decltype(e)>;
            if constexpr(std::is_same_v<E,reconstruction::PlaneEvidence>) {v["normal"]=vector(e.plane.normal);v["offset_mm"]=e.plane.offset;}
            else if constexpr(std::is_same_v<E,reconstruction::CylinderEvidence>) {v["axis_point_mm"]=vector(e.cylinder.axisPoint);v["axis_direction"]=vector(e.cylinder.axisDirection);v["radius_mm"]=e.cylinder.radius;}
            else if constexpr(std::is_same_v<E,reconstruction::ConeEvidence>) {v["apex_mm"]=vector(e.cone.apex);v["axis_direction"]=vector(e.cone.axisDirection);v["opening_angle_radians"]=e.cone.openingAngleRadians;}
            else if constexpr(std::is_same_v<E,reconstruction::SphereEvidence>) {v["center_mm"]=vector(e.sphere.center);v["radius_mm"]=e.sphere.radius;}
            else {v["center_mm"]=vector(e.torus.center);v["axis_direction"]=vector(e.torus.axisDirection);v["major_radius_mm"]=e.torus.majorRadius;v["minor_radius_mm"]=e.torus.minorRadius;}
            values.append(v);
        },candidate);
        return values;
    };
    QJsonArray localFailures;
    for(const auto& f:analysis.localFailures) {
        QJsonArray ids,recovered;for(auto id:f.pointIds)ids.append(QString::number(id));
        for(auto id:f.recoveredPointIds)recovered.append(QString::number(id));
        localFailures.append(QJsonObject{{"reason",reconstruction::localFailureReasonName(f.reason)},{"message",QString::fromStdString(f.message)},
            {"point_ids",ids},{"recovered_point_ids",recovered},{"source_fingerprint",QString::number(f.sourceFingerprint)},
            {"candidates",serializeCandidates(f.candidates)},{"repaired_candidates",serializeCandidates(f.repairedCandidates)},
            {"rejected_relations",double(f.rejectedRelations.size())},{"detection_passes",double(f.detectionPasses)},{"repaired",f.repaired}});
    }
    report["local_analysis_failures"]=localFailures;
    const std::string compoundFailure = text;
    report["detection_options"] =
        QJsonObject{{"plane_distance_threshold_mm", planes.distanceThreshold},
                    {"plane_minimum_support", double(analysis.options.detection.plane.minimumSupportPoints)},
                    {"cylinder_distance_threshold_mm", cylinders.distanceThreshold},
                    {"cylinder_minimum_support", double(analysis.options.detection.cylinder.minimumSupportPoints)},
                    {"shared_maximum_planes", double(analysis.options.detection.plane.maximumPlanes)}};
    reconstruction::PrimitiveDetectionResult evidence;
    evidence.evidence = analysis.surfaces;
    evidence.unassignedPointIds = analysis.unassignedPointIds;
    const bool detected = analyzed;
    QJsonArray summaries;
    QJsonObject counts{{"planes", 0}, {"cylinders", 0}, {"spheres", 0}, {"cones", 0}, {"tori", 0}};
    const char *keys[] = {"planes", "cylinders", "spheres", "cones", "tori"};
    for (const auto &primitive : evidence.evidence) {
        const char *key = keys[static_cast<int>(reconstruction::primitiveKind(primitive))];
        counts[key] = counts[key].toInt() + 1;
        std::visit(
            [&](const auto &e) {
                QJsonObject item{{"kind", key},
                                 {"support_points", double(e.supportPointIds.size())},
                                 {"mean_residual_mm", e.meanAbsoluteResidual},
                                 {"confidence", e.confidence}};
                using E = std::decay_t<decltype(e)>;
                if constexpr (std::is_same_v<E, reconstruction::PlaneEvidence>) {
                    item["normal"] = vector(e.plane.normal);
                    item["offset_mm"] = e.plane.offset;
                } else if constexpr (std::is_same_v<E, reconstruction::CylinderEvidence>) {
                    item["diameter_mm"] = 2 * e.cylinder.radius;
                    item["angular_coverage"] = e.angularCoverage;
                    item["axial_span_mm"] = e.axialMaximum - e.axialMinimum;
                }
                summaries.append(item);
            },
            primitive);
    }
    counts["unassigned_points"] = double(evidence.unassignedPointIds.size());
    counts["detected"] = detected;
    counts["skipped_for_compound"] = false;
    counts["shared_with_recognition"] = true;
    if (!detected)
        counts["reason"] = QString::fromStdString(text);
    report["primitive_proposals"] = counts;
    report["evidence_summary"] = summaries;
    reconstruction::BoxWithThroughHolesCandidate candidate;
    checkpoint("production_reconstruction");
    const bool recognized = compoundRecognized;
    // Optional joint diagnostics must not overwrite the production failure.
    if (!recognized)
        text = compoundFailure;
    if (compoundRecognized && !compound.boxes.empty())
        candidate.box = compound.boxes.front();
    candidate.throughHoles = compound.throughHoles;
    report["compound_recognition"] =
        QJsonObject{{"recognized", compoundRecognized},
                    {"boxes", int(compound.boxes.size())},
                    {"prisms", int(compound.prisms.size())},
                    {"curved_solids", int(compound.curvedSolids.size())},
                    {"occupancy_error", compound.occupancyError},
                    {"grid_cells", int(compound.gridCells)},
                    {"surface_coverage", compound.surfaceCoverage},
                    {"minimum_box_count_proven", compound.minimumBoxCountProven},
                    {"search_nodes", int(compound.searchNodes)},
                    {"fallback_reason", QString::fromStdString(compoundFailure)}};
    QJsonArray boxGeometry,prismGeometry;
    for(const auto& box:compound.boxes) {
        const auto& p=box.primitive;
        boxGeometry.append(QJsonObject{{"origin_mm",vector(p.pose.origin)},
            {"x_direction",vector(p.pose.xDirection)},{"z_direction",vector(p.pose.zDirection)},
            {"dimensions_mm",QJsonArray{p.sizeX,p.sizeY,p.sizeZ}}});
    }
    for(const auto& p:compound.prisms) {
        QJsonArray polygon;for(auto vertex:p.polygon) polygon.append(QJsonArray{vertex.x,vertex.y});
        prismGeometry.append(QJsonObject{{"origin_mm",vector(p.plane.origin)},{"normal",vector(p.plane.normal)},
            {"x_direction",vector(p.plane.xDirection)},{"depth_mm",p.depth},{"subtract",p.subtract},{"polygon_mm",polygon}});
    }
    report["boxes"]=boxGeometry;report["prisms"]=prismGeometry;
    const bool boxRecognized = !compound.boxes.empty();
    QJsonObject recognition{
        {"recognized", recognized},
        {"reason", recognized ? QJsonValue() : QJsonValue(QString::fromStdString(text))}};
    if (!recognized) {
        recognition["failure_stage"] = "solid_recognition";
    }
    report["recognition"] = recognition;
    report["box_recognition"] = QJsonObject{{"recognized", boxRecognized}};
    if (boxRecognized)
        report["box_recognition"] =
            QJsonObject{{"recognized", true},
                        {"dimensions_mm",
                         QJsonArray{candidate.box.primitive.sizeX, candidate.box.primitive.sizeY,
                                    candidate.box.primitive.sizeZ}},
                        {"confidence", candidate.box.confidence}};
    report["complete_cad_fit"] = false;
    report["status"] = "cad_not_recognized";
    if (!recognized)
        return;
    QJsonArray holes;
    for (const auto &hole : candidate.throughHoles)
        holes.append(QJsonObject{{"diameter_mm", 2 * hole.primitive.radius},
                                 {"axis_point_mm", vector(hole.primitive.axisPoint)},
                                 {"axis_direction", vector(hole.primitive.axisDirection)},
                                 {"confidence", hole.confidence}});
    report["through_holes"] = holes;
    QJsonArray curved;
    for (const auto &solid : compound.curvedSolids) {
        auto item = curvedParameters(solid.primitive);
        item["surface_coverage"] = solid.surfaceCoverage;
        item["boundary_coverage"] = solid.boundaryCoverage;
        item["mean_residual_mm"] = solid.meanResidual;
        item["join"] = std::visit([](const auto& p){return p.join;},solid.primitive);
        curved.append(item);
    }
    report["curved_solids"] = curved;
    modeling::ModelingCore core;
    checkpoint("model_patch");
    const auto commit =
        compoundRecognized ? reconstruction::commitCompoundCandidate(compound, core, 0)
                           : reconstruction::commitBoxWithThroughHolesCandidate(candidate, core, 0);
    require(commit.success, commit.error);
    require(BRepCheck_Analyzer(core.bodyShape()).IsValid(), "Reconstructed B-Rep is invalid");
    // A small residual skin can evade a global volume threshold. Verify the
    // declared through-cut at several depths of the independently fitted host.
    QJsonArray throughCuts;
    for(const auto& cut:compound.prisms)if(cut.subtract && !compound.boxes.empty() && !cut.polygon.empty()) {
        const auto& host=compound.boxes.front().primitive;
        gp_Vec n(cut.plane.normal.x,cut.plane.normal.y,cut.plane.normal.z),u(cut.plane.xDirection.x,cut.plane.xDirection.y,cut.plane.xDirection.z);
        n.Normalize();u.Normalize();const auto v=n.Crossed(u);
        double x=0,y=0;for(const auto& q:cut.polygon){x+=q.x;y+=q.y;}x/=cut.polygon.size();y/=cut.polygon.size();
        gp_Pnt center(cut.plane.origin.x,cut.plane.origin.y,cut.plane.origin.z);center.Translate(u*x+v*y);
        gp_Vec bx(host.pose.xDirection.x,host.pose.xDirection.y,host.pose.xDirection.z),bz(host.pose.zDirection.x,host.pose.zDirection.y,host.pose.zDirection.z);
        bx.Normalize();bz.Normalize();const auto by=bz.Crossed(bx);double low=INFINITY,high=-INFINITY;
        for(int i=0;i<2;++i)for(int j=0;j<2;++j)for(int k=0;k<2;++k){
            gp_Pnt p(host.pose.origin.x,host.pose.origin.y,host.pose.origin.z);p.Translate(bx*(i*host.sizeX)+by*(j*host.sizeY)+bz*(k*host.sizeZ));
            const double t=gp_Vec(gp_Pnt(0,0,0),p).Dot(n);low=std::min(low,t);high=std::max(high,t);
        }
        const double height=gp_Vec(gp_Pnt(0,0,0),center).Dot(n);
        for(double fraction:{0.001,0.5,0.999}) {
            const auto p=center.Translated(n*(low+(high-low)*fraction-height));
            for(TopExp_Explorer solid(core.bodyShape(),TopAbs_SOLID);solid.More();solid.Next()) {
                BRepClass3d_SolidClassifier classifier(solid.Current(),p,1e-7);
                require(classifier.State()==TopAbs_OUT,"Subtractive through cut retains material within host thickness");
            }
        }
        throughCuts.append(QJsonObject{{"host_span_mm",high-low},{"empty_at_bottom_middle_top",true}});
    }
    report["subtractive_through_span_checks"]=throughCuts;
    checkpoint("step_export");
    require(modeling::exportStep(core.bodyShape(), nativePath(args[4]), &text), text);
    LoadedModel exported;
    checkpoint("step_round_trip");
    StepModelLoader exportReader;
    require(exportReader.load(args[4], exported, error), error.toStdString());
    const int expectedSolids = args.size() == 7
                                   ? readObject(args[6])["expected"].toObject()["solids"].toInt(1)
                                   : exported.solidCount;
    require(exported.solidCount == expectedSolids && BRepCheck_Analyzer(exported.shape).IsValid(),
            "Exported CAD solid count or validity mismatch");
    require(std::abs(volume(exported.shape) - volume(core.bodyShape())) <
                volume(core.bodyShape()) * 1e-7,
            "Exported CAD changed volume");
    report["status"] = "cad_exported";
    report["cad"] = QJsonObject{{"brep_valid", true},
                                {"features", int(core.features().size())},
                                {"solids", exported.solidCount},
                                {"step", args[4]},
                                {"step_round_trip_verified", true},
                                {"volume_mm3", volume(core.bodyShape())},
                                {"bounding_box_mm", bounds(core.bodyShape())}};
    // Recognizing a box alone does not establish that the source part was
    // recovered.
    if (args.size() == 7) {
        LoadedModel reference;
        StepModelLoader loader;
        require(loader.load(args[5], reference, error), error.toStdString());
        const auto definition = readObject(args[6]);
        if(definition["geometry"]=="mixed_template_board") reference.shape=readPlacedStep(args[5]);
        const auto expected = definition["expected"].toObject();
        const auto acceptance = definition["acceptance"].toObject();
        const double referenceVolume = volume(reference.shape);
        // Exact test clouds put independently constructed faces within floating
        // point noise of one another. Use a documented 0.00001 mm boolean
        // tolerance to avoid spurious whole-body differences at coincident faces.
        constexpr double comparisonTolerance = 1.0e-5;
        checkpoint("accuracy_boolean_comparison");
        const auto removed = cut(reference.shape, core.bodyShape(), comparisonTolerance);
        const auto added = cut(core.bodyShape(), reference.shape, comparisonTolerance);
        const double rawDifference = (volume(removed) + volume(added)) / referenceVolume;
        const auto occtSampling =
            sampledDifference(reference.shape, core.bodyShape(), referenceVolume);
        auto sampling = occtSampling;
        double boundaryDistance = 0;
        const bool analytic = expected.contains("curved_kind") && compound.curvedSolids.size() == 1;
        if (analytic) {
            const auto sourcePrimitive = referencePrimitive(definition);
            boundaryDistance = sourceBoundaryDistance(sourcePrimitive, core.bodyShape());
            sampling = sampledDifference(reference.shape, core.bodyShape(), referenceVolume,
                                         analyticInside(sourcePrimitive),
                                         analyticInside(compound.curvedSolids.front().primitive));
            sampling["method"] = "independent_analytic_membership";
        }
        const bool booleanTrusted =
            BRepCheck_Analyzer(removed).IsValid() && BRepCheck_Analyzer(added).IsValid() &&
            rawDifference >= 0 && std::abs(rawDifference - sampling["estimate"].toDouble()) < 0.02;
        const double difference =
            booleanTrusted ? rawDifference : sampling["upper_confidence_bound"].toDouble();
        const auto actualBounds = bounds(core.bodyShape());
        const auto expectedBounds = expected["bounding_box_mm"].toArray();
        QJsonArray dimensionErrors;
        bool dimensionsPass = true;
        for (int i = 0; i < 3; ++i) {
            double value = std::abs(actualBounds[i].toDouble() - expectedBounds[i].toDouble());
            dimensionErrors.append(value);
            dimensionsPass &= value <= acceptance["dimension_tolerance_mm"].toDouble();
        }
        auto expectedDiameters = expected["hole_diameters_mm"].toArray();
        std::vector<double> actualDiameters, targetDiameters;
        for (auto h : holes)
            actualDiameters.push_back(h.toObject()["diameter_mm"].toDouble());
        for (auto d : expectedDiameters)
            targetDiameters.push_back(d.toDouble());
        std::sort(actualDiameters.begin(), actualDiameters.end());
        std::sort(targetDiameters.begin(), targetDiameters.end());
        bool holesPass = holes.size() == expected["through_holes"].toInt() &&
                         actualDiameters.size() == targetDiameters.size();
        QJsonArray diameterErrors;
        for (std::size_t i = 0; i < std::min(actualDiameters.size(), targetDiameters.size()); ++i) {
            const double value = std::abs(actualDiameters[i] - targetDiameters[i]);
            diameterErrors.append(value);
            holesPass &= value <= acceptance["hole_diameter_tolerance_mm"].toDouble();
        }
        bool primitivePass = true;
        QJsonObject primitiveErrors;
        if (expected.contains("curved_kind")) {
            primitivePass =
                curved.size() == 1 && curved[0].toObject()["kind"] == expected["curved_kind"];
            auto actual = curved.isEmpty() ? QJsonObject{} : curved[0].toObject();
            if (actual["kind"] == "cone") {
                actual["small_radius_mm"] = std::min(actual["base_radius_mm"].toDouble(),
                                                     actual["top_radius_mm"].toDouble());
                actual["large_radius_mm"] = std::max(actual["base_radius_mm"].toDouble(),
                                                     actual["top_radius_mm"].toDouble());
            }
            const auto targets = expected["curved_dimensions_mm"].toObject();
            for (auto it = targets.begin(); it != targets.end(); ++it) {
                const double value = std::abs(actual[it.key()].toDouble() - it.value().toDouble());
                primitiveErrors[it.key()] = value;
                primitivePass &= actual.contains(it.key()) &&
                                 value <= acceptance["primitive_dimension_tolerance_mm"].toDouble();
            }
        }
        report["accuracy"] =
            QJsonObject{{"dimension_errors_mm", dimensionErrors},
                        {"primitive_dimension_errors_mm", primitiveErrors},
                        {"primitive_dimensions_pass", primitivePass},
                        {"source_boundary_distance_max_mm",
                         analytic ? QJsonValue(boundaryDistance) : QJsonValue()},
                        {"occt_volume_sampling", occtSampling},
                        {"dimensions_pass", dimensionsPass},
                        {"hole_diameter_errors_mm", diameterErrors},
                        {"holes_pass", holesPass},
                        {"relative_volume_error",
                         std::abs(volume(core.bodyShape()) - referenceVolume) / referenceVolume},
                        {"symmetric_difference_volume_fraction", difference},
                        {"boolean_fuzzy_tolerance_mm", comparisonTolerance},
                        {"boolean_raw_difference_fraction", rawDifference},
                        {"boolean_result_trusted", booleanTrusted},
                        {"difference_method", booleanTrusted ? "boolean_verified_by_sampling"
                                                             : "sampling_upper_confidence_bound"},
                        {"independent_volume_sampling", sampling}};
        const double tolerance = acceptance["symmetric_difference_volume_fraction"].toDouble();
        const bool passed =
            dimensionsPass && holesPass && primitivePass && difference <= tolerance &&
            sampling["upper_confidence_bound"].toDouble() <= tolerance &&
            (!analytic || boundaryDistance <= acceptance["dimension_tolerance_mm"].toDouble());
        report["complete_cad_fit"] = passed;
        report["status"] = passed ? "cad_accuracy_passed" : "cad_accuracy_failed";
    }
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    const auto args = application.arguments();
    const bool generating = args.value(1) == "generate";
    const QString reportPath = generating ? args.value(4) : args.value(3);
#ifdef _WIN32
    nativeCrashPath = reportPath + ".native-crash.json";
    QFile::remove(nativeCrashPath);
    SetUnhandledExceptionFilter(recordNativeCrash);
#endif
    QJsonObject report{{"schema", "paramcad.cad-validation"}, {"version", 1}};
    QElapsedTimer timer;
    timer.start();
    int exitCode = 0;
    try {
        if (generating) {
            generate(args, report);
        } else if(args.value(1) == "sample") {
            sampleStepCloud(args, report);
        } else if(args.value(1) == "inspect-step") {
            inspectStepTopology(args,report);
        } else if(args.value(1) == "reconstruct-cloud") {
            validatePointCloudCad(args,report);
            if(!report["cad_constructed"].toBool())exitCode=1;
        } else if(args.value(1) == "analyze" || args.value(1) == "analyze-client") {
            validateSurfaceCoverage(args, report);
        } else {
            require(args.value(1) == "fit", "Expected generate or fit mode");
            fit(args, report);
        }
    } catch (const Standard_Failure &error) {
        report["status"] = "failed";
        report["error"] = error.what();
        exitCode = 2;
    } catch (const std::exception &error) {
        report["status"] = "failed";
        report["error"] = error.what();
        exitCode = 2;
    }
    report["elapsed_seconds"] = timer.elapsed() / 1000.0;
    if (report["status"] != "failed")
        report["stage"] = "completed";
    if (!reportPath.isEmpty()) {
        try {
            writeObject(reportPath, report);
        } catch (const std::exception &error) {
            std::cerr << error.what() << '\n';
            return 2;
        }
    }
    std::cout << "CAD validation: " << report["status"].toString().toStdString() << '\n';
    return exitCode;
}
