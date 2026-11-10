#include "ModuleCli.h"

#include "io/PlyWriter.h"

#include <reconstruction/settings/ModuleSettings.h>
#include <reconstruction/testing/SyntheticPointCloud.h>

#include <QCoreApplication>
#include <QFileInfo>
#include <QJsonDocument>
#include <QSaveFile>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

ModuleArguments parseModuleArguments(const QStringList& arguments, const QString& usage)
{
    ModuleArguments result;
    for (qsizetype index = 1; index < arguments.size(); ++index) {
        const auto argument = arguments[index];
        const auto readValue = [&]() -> QString {
            if (index + 1 >= arguments.size() || arguments[index + 1].startsWith(QStringLiteral("--")))
                throw std::invalid_argument("Missing value for " + argument.toStdString());
            return arguments[++index];
        };
        if (argument == QStringLiteral("--help") || argument == QStringLiteral("-h")) {
            std::cout << usage.toStdString() << '\n';
            std::exit(0);
        }
        if (argument == QStringLiteral("--in")) result.input = readValue();
        else if (argument == QStringLiteral("--out")) result.output = readValue();
        else if (argument == QStringLiteral("--report")) result.report = readValue();
        else if (argument == QStringLiteral("--step")) result.step = readValue();
        else if (argument == QStringLiteral("--unit")) result.unit = readValue();
        else if (argument == QStringLiteral("--synthetic")) result.synthetic = readValue();
        else if (argument == QStringLiteral("--path")) result.path = readValue();
        else if (argument == QStringLiteral("--config"))
            result.config = std::filesystem::path(readValue().toStdU16String());
        else if (argument == QStringLiteral("--set")) result.overrides.push_back(readValue().toStdString());
        else if (argument == QStringLiteral("--print-config")) result.printConfig = true;
        else throw std::invalid_argument("Unknown option: " + argument.toStdString() + "\n" + usage.toStdString());
    }
    return result;
}

void prepareModuleOverrides(const std::string& moduleId, const std::vector<std::string>& values)
{
    for (const auto& value : values) {
        const auto separator = value.find('=');
        if (separator == value.npos) throw std::invalid_argument("Expected key=value: " + value);
        const auto key = value.substr(0, separator);
        reconstruction::settings::addOverride(
            key.find('.') == key.npos ? moduleId + "." + value : value);
    }
}

QJsonObject moduleSettingsJson(const reconstruction::settings::Document& document)
{
    QJsonObject output;
    const auto& fields = document.fields();
    for (std::size_t index = 0; index < fields.size(); ++index) {
        const auto& field = fields[index];
        const double value = document.value(index);
        if (field.type == 2)
            output[QString::fromStdString(field.key)] = value != 0;
        else if (field.type == 0)
            output[QString::fromStdString(field.key)] = static_cast<qint64>(std::llround(value));
        else
            output[QString::fromStdString(field.key)] = value;
    }
    return output;
}

namespace {

reconstruction::LengthUnit parseUnit(const QString& text)
{
    if (text == QStringLiteral("mm")) return reconstruction::LengthUnit::Millimeter;
    if (text == QStringLiteral("m")) return reconstruction::LengthUnit::Meter;
    if (text == QStringLiteral("arbitrary")) return reconstruction::LengthUnit::Arbitrary;
    throw std::invalid_argument("--unit must be mm, m or arbitrary");
}

ModuleCloud syntheticBox()
{
    reconstruction::SyntheticBoxRequest request;
    request.box.pose.origin = {-30.0, -20.0, -10.0};
    request.box.sizeX = 60.0;
    request.box.sizeY = 40.0;
    request.box.sizeZ = 20.0;
    request.samplesPerEdge = 24;
    auto store = reconstruction::makeSyntheticBoxSurface(request);
    ModuleCloud cloud;
    cloud.ply.positions.reserve(store.size());
    cloud.ply.normals.reserve(store.size());
    for (const auto& point : store.points()) {
        cloud.ply.positions.push_back({float(point.position.x), float(point.position.y),
                                       float(point.position.z)});
        if (point.normal)
            cloud.ply.normals.push_back({float(point.normal->x), float(point.normal->y),
                                         float(point.normal->z)});
    }
    cloud.store = std::move(store);
    return cloud;
}

}  // namespace

ModuleCloud loadModuleCloud(const ModuleArguments& arguments, QString& error)
{
    if (!arguments.synthetic.isEmpty()) {
        if (arguments.synthetic != QStringLiteral("box"))
            throw std::invalid_argument("--synthetic currently supports only 'box'");
        return syntheticBox();
    }
    if (arguments.input.isEmpty())
        throw std::invalid_argument("Provide --in cloud.ply or --synthetic box");
    ModuleCloud cloud;
    if (!PlyReader::read(arguments.input, cloud.ply, error)) return {};
    const auto unit = arguments.unit.isEmpty()
        ? PointCloudAdapter::lengthUnitFor(arguments.input)
        : parseUnit(arguments.unit);
    cloud.store = PointCloudAdapter::toPointStore(cloud.ply, unit);
    return cloud;
}

double cloudDiagonal(const reconstruction::PointStore& cloud)
{
    if (cloud.empty()) return 0.0;
    auto lower = cloud.points().front().position;
    auto upper = lower;
    for (const auto& point : cloud.points()) {
        lower.x = std::min(lower.x, point.position.x);
        lower.y = std::min(lower.y, point.position.y);
        lower.z = std::min(lower.z, point.position.z);
        upper.x = std::max(upper.x, point.position.x);
        upper.y = std::max(upper.y, point.position.y);
        upper.z = std::max(upper.z, point.position.z);
    }
    const double x = upper.x - lower.x;
    const double y = upper.y - lower.y;
    const double z = upper.z - lower.z;
    return std::sqrt(x * x + y * y + z * z);
}

QString lengthUnitName(reconstruction::LengthUnit unit)
{
    switch (unit) {
    case reconstruction::LengthUnit::Millimeter: return QStringLiteral("mm");
    case reconstruction::LengthUnit::Meter: return QStringLiteral("m");
    case reconstruction::LengthUnit::Arbitrary: return QStringLiteral("arbitrary");
    }
    return QStringLiteral("arbitrary");
}

void writeModuleReport(const QString& path, const QJsonObject& report)
{
    const auto bytes = QJsonDocument(report).toJson(QJsonDocument::Indented);
    if (path.isEmpty()) {
        std::cout << bytes.toStdString();
        return;
    }
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("Cannot write report: " + QFileInfo(path).absoluteFilePath().toStdString());
}

void printModuleReport(const QJsonObject& report)
{
    std::cout << QJsonDocument(report).toJson(QJsonDocument::Indented).toStdString();
}
