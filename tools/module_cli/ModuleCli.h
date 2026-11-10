#pragma once

#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"

#include <QJsonObject>
#include <QString>
#include <QStringList>

#include <filesystem>
#include <string>
#include <vector>

struct ModuleArguments {
    QString input;
    QString output;
    QString report;
    QString step;
    QString unit;
    QString synthetic;
    QString path;
    std::filesystem::path config;
    std::vector<std::string> overrides;
    bool printConfig = false;
};

// Parses the options shared by every module CLI. Unknown options are rejected
// so a typo never silently falls back to defaults.
ModuleArguments parseModuleArguments(const QStringList& arguments, const QString& usage);

// Converts bare key=value overrides into module.key=value and registers them
// with the shared settings layer. Qualified keys are kept as written.
void prepareModuleOverrides(const std::string& moduleId, const std::vector<std::string>& values);

QJsonObject moduleSettingsJson(const reconstruction::settings::Document& document);

struct ModuleCloud {
    PlyCloud ply;
    reconstruction::PointStore store{std::vector<reconstruction::PointSample>{},
                                     reconstruction::LengthUnit::Millimeter};
};

// Loads a PLY file or the built-in synthetic box used by smoke tests. The
// synthetic cloud is always millimetres and centred on the origin.
ModuleCloud loadModuleCloud(const ModuleArguments& arguments, QString& error);

double cloudDiagonal(const reconstruction::PointStore& cloud);
QString lengthUnitName(reconstruction::LengthUnit unit);

void writeModuleReport(const QString& path, const QJsonObject& report);
void printModuleReport(const QJsonObject& report);
