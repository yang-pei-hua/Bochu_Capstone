#pragma once
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

// Canonical unit name from the existing reconstruction sidecars. Qt Core only,
// so the document can retain scale metadata without a geometry dependency.
inline QString pointCloudUnitName(const QString& path)
{
    const QFileInfo cloud(path);
    const auto directory = cloud.dir();
    for (const auto& candidate :
         QStringList{directory.filePath("workspace/reconstruction.json"),
                     directory.filePath("reconstruction.json"),
                     directory.filePath(cloud.completeBaseName() + ".json")}) {
        if (!QFileInfo(candidate).isFile())
            continue;
        QFile file(candidate);
        if (!file.open(QIODevice::ReadOnly))
            return QStringLiteral("arbitrary");
        const auto unit =
            QJsonDocument::fromJson(file.readAll()).object()["unit"].toString().trimmed().toLower();
        if (unit == "mm" || unit == "millimeter" || unit == "millimetre")
            return QStringLiteral("mm");
        if (unit == "m" || unit == "meter" || unit == "metre")
            return QStringLiteral("m");
        return QStringLiteral("arbitrary");
    }
    return QStringLiteral("arbitrary");
}
