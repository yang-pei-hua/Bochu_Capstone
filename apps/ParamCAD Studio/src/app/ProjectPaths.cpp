#include "app/ProjectPaths.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace {

// The reconstruction pipeline is the one dependency that is always present in a
// checkout and never in an installed copy, which makes it a reliable marker.
constexpr char kRootMarker[] = "core/reconstruction/reconstruct.py";

} // namespace

QString ProjectPaths::projectRoot()
{
    const QSettings settings;
    const QString configured = settings.value(QStringLiteral("reconstruction/repoRoot")).toString();
    if (!configured.isEmpty()
        && QFileInfo::exists(QDir(configured).filePath(QLatin1String(kRootMarker)))) {
        return QDir(configured).absolutePath();
    }

    // The executable lives several levels below the root (build/<preset>/Release),
    // so walk up until the marker shows up.
    QDir directory(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth <= 8; ++depth) {
        if (QFileInfo::exists(directory.filePath(QLatin1String(kRootMarker)))) {
            return directory.absolutePath();
        }
        if (!directory.cdUp()) {
            break;
        }
    }
    return QString();
}

QString ProjectPaths::outputsRoot()
{
    const QString root = projectRoot();
    // Even without a repository root there is no reason to write outside the
    // project, so the folder beside the executable is the fallback.
    const QDir base(root.isEmpty() ? QCoreApplication::applicationDirPath() : root);
    return QDir::cleanPath(base.filePath(QStringLiteral("outputs")));
}

QString ProjectPaths::capturesRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/captures"));
}

QString ProjectPaths::reconstructionsRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/reconstructions"));
}

QString ProjectPaths::screenshotsRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/screenshots"));
}