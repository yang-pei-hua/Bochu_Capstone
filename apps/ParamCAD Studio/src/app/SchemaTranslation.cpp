#include "app/SchemaTranslation.h"

#include "app/UiLanguage.h"

#include <reconstruction/settings/ModuleSettings.h>

#include <QFile>
#include <QJsonDocument>

namespace i18n {

SchemaTranslation& SchemaTranslation::instance()
{
    static SchemaTranslation translation;
    if (!translation.m_loaded) translation.reload();
    return translation;
}

void SchemaTranslation::reload()
{
    m_loaded = true;
    m_active = currentLanguage() == QStringLiteral("en");
    m_root = QJsonObject();
    m_path = QString::fromStdU16String(
        (reconstruction::settings::configDirectory() / "i18n" / "en.json").u16string());
    if (!m_active) return;

    QFile file(m_path);
    if (!file.open(QIODevice::ReadOnly)) return;
    const auto document = QJsonDocument::fromJson(file.readAll());
    if (document.isObject()) m_root = document.object();
}

void SchemaTranslation::loadForTesting(const QString& path, bool active)
{
    m_loaded = true;
    m_active = active;
    m_path = path;
    m_root = QJsonObject();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return;
    const auto document = QJsonDocument::fromJson(file.readAll());
    if (document.isObject()) m_root = document.object();
}

QString SchemaTranslation::fromObject(const QJsonObject& object, const QString& key) const
{
    const auto value = object.value(key);
    return value.isString() ? value.toString() : QString();
}

QString SchemaTranslation::lookup(const QString& module, const QString& section,
                                  const QString& key) const
{
    // Module-specific entry first, then the shared section. Sharing keeps the
    // catalog small because most group and field names recur across modules.
    const auto modules = m_root.value(QStringLiteral("modules")).toObject();
    const auto moduleObject = modules.value(module).toObject();
    if (!moduleObject.isEmpty()) {
        const auto scoped = moduleObject.value(section).toObject();
        const auto value = fromObject(scoped, key);
        if (!value.isEmpty()) return value;
    }
    return fromObject(m_root.value(section).toObject(), key);
}

QString SchemaTranslation::moduleTitle(const std::string& module, const std::string& fallback) const
{
    if (!m_active) return QString::fromStdString(fallback);
    const auto modules = m_root.value(QStringLiteral("modules")).toObject();
    const auto value = fromObject(modules.value(QString::fromStdString(module)).toObject(),
                                  QStringLiteral("title"));
    return value.isEmpty() ? QString::fromStdString(fallback) : value;
}

QString SchemaTranslation::groupName(const std::string& module, const std::string& group,
                                     const std::string& fallback) const
{
    if (!m_active) return QString::fromStdString(fallback);
    const auto value = lookup(QString::fromStdString(module), QStringLiteral("groups"),
                              QString::fromStdString(group));
    return value.isEmpty() ? QString::fromStdString(fallback) : value;
}

QString SchemaTranslation::fieldLabel(const std::string& module, const std::string& key,
                                      const std::string& fallback) const
{
    if (!m_active) return QString::fromStdString(fallback);
    const auto value = lookup(QString::fromStdString(module), QStringLiteral("fields"),
                              QString::fromStdString(key));
    return value.isEmpty() ? QString::fromStdString(fallback) : value;
}

QStringList SchemaTranslation::missing(const std::string& module,
                                       const std::vector<reconstruction::settings::Field>& fields) const
{
    QStringList result;
    const auto moduleId = QString::fromStdString(module);
    const auto modules = m_root.value(QStringLiteral("modules")).toObject();
    const auto moduleObject = modules.value(moduleId).toObject();
    if (fromObject(moduleObject, QStringLiteral("title")).isEmpty())
        result << QStringLiteral("modules.%1.title").arg(moduleId);
    QStringList reportedGroups;
    for (const auto& field : fields) {
        const auto group = QString::fromStdString(field.group);
        if (!reportedGroups.contains(group)) {
            reportedGroups << group;
            if (lookup(moduleId, QStringLiteral("groups"), group).isEmpty())
                result << QStringLiteral("groups.%1").arg(group);
        }
        const auto key = QString::fromStdString(field.key);
        if (lookup(moduleId, QStringLiteral("fields"), key).isEmpty())
            result << QStringLiteral("fields.%1").arg(key);
    }
    return result;
}

}  // namespace i18n
