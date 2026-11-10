#pragma once

#include <QJsonObject>
#include <QString>
#include <QStringList>

#include <string>
#include <vector>

namespace reconstruction::settings {
struct Field;
}

namespace i18n {

// English text for hyperparameter metadata (module titles, group headings and
// field labels). The metadata lives in the Qt-free reconstruction core as
// Chinese literals, so it cannot be run through Qt's translator; instead this
// lookup reads config/i18n/en.json, which ships beside the .conf files.
//
// Adding a module therefore only requires a new "modules.<id>.title" entry and
// translations for the group/label strings it introduces; anything that is not
// listed falls back to the Chinese source text.
class SchemaTranslation {
public:
    static SchemaTranslation& instance();

    // Re-reads <config>/i18n/en.json. Active only when the UI language is
    // English; an unreadable catalog leaves every lookup on its fallback.
    void reload();
    bool active() const { return m_active; }
    QString sourcePath() const { return m_path; }

    QString moduleTitle(const std::string& module, const std::string& fallback) const;
    QString groupName(const std::string& module, const std::string& group,
                      const std::string& fallback) const;
    QString fieldLabel(const std::string& module, const std::string& key,
                       const std::string& fallback) const;

    // Keys that have no English entry for the given document. Empty when the
    // catalog is complete; used by schema_translation_test --dump-missing.
    QStringList missing(const std::string& module,
                        const std::vector<reconstruction::settings::Field>& fields) const;

    // Test hook: load a specific catalog and force the lookups on.
    void loadForTesting(const QString& path, bool active);

private:
    SchemaTranslation() = default;

    QString lookup(const QString& module, const QString& section, const QString& key) const;
    QString fromObject(const QJsonObject& object, const QString& key) const;

    QJsonObject m_root;
    QString m_path;
    bool m_active = false;
    bool m_loaded = false;
};

}  // namespace i18n
