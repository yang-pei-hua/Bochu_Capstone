#include "app/UiLanguage.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibraryInfo>
#include <QLocale>
#include <QProcess>
#include <QProcessEnvironment>
#include <QSettings>
#include <QTranslator>

namespace i18n {
namespace {

constexpr auto kLanguageKey = "ui/language";
constexpr auto kEnvironmentVariable = "PARAMCAD_UI_LANGUAGE";

QString g_language = QStringLiteral("zh");
QTranslator* g_applicationTranslator = nullptr;
QTranslator* g_qtTranslator = nullptr;

// Returns "zh", "en", or an empty string for "not specified". Keeping the
// unspecified state separate lets each source fall through to the next one.
QString supportedLanguage(const QString& value)
{
    const auto code = value.trimmed().toLower();
    if (code.startsWith(QStringLiteral("en"))) return QStringLiteral("en");
    if (code.startsWith(QStringLiteral("zh")) || code.startsWith(QStringLiteral("cn")))
        return QStringLiteral("zh");
    return {};
}

QString storedLanguage()
{
    // QSettings is only meaningful once an organization and application name
    // are configured; the entry point does that before installing catalogs.
    QSettings settings;
    return supportedLanguage(settings.value(QString::fromLatin1(kLanguageKey)).toString());
}

QString environmentLanguage()
{
    return supportedLanguage(QString::fromLocal8Bit(qgetenv(kEnvironmentVariable)));
}

QString systemLanguage()
{
    return QLocale::system().language() == QLocale::English
        ? QStringLiteral("en") : QStringLiteral("zh");
}

// Catalogs are embedded in the executable by qt_add_translations; a deployed
// build may instead ship them in <app>/translations, and Qt's own catalogs
// live in the SDK. All three are tried in that order.
bool loadCatalog(QTranslator& translator, const QString& fileName)
{
    const QStringList candidates{
        QStringLiteral(":/i18n/%1").arg(fileName),
        QDir(QCoreApplication::applicationDirPath() + QStringLiteral("/translations")).filePath(fileName),
        QDir(QLibraryInfo::path(QLibraryInfo::TranslationsPath)).filePath(fileName),
    };
    for (const auto& candidate : candidates)
        if (QFileInfo::exists(candidate) && translator.load(candidate)) return true;
    return false;
}

void removeTranslator(QTranslator*& translator)
{
    if (!translator) return;
    if (auto* application = qobject_cast<QCoreApplication*>(translator->parent()))
        application->removeTranslator(translator);
    delete translator;
    translator = nullptr;
}

}  // namespace

QString normalizeLanguage(const QString& value)
{
    const auto code = supportedLanguage(value);
    return code.isEmpty() ? QStringLiteral("zh") : code;
}

QString resolveLanguage(const QString& commandLineValue)
{
    const auto requested = supportedLanguage(commandLineValue);
    if (!requested.isEmpty()) return requested;
    const auto environment = environmentLanguage();
    if (!environment.isEmpty()) return environment;
    const auto stored = storedLanguage();
    if (!stored.isEmpty()) return stored;
    return systemLanguage();
}

void installTranslators(QCoreApplication& application, const QString& commandLineValue)
{
    g_language = resolveLanguage(commandLineValue);
    removeTranslator(g_applicationTranslator);
    removeTranslator(g_qtTranslator);

    if (g_language == QStringLiteral("en")) {
        auto* translator = new QTranslator(&application);
        if (loadCatalog(*translator, QStringLiteral("paramcad_en.qm"))) {
            application.installTranslator(translator);
            g_applicationTranslator = translator;
        } else {
            delete translator;
        }
        return;
    }

    // Chinese is the source language, so only Qt's own catalog is needed to
    // translate standard dialog buttons and message boxes.
    auto* translator = new QTranslator(&application);
    if (loadCatalog(*translator, QStringLiteral("qtbase_zh_CN.qm"))) {
        application.installTranslator(translator);
        g_qtTranslator = translator;
    } else {
        delete translator;
    }
}

QString currentLanguage()
{
    return g_language;
}

void storeLanguage(const QString& language)
{
    QSettings settings;
    settings.setValue(QString::fromLatin1(kLanguageKey), normalizeLanguage(language));
    settings.sync();
}

void restartApplication()
{
    QProcess process;
    auto environment = QProcessEnvironment::systemEnvironment();
    environment.remove(QString::fromLatin1(kEnvironmentVariable));
    process.setProcessEnvironment(environment);
    process.setProgram(QCoreApplication::applicationFilePath());
    process.startDetached();
    QCoreApplication::quit();
}

}  // namespace i18n
