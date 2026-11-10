#include "app/UiLanguage.h"

#include <QCoreApplication>
#include <QSettings>

#include <cstdio>

namespace {
int g_failures = 0;

void expect(bool condition, const char* message)
{
    if (condition) return;
    std::printf("FAILED: %s\n", message);
    ++g_failures;
}

void expectEqual(const QString& actual, const QString& expected, const char* message)
{
    if (actual == expected) return;
    std::printf("FAILED: %s (expected '%s', got '%s')\n", message,
                qPrintable(expected), qPrintable(actual));
    ++g_failures;
}
}  // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QCoreApplication::setApplicationName(QStringLiteral("ParamCAD Studio Language Test"));

    QSettings settings;
    settings.clear();
    qunsetenv("PARAMCAD_UI_LANGUAGE");

    expectEqual(i18n::normalizeLanguage(QStringLiteral("en")), QStringLiteral("en"), "en is kept");
    expectEqual(i18n::normalizeLanguage(QStringLiteral("EN_us")), QStringLiteral("en"),
                "regional English maps to en");
    expectEqual(i18n::normalizeLanguage(QStringLiteral("zh-CN")), QStringLiteral("zh"),
                "regional Chinese maps to zh");
    expectEqual(i18n::normalizeLanguage(QStringLiteral("fr")), QStringLiteral("zh"),
                "unsupported languages fall back to the source language");
    expectEqual(i18n::normalizeLanguage(QString()), QStringLiteral("zh"), "empty maps to zh");

    // Command line wins over the environment, which wins over the stored value.
    qputenv("PARAMCAD_UI_LANGUAGE", "zh");
    settings.setValue(QStringLiteral("ui/language"), QStringLiteral("zh"));
    settings.sync();
    expectEqual(i18n::resolveLanguage(QStringLiteral("en")), QStringLiteral("en"),
                "command line has the highest priority");
    qputenv("PARAMCAD_UI_LANGUAGE", "en");
    expectEqual(i18n::resolveLanguage(QString()), QStringLiteral("en"),
                "environment overrides the stored preference");
    qunsetenv("PARAMCAD_UI_LANGUAGE");
    expectEqual(i18n::resolveLanguage(QString()), QStringLiteral("zh"),
                "stored preference is used when no override is present");

    // Installing a language updates the reported state and is reversible.
    i18n::installTranslators(application, QStringLiteral("en"));
    expectEqual(i18n::currentLanguage(), QStringLiteral("en"), "English is installed");
    i18n::installTranslators(application, QStringLiteral("zh"));
    expectEqual(i18n::currentLanguage(), QStringLiteral("zh"), "Chinese is installed");

    // Storing normalizes whatever the menu passed in.
    i18n::storeLanguage(QStringLiteral("EN"));
    settings.sync();
    expectEqual(settings.value(QStringLiteral("ui/language")).toString(), QStringLiteral("en"),
                "stored language is normalized");

    // The source strings are Chinese, so an unknown language must not break the
    // fallback path used before a catalog is available.
    expectEqual(QCoreApplication::translate("UiLanguageTest", "回退"),
                QStringLiteral("回退"), "untranslated source text is preserved");

    settings.clear();
    settings.sync();
    if (g_failures != 0) {
        std::printf("%d language check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ui_language_test passed\n");
    return 0;
}
