#include "app/ShortcutSettings.h"

#include <QCoreApplication>
#include <QSettings>

#include <cstdio>

namespace {
int g_failures = 0;

void expectEqual(const QKeySequence& actual, const QKeySequence& expected, const char* message)
{
    if (actual == expected) return;
    std::printf("FAILED: %s (expected '%s', got '%s')\n", message,
                qPrintable(expected.toString(QKeySequence::PortableText)),
                qPrintable(actual.toString(QKeySequence::PortableText)));
    ++g_failures;
}
} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QCoreApplication::setApplicationName(QStringLiteral("ParamCAD Studio Shortcut Test"));

    QSettings settings;
    settings.clear();
    settings.sync();

    // A missing entry falls back to the built-in binding.
    expectEqual(shortcuts::screenshotSequence(), QKeySequence(QStringLiteral("Ctrl+P")),
                "missing binding uses Ctrl+P");

    // A stored binding survives a fresh read.
    shortcuts::storeScreenshotSequence(QKeySequence(QStringLiteral("Ctrl+Shift+F12")));
    expectEqual(shortcuts::screenshotSequence(), QKeySequence(QStringLiteral("Ctrl+Shift+F12")),
                "stored binding is read back");

    // An explicitly empty binding means the command is unbound, not defaulted.
    shortcuts::storeScreenshotSequence(QKeySequence());
    expectEqual(shortcuts::screenshotSequence(), QKeySequence(),
                "empty binding disables the shortcut");

    shortcuts::storeScreenshotSequence(shortcuts::defaultScreenshotSequence());
    expectEqual(shortcuts::screenshotSequence(), QKeySequence(QStringLiteral("Ctrl+P")),
                "default binding can be restored");

    settings.clear();
    settings.sync();
    if (g_failures != 0) {
        std::printf("%d shortcut check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("shortcut_settings_test passed\n");
    return 0;
}
