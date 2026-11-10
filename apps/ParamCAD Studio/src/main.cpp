#include "app/MainWindow.h"
#include "app/UiLanguage.h"

#include <reconstruction/settings/ModuleSettings.h>

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QIcon>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include <filesystem>

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ParamCAD Studio"));
    QApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QApplication::setApplicationVersion(QStringLiteral("0.2.2"));
    QApplication::setWindowIcon(QIcon(QStringLiteral(":/icons/paramcad-studio.png")));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("ParamCAD Studio"));
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption performanceCapture(
        QStringLiteral("performance-capture"),
        QStringLiteral("Capture a STEP model as a self-contained validation case."),
        QStringLiteral("step-file"));
    const QCommandLineOption outputDirectory(
        QStringLiteral("output"), QStringLiteral("Validation case directory."),
        QStringLiteral("directory"));
    const QCommandLineOption texture(
        QStringLiteral("texture"), QStringLiteral("Optional feature-rich surface texture."),
        QStringLiteral("image-file"));
    const QCommandLineOption configDirectory(
        QStringLiteral("config-dir"), QStringLiteral("Directory that holds the module .conf files."),
        QStringLiteral("directory"));
    const QCommandLineOption overrides(
        QStringLiteral("set"), QStringLiteral("Override one hyperparameter (module.key=value). Repeatable."),
        QStringLiteral("module.key=value"));
    const QCommandLineOption language(
        QStringLiteral("lang"), QStringLiteral("Interface language: zh (default) or en."),
        QStringLiteral("language"));
    parser.addOption(performanceCapture);
    parser.addOption(outputDirectory);
    parser.addOption(texture);
    parser.addOption(configDirectory);
    parser.addOption(overrides);
    parser.addOption(language);
    parser.process(application);

    // Catalogs must be installed before the first widget is constructed so
    // every menu entry and panel uses the requested language.
    i18n::installTranslators(application, parser.value(language));

    if (parser.isSet(configDirectory)) {
        reconstruction::settings::setConfigDirectory(
            std::filesystem::path(parser.value(configDirectory).toStdU16String()));
    }
    for (const auto& assignment : parser.values(overrides))
        reconstruction::settings::addOverride(assignment.toStdString());

    MainWindow window;
    window.show();

    if (parser.isSet(performanceCapture)) {
        const QString modelPath = parser.value(performanceCapture);
        const QString outputPath = parser.value(outputDirectory);
        if (modelPath.isEmpty() || outputPath.isEmpty()) {
            parser.showHelp(2);
        }
        application.processEvents();
        QString error;
        const bool succeeded = window.runPerformanceCapture(
            QFileInfo(modelPath).absoluteFilePath(), QFileInfo(outputPath).absoluteFilePath(),
            parser.value(texture).isEmpty()
                ? QString()
                : QFileInfo(parser.value(texture)).absoluteFilePath(),
            error);
        if (!succeeded) {
            window.logError(error);
            return 2;
        }
        return 0;
    }
    return application.exec();
}
