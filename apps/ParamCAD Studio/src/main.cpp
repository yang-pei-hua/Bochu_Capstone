#include "app/MainWindow.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ParamCAD Studio"));
    QApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QApplication::setApplicationVersion(QStringLiteral("0.1.0"));

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
    parser.addOption(performanceCapture);
    parser.addOption(outputDirectory);
    parser.addOption(texture);
    parser.process(application);

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
