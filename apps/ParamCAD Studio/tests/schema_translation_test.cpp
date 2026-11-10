#include "app/CaptureSettings.h"
#include "app/SchemaTranslation.h"

#include <reconstruction/recognition/ProjectionSettings.h>
#include <reconstruction/settings/ModuleConfig.h>
#include <reconstruction/settings/ModuleSettings.h>

#include <QCoreApplication>
#include <QStringList>

#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

// Verifies that config/i18n/en.json covers every module title, group heading
// and field label used by the hyperparameter dialog. Run with --dump-missing to
// list the gaps without failing, which is handy while adding a module.
int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QCoreApplication::setApplicationName(QStringLiteral("ParamCAD Studio Schema Translation Test"));

    const bool dump = argc > 1 && std::strcmp(argv[1], "--dump-missing") == 0;

    auto& translation = i18n::SchemaTranslation::instance();
    translation.loadForTesting(QString::fromUtf8(SCHEMA_I18N_PATH), true);
    if (!translation.active()) {
        std::printf("FAILED: the English schema catalog did not load from %s\n",
                    qPrintable(translation.sourcePath()));
        return 1;
    }

    std::vector<reconstruction::settings::DocumentPtr> documents;
    documents.push_back(makeCaptureSettingsDocument());
    documents.push_back(reconstruction::makePreprocessingSettings());
    documents.push_back(reconstruction::makeDetectionSettings());
    documents.push_back(reconstruction::makeAnalysisSettings());
    documents.push_back(reconstruction::makeRecognitionSettings());
    documents.push_back(reconstruction::makeAssemblySettings());
    documents.push_back(reconstruction::makeProjectionSettings());

    QStringList missing;
    for (const auto& document : documents) {
        for (const auto& key : translation.missing(document->id(), document->fields()))
            missing << QStringLiteral("%1: %2").arg(QString::fromStdString(document->id()), key);
    }
    missing.removeDuplicates();

    if (dump) {
        for (const auto& key : missing) std::printf("%s\n", qPrintable(key));
        std::printf("%d missing schema translation(s)\n", int(missing.size()));
        return 0;
    }

    if (!missing.isEmpty()) {
        std::printf("Missing schema translations:\n");
        for (const auto& key : missing) std::printf("  %s\n", qPrintable(key));
        std::printf("%d missing schema translation(s)\n", int(missing.size()));
        return 1;
    }

    // Spot-check the lookup chain: the module title, a shared group name and a
    // module-scoped field must all differ from their Chinese source text.
    const auto title = translation.moduleTitle("preprocessing", "点云预处理");
    const auto group = translation.groupName("preprocessing", "降采样", "降采样");
    const auto field = translation.fieldLabel("preprocessing", "voxel_size_mm", "体素尺寸 mm，0 为自动");
    if (title == QStringLiteral("点云预处理") || group == QStringLiteral("降采样")
        || field == QStringLiteral("体素尺寸 mm，0 为自动")) {
        std::printf("FAILED: a known catalog entry was not applied\n");
        return 1;
    }

    std::printf("schema_translation_test passed (%d documents)\n", int(documents.size()));
    return 0;
}
