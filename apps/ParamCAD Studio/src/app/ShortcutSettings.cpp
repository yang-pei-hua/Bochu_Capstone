#include "app/ShortcutSettings.h"

#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

namespace shortcuts {
namespace {

constexpr auto kScreenshotKey = "shortcuts/screenshot";

QKeySequence storedSequence(const char* key, const QKeySequence& fallback)
{
    const QSettings settings;
    const QString name = QString::fromLatin1(key);
    // An empty stored value is a deliberate "unbound", so only a missing key
    // falls back to the default.
    if (!settings.contains(name)) return fallback;
    return QKeySequence::fromString(settings.value(name).toString(),
                                    QKeySequence::PortableText);
}

void storeSequence(const char* key, const QKeySequence& sequence)
{
    QSettings settings;
    settings.setValue(QString::fromLatin1(key), sequence.toString(QKeySequence::PortableText));
    settings.sync();
}

} // namespace

QKeySequence defaultScreenshotSequence()
{
    return QKeySequence(QStringLiteral("Ctrl+P"));
}

QKeySequence screenshotSequence()
{
    return storedSequence(kScreenshotKey, defaultScreenshotSequence());
}

void storeScreenshotSequence(const QKeySequence& sequence)
{
    storeSequence(kScreenshotKey, sequence);
}

bool showShortcutSettings(QWidget* parent, const QList<QKeySequence>& reserved)
{
    QDialog dialog(parent);
    dialog.setObjectName(QStringLiteral("shortcutSettingsDialog"));
    dialog.setWindowTitle(QCoreApplication::translate("ShortcutSettings", "快捷键设置"));

    auto* layout = new QVBoxLayout(&dialog);
    auto* help = new QLabel(QCoreApplication::translate("ShortcutSettings",
        "为常用命令绑定快捷键。留空表示不绑定。"), &dialog);
    help->setWordWrap(true);
    layout->addWidget(help);

    auto* form = new QFormLayout;
    auto* row = new QWidget(&dialog);
    auto* rowLayout = new QHBoxLayout(row);
    rowLayout->setContentsMargins(0, 0, 0, 0);
    auto* editor = new QKeySequenceEdit(row);
    editor->setObjectName(QStringLiteral("screenshotShortcutEdit"));
    editor->setMaximumSequenceLength(1);
    editor->setKeySequence(screenshotSequence());
    auto* clear = new QPushButton(QCoreApplication::translate("ShortcutSettings", "清除"), row);
    clear->setObjectName(QStringLiteral("clearScreenshotShortcutButton"));
    rowLayout->addWidget(editor, 1);
    rowLayout->addWidget(clear);
    form->addRow(QCoreApplication::translate("ShortcutSettings", "视图截图"), row);
    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::RestoreDefaults,
        &dialog);
    layout->addWidget(buttons);

    QObject::connect(clear, &QPushButton::clicked, editor, &QKeySequenceEdit::clear);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons->button(QDialogButtonBox::RestoreDefaults), &QPushButton::clicked,
                     &dialog, [editor] { editor->setKeySequence(defaultScreenshotSequence()); });
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        const QKeySequence sequence = editor->keySequence();
        if (!sequence.isEmpty() && reserved.contains(sequence)) {
            QMessageBox::warning(&dialog,
                QCoreApplication::translate("ShortcutSettings", "快捷键设置"),
                QCoreApplication::translate("ShortcutSettings",
                    "快捷键 %1 已被其他命令使用，请换一个组合。")
                    .arg(sequence.toString(QKeySequence::NativeText)));
            return;
        }
        storeScreenshotSequence(sequence);
        dialog.accept();
    });

    return dialog.exec() == QDialog::Accepted;
}

} // namespace shortcuts
