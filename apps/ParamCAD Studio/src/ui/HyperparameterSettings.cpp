#include "ui/HyperparameterSettings.h"

#include "app/CaptureSettings.h"
#include "app/SchemaTranslation.h"

#include <reconstruction/recognition/ProjectionSettings.h>
#include <reconstruction/settings/ModuleConfig.h>
#include <reconstruction/settings/ModuleSettings.h>

#include <QCheckBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QSaveFile>
#include <QScrollArea>
#include <QSpinBox>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {

using reconstruction::settings::Document;
using reconstruction::settings::DocumentPtr;
using reconstruction::settings::Field;

std::vector<DocumentPtr> settingsDocuments()
{
    std::vector<DocumentPtr> documents;
    documents.push_back(makeCaptureSettingsDocument());
    documents.push_back(reconstruction::makePreprocessingSettings());
    documents.push_back(reconstruction::makeDetectionSettings());
    documents.push_back(reconstruction::makeAnalysisSettings());
    documents.push_back(reconstruction::makeRecognitionSettings());
    documents.push_back(reconstruction::makeAssemblySettings());
    documents.push_back(reconstruction::makeProjectionSettings());
    return documents;
}

double parseNumber(const QString& text, const QString& key)
{
    const auto raw = text.trimmed().toStdString();
    std::istringstream stream(raw);
    stream.imbue(std::locale::classic());
    double value = 0;
    if (raw.empty() || !(stream >> value) || !(stream >> std::ws).eof())
        throw std::invalid_argument("参数值无效：" + key.toStdString());
    return value;
}

struct DocumentPage {
    DocumentPtr document;
    QString path;
    std::vector<double> initial;
    std::vector<std::function<double()>> readers;
};

}  // namespace

void showHyperparameterSettings(QWidget* parent)
{
    if (auto* existing = parent->findChild<QDialog*>(QStringLiteral("hyperparameterSettingsDialog"))) {
        existing->show();
        existing->raise();
        existing->activateWindow();
        return;
    }
    auto* dialog = new QDialog(parent);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setObjectName(QStringLiteral("hyperparameterSettingsDialog"));
    dialog->setWindowTitle(QCoreApplication::translate("HyperparameterSettings", "超参数"));
    dialog->resize(960, 720);

    auto* layout = new QVBoxLayout(dialog);
    auto* modules = new QListWidget(dialog);
    modules->setObjectName(QStringLiteral("hyperparameterModules"));
    auto* pages = new QStackedWidget(dialog);
    QObject::connect(modules, &QListWidget::currentRowChanged, pages, &QStackedWidget::setCurrentIndex);
    auto* columns = new QHBoxLayout;
    columns->addWidget(modules);
    columns->addWidget(pages, 1);
    layout->addLayout(columns, 1);

    auto* locations = new QLabel(dialog);
    locations->setTextFormat(Qt::PlainText);
    locations->setTextInteractionFlags(Qt::TextSelectableByMouse);
    locations->setWordWrap(true);
    layout->addWidget(locations);

    std::vector<DocumentPage> documentPages;
    for (auto& document : settingsDocuments()) {
        DocumentPage page;
        page.document = std::move(document);
        page.path = QString::fromStdU16String(
            reconstruction::settings::configPath(page.document->id(), page.document->filename()).u16string());

        auto* scroll = new QScrollArea(pages);
        scroll->setObjectName(QStringLiteral("hyperparameterPage_%1").arg(QString::fromStdString(page.document->id())));
        scroll->setWidgetResizable(true);
        auto* container = new QWidget(scroll);
        auto* groups = new QVBoxLayout(container);
        scroll->setWidget(container);
        pages->addWidget(scroll);

        try {
            page.document->load(std::filesystem::path(page.path.toStdU16String()));
            page.document->validate();
        } catch (const std::exception& error) {
            auto* message = new QLabel(QString::fromUtf8(error.what()), container);
            message->setTextFormat(Qt::PlainText);
            message->setWordWrap(true);
            groups->addWidget(message);
        }

        QFormLayout* fields = nullptr;
        std::string previousGroup;
        const auto& schema = page.document->fields();
        const auto& translation = i18n::SchemaTranslation::instance();
        const auto documentId = page.document->id();
        for (std::size_t index = 0; index < schema.size(); ++index) {
            const auto& field = schema[index];
            if (field.group != previousGroup) {
                previousGroup = field.group;
                auto* box = new QGroupBox(
                    translation.groupName(documentId, field.group, field.group), container);
                fields = new QFormLayout(box);
                groups->addWidget(box);
            }
            const auto key = QString::fromStdString(field.key);
            const double value = page.document->value(index);
            QWidget* control = nullptr;
            if (field.type == 2) {
                auto* check = new QCheckBox(container);
                check->setChecked(value != 0);
                page.readers.push_back([check] { return check->isChecked() ? 1.0 : 0.0; });
                control = check;
            } else if (field.type == 0 && field.maximum <= double(std::numeric_limits<int>::max())) {
                auto* spin = new QSpinBox(container);
                spin->setRange(int(field.minimum), int(field.maximum));
                spin->setValue(int(std::llround(value)));
                page.readers.push_back([spin] { return double(spin->value()); });
                control = spin;
            } else {
                auto* edit = new QLineEdit(QString::number(value, 'g', 17), container);
                page.readers.push_back([edit, key] { return parseNumber(edit->text(), key); });
                control = edit;
            }
            control->setObjectName(key);
            control->setToolTip(QCoreApplication::translate("HyperparameterSettings", "%1 · 范围 %2 至 %3")
                                    .arg(key).arg(field.minimum).arg(field.maximum));
            fields->addRow(translation.fieldLabel(documentId, field.key, field.label), control);
            page.initial.push_back(value);
        }
        groups->addStretch();
        documentPages.push_back(std::move(page));
        modules->addItem(translation.moduleTitle(documentPages.back().document->id(),
                                                 documentPages.back().document->title()));
    }
    auto pageModel = std::make_shared<std::vector<DocumentPage>>(std::move(documentPages));
    // Size the module list to its contents so longer English titles are not
    // clipped, while keeping a sane range for short Chinese titles.
    modules->setFixedWidth(std::clamp(modules->sizeHintForColumn(0) + 32, 150, 360));
    modules->setCurrentRow(0);
    if (modules->count() > 0)
        locations->setText(pageModel->front().path);

    QObject::connect(modules, &QListWidget::currentRowChanged, dialog,
                     [pageModel, locations](int row) {
                         if (row >= 0 && std::size_t(row) < pageModel->size())
                             locations->setText((*pageModel)[std::size_t(row)].path);
                     });

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, dialog);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, dialog,
                     [dialog, pageModel] {
                         std::vector<std::pair<QString, QByteArray>> pending;
                         try {
                             for (auto& page : *pageModel) {
                                 bool dirty = false;
                                 for (std::size_t index = 0; index < page.readers.size(); ++index) {
                                     const double value = page.readers[index]();
                                     if (std::abs(value - page.initial[index]) > 0.0) dirty = true;
                                     page.document->setValue(index, value);
                                 }
                                 page.document->validate();
                                 if (!dirty) continue;
                                 pending.emplace_back(page.path,
                                     QByteArray::fromStdString(page.document->serialize()));
                             }
                         } catch (const std::exception& error) {
                             QMessageBox::warning(dialog, QCoreApplication::translate("HyperparameterSettings", "无法保存超参数"),
                                                  QString::fromUtf8(error.what()));
                             return;
                         }
                         try {
                             for (const auto& [path, bytes] : pending) {
                                 if (!QDir().mkpath(QFileInfo(path).absolutePath()))
                                     throw std::runtime_error("无法创建配置目录");
                                 QSaveFile file(path);
                                 if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size()
                                     || !file.commit())
                                     throw std::runtime_error(file.errorString().toStdString());
                             }
                         } catch (const std::exception& error) {
                             QMessageBox::warning(dialog, QCoreApplication::translate("HyperparameterSettings", "无法保存超参数"),
                                                  QString::fromUtf8(error.what()));
                             return;
                         }
                         dialog->accept();
                     });
    dialog->show();
}
