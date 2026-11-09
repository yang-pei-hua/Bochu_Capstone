#include "ui/HyperparameterSettings.h"

#include <reconstruction/recognition/ProjectionSettings.h>

#include <QCheckBox>
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

#include <functional>
#include <stdexcept>

void showHyperparameterSettings(QWidget* parent)
{
    using namespace reconstruction;
    if (auto* existing = parent->findChild<QDialog*>(QStringLiteral("hyperparameterSettingsDialog"))) {
        existing->show();
        existing->raise();
        existing->activateWindow();
        return;
    }
    auto* dialog = new QDialog(parent);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setObjectName(QStringLiteral("hyperparameterSettingsDialog"));
    dialog->setWindowTitle(QObject::tr("超参数"));
    dialog->resize(860,650);

    auto* layout = new QVBoxLayout(dialog);
    auto* modules = new QListWidget(dialog);
    modules->setObjectName(QStringLiteral("hyperparameterModules"));
    modules->addItem(QObject::tr("投影重建"));
    modules->setFixedWidth(155);
    auto* pages = new QStackedWidget(dialog);
    QObject::connect(modules, &QListWidget::currentRowChanged, pages, &QStackedWidget::setCurrentIndex);
    auto* columns = new QHBoxLayout;
    columns->addWidget(modules);
    columns->addWidget(pages,1);
    layout->addLayout(columns,1);

    auto* scroll = new QScrollArea(pages);
    scroll->setWidgetResizable(true);
    auto* page = new QWidget(scroll);
    auto* formGroups = new QVBoxLayout(page);
    scroll->setWidget(page);
    pages->addWidget(scroll);
    modules->setCurrentRow(0);

    const auto path = projectionConfigPath();
    const auto filename = QString::fromStdU16String(path.u16string());
    ProjectionExtrusionOptions options;
    try {
        options = loadProjectionOptions(path);
    } catch (const std::exception& error) {
        auto* message = new QLabel(QString::fromUtf8(error.what()),page);
        message->setTextFormat(Qt::PlainText);
        message->setWordWrap(true);
        formGroups->addWidget(message);
    }
    auto* location = new QLabel(filename,dialog);
    location->setTextFormat(Qt::PlainText);
    location->setTextInteractionFlags(Qt::TextSelectableByMouse);
    location->setWordWrap(true);
    layout->addWidget(location);

    using ReadSetting = std::function<void(ProjectionExtrusionOptions&)>;
    std::vector<ReadSetting> readers;
    QString previousGroup;
    QFormLayout* fields = nullptr;
    for (const auto& setting : projectionSettings()) {
        const auto group = QString::fromUtf8(setting.group);
        if (group != previousGroup) {
            previousGroup = group;
            auto* box = new QGroupBox(group,page);
            fields = new QFormLayout(box);
            formGroups->addWidget(box);
        }
        const auto label = QString::fromUtf8(setting.label);
        const auto key = QString::fromLatin1(setting.key);
        const double value = setting.value(options);
        QWidget* control = nullptr;
        switch (setting.type) {
        case ProjectionSetting::Type::Integer: {
            auto* spin = new QSpinBox(page);
            spin->setRange(int(setting.minimum),int(setting.maximum));
            spin->setValue(int(value));
            readers.push_back([spin, setting](auto& next) { setting.assign(next,spin->value()); });
            control = spin;
            break;
        }
        case ProjectionSetting::Type::Boolean: {
            auto* check = new QCheckBox(page);
            check->setChecked(value != 0);
            readers.push_back([check, setting](auto& next) { setting.assign(next,check->isChecked()); });
            control = check;
            break;
        }
        case ProjectionSetting::Type::Real: {
            const auto initial = QString::number(value,'g',16);
            auto* edit = new QLineEdit(initial,page);
            readers.push_back([edit, key, initial, value, setting](auto& next) {
                if (edit->text() == initial) setting.assign(next,value);
                else setProjectionOption(next,(key+QLatin1Char('=')+edit->text()).toStdString());
            });
            control = edit;
            break;
        }
        }
        control->setObjectName(key);
        control->setToolTip(QObject::tr("%1 · 范围 %2 至 %3").arg(key).arg(setting.minimum).arg(setting.maximum));
        fields->addRow(label,control);
    }
    formGroups->addStretch();

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel,dialog);
    layout->addWidget(buttons);
    QObject::connect(buttons,&QDialogButtonBox::rejected,dialog,&QDialog::reject);
    QObject::connect(buttons,&QDialogButtonBox::accepted,dialog,[dialog,filename,readers=std::move(readers)] {
        try {
            ProjectionExtrusionOptions next;
            for (const auto& read : readers) read(next);
            const auto bytes = QByteArray::fromStdString(serializeProjectionOptions(next));
            if (!QDir().mkpath(QFileInfo(filename).absolutePath()))
                throw std::runtime_error("Cannot create the config directory");
            QSaveFile file(filename);
            if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
                throw std::runtime_error(file.errorString().toStdString());
            dialog->accept();
        } catch (const std::exception& error) {
            QMessageBox::warning(dialog,QObject::tr("无法保存超参数"),QString::fromUtf8(error.what()));
        }
    });
    dialog->show();
}
