#pragma once

#include <QString>

class QCoreApplication;

// UI language selection for ParamCAD Studio.
//
// The interface is authored in Simplified Chinese; English is supplied by the
// Qt translation catalog (translations/paramcad_en.ts) and, for hyperparameter
// metadata that lives in the Qt-free reconstruction core, by config/i18n/en.json.
namespace i18n {

// "zh" or "en". Anything that does not name a supported language yields "zh"
// because the source strings are already Chinese.
QString normalizeLanguage(const QString& value);

// --lang > PARAMCAD_UI_LANGUAGE > QSettings(ui/language) > system locale.
QString resolveLanguage(const QString& commandLineValue);

// Installs the catalogs for the resolved language. Call once, after the
// QApplication (and its organization/application names) exist and before any
// widget is constructed.
void installTranslators(QCoreApplication& application, const QString& commandLineValue);

// The language that is currently installed.
QString currentLanguage();

// Persists the user's choice for the next start.
void storeLanguage(const QString& language);

// Starts a detached copy of the executable and quits this process so the new
// language takes effect. The language environment override is dropped so the
// stored preference wins.
void restartApplication();

}  // namespace i18n
