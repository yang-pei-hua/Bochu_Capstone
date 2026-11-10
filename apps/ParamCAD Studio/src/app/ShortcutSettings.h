#pragma once

#include <QKeySequence>
#include <QList>

class QWidget;

// User-assigned command shortcuts. Bindings live in QSettings so they survive
// restarts; a missing entry means the built-in default, while an explicitly
// empty sequence means the command has no shortcut.
namespace shortcuts {

QKeySequence defaultScreenshotSequence();
QKeySequence screenshotSequence();
void storeScreenshotSequence(const QKeySequence& sequence);

// Modal editor for the shortcut table. `reserved` lists sequences already owned
// by other window actions; assigning one of them is rejected. Returns true when
// the user accepted a new binding.
bool showShortcutSettings(QWidget* parent, const QList<QKeySequence>& reserved = {});

} // namespace shortcuts
