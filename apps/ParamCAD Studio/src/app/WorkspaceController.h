#pragma once

#include <QCoreApplication>
#include <QObject>
#include <QString>

class QWidget;

// Workspaces report application feedback without depending on MainWindow.
// Views and command controllers are borrowed; QObject parenting owns workspaces.
class WorkspaceController : public QObject
{
    Q_OBJECT
public:
    WorkspaceController(QWidget* dialogParent, QObject* parent)
        : QObject(parent), m_dialogParent(dialogParent) {}

signals:
    void logInfo(const QString& message);
    void logWarning(const QString& message);
    void logError(const QString& message);
    void statusMessage(const QString& message, int timeout);
    void statusTextChanged(const QString& text);
    void sceneFramed();

protected:
    QWidget* const m_dialogParent;
};

// Keep the existing translation context while relocating window workflows.
inline QString workspaceTr(const char* text)
{
    return QCoreApplication::translate("MainWindow", text);
}
