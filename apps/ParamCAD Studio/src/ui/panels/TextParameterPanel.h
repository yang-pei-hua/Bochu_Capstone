#pragma once
#include <QWidget>
class QLineEdit;
class QPlainTextEdit;
class QPushButton;

class TextParameterPanel final : public QWidget {
    Q_OBJECT
public:
    explicit TextParameterPanel(QWidget* parent = nullptr);
    QString input() const;
    void setEditingAllowed(bool allowed);
    void setPreview(const QString& summary);
    void invalidatePreview(const QString& message = {});
signals:
    void previewRequested();
    void applyRequested();
    void inputChanged();
private:
    QLineEdit* m_input;
    QPlainTextEdit* m_message;
    QPushButton* m_preview;
    QPushButton* m_apply;
    bool m_allowed = false;
    bool m_ready = false;
    void updateControls();
};
