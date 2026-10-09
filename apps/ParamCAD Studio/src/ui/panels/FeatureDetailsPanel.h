#pragma once

#include <modeling/Feature.h>
#include <QWidget>

class QTextBrowser;
class FeatureEditorPanel;
class TextParameterPanel;

// Formats the current core parameters on demand; owns no feature state.
class FeatureDetailsPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit FeatureDetailsPanel(QWidget* parent = nullptr);
    void setFeature(const std::vector<modeling::Feature>& features, modeling::FeatureId id);
    FeatureEditorPanel* editor() const { return m_editor; }
    TextParameterPanel* textEditor() const { return m_textEditor; }

signals:
    void featureRequested(modeling::FeatureId id);

private:
    QTextBrowser* m_details;
    FeatureEditorPanel* m_editor;
    TextParameterPanel* m_textEditor;
};
