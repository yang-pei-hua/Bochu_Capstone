#pragma once

#include <modeling/Feature.h>
#include <QWidget>

class QTextBrowser;

// Formats the current core parameters on demand; owns no feature state.
class FeatureDetailsPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit FeatureDetailsPanel(QWidget* parent = nullptr);
    void setFeature(const std::vector<modeling::Feature>& features, modeling::FeatureId id);

signals:
    void featureRequested(modeling::FeatureId id);

private:
    QTextBrowser* m_details;
};
