#pragma once

#include <modeling/Feature.h>
#include <modeling/Id.h>

#include <QWidget>

#include <vector>

class QLabel;
class QTreeWidget;

// Read-only view of the core feature history. It mirrors the features and
// reports the user's selection; it never stores features or touches the core.
class ModelingPanel final : public QWidget
{
    Q_OBJECT

public:
    explicit ModelingPanel(QWidget* parent = nullptr);

    void setStatusText(const QString& text);

    // Read-only mirror of the core feature history. Nothing here is editable and
    // nothing is written back; the panel is a view, not a second model.
    void setFeatures(const std::vector<modeling::Feature>& features);

    // The feature the user picked in the history, or the invalid id when the
    // selection is empty. Deleting is the window's decision because only it can
    // ask the core which features depend on this one.
    modeling::FeatureId selectedFeatureId() const noexcept;

signals:
    void deleteFeatureRequested(modeling::FeatureId featureId);

private:
    QTreeWidget* m_features = nullptr;
    QLabel* m_status = nullptr;
};