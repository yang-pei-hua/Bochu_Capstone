#pragma once
#include <modeling/Feature.h>
#include <modeling/ModelPatch.h>
#include <QWidget>
#include <array>
#include <optional>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

class FeatureEditorPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit FeatureEditorPanel(QWidget* parent = nullptr);
    void setFeature(const modeling::Feature* feature, modeling::ModelRevision revision);
    void setEditingAllowed(bool allowed);
    void setAnchor(modeling::ResizeAnchor anchor);
    void setMessage(const QString& message);
    modeling::FeatureId featureId() const;
signals:
    void applyRequested(modeling::FeatureId id, double first, double second, double third,
                        modeling::ResizeAnchor anchor, quint64 revision);
    void selectReferenceRequested();
    void cancelRequested();
private:
    void resetFields();
    void updateControls();
    std::optional<modeling::Feature> m_feature;
    modeling::ModelRevision m_revision = 0;
    QComboBox* m_anchor;
    std::array<QDoubleSpinBox*,3> m_values;
    std::array<QLabel*,3> m_labels;
    std::array<double,3> m_committedValues{}, m_displayedValues{};
    QLabel* m_message;
    QPushButton* m_apply;
    QPushButton* m_choose;
    bool m_allowed = true;
};
