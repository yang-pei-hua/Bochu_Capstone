#pragma once

#include "core/Scene.h"

#include <QWidget>

class QTreeWidget;

class ScenePanel final : public QWidget
{
    Q_OBJECT

public:
    explicit ScenePanel(QWidget* parent = nullptr);
    void setScene(const Scene& scene);

    // Renames the first node of the given type. Returns false when there is
    // none. It never reports a selection change.
    bool updateNodeLabel(SceneNodeType type, const QString& label);

signals:
    void nodeSelected(SceneNodeType type);

private:
    QTreeWidget* m_tree = nullptr;
};
