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

signals:
    void nodeSelected(SceneNodeType type);

private:
    QTreeWidget* m_tree = nullptr;
};
