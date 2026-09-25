#pragma once

#include <QString>
#include <QVector>

enum class SceneNodeType
{
    Root,
    Group,
    Model,
    Camera,
    Light
};
struct SceneNode
{
    QString id;
    QString name;
    SceneNodeType type = SceneNodeType::Group;
    QString parentId;
};

class Scene
{
public:
    Scene();

    const QVector<SceneNode>& nodes() const;

private:
    QVector<SceneNode> m_nodes;
};
