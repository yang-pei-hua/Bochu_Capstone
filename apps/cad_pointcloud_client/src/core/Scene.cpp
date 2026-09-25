#include "core/Scene.h"

Scene::Scene()
    : m_nodes{
          {QStringLiteral("scene"), QStringLiteral("Scene"), SceneNodeType::Root, {}},
          {QStringLiteral("models"), QStringLiteral("Models"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("demo-model"), QStringLiteral("DemoModel"), SceneNodeType::Model, QStringLiteral("models")},
          {QStringLiteral("cameras"), QStringLiteral("Cameras"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("camera-01"), QStringLiteral("Camera01"), SceneNodeType::Camera, QStringLiteral("cameras")},
          {QStringLiteral("lights"), QStringLiteral("Lights"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("light-01"), QStringLiteral("Light01"), SceneNodeType::Light, QStringLiteral("lights")}}
{
}
const QVector<SceneNode>& Scene::nodes() const
{
    return m_nodes;
}
