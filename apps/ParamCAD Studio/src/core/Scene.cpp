#include "core/Scene.h"

Scene::Scene()
    : m_nodes{
          {QStringLiteral("scene"), QStringLiteral("Scene"), SceneNodeType::Root, {}},
          {QStringLiteral("models"), QStringLiteral("Models"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("model"), QStringLiteral("Model"), SceneNodeType::Model, QStringLiteral("models")},
          {QStringLiteral("cameras"), QStringLiteral("Cameras"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("camera-01"), QStringLiteral("Camera01"), SceneNodeType::Camera, QStringLiteral("cameras")},
          {QStringLiteral("point-clouds"), QStringLiteral("Point Clouds"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("point-cloud-01"), QStringLiteral("PointCloud"), SceneNodeType::PointCloud, QStringLiteral("point-clouds")}}
{
}
const QVector<SceneNode>& Scene::nodes() const
{
    return m_nodes;
}
