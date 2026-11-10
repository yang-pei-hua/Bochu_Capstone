#include "core/Scene.h"
#include <QCoreApplication>

Scene::Scene()
    : m_nodes{
          {QStringLiteral("scene"), QCoreApplication::translate("Scene", "场景"), SceneNodeType::Root, {}},
          {QStringLiteral("models"), QCoreApplication::translate("Scene", "模型"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("model"), QCoreApplication::translate("Scene", "模型"), SceneNodeType::Model, QStringLiteral("models")},
          {QStringLiteral("cameras"), QCoreApplication::translate("Scene", "相机"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("camera-01"), QCoreApplication::translate("Scene", "相机 01"), SceneNodeType::Camera, QStringLiteral("cameras")},
          {QStringLiteral("point-clouds"), QCoreApplication::translate("Scene", "点云"), SceneNodeType::Group, QStringLiteral("scene")},
          {QStringLiteral("point-cloud-01"), QCoreApplication::translate("Scene", "点云"), SceneNodeType::PointCloud, QStringLiteral("point-clouds")}}
{
}
const QVector<SceneNode>& Scene::nodes() const
{
    return m_nodes;
}
