#pragma once
#include "core/CameraController.h"
#include "core/PointCloudDocument.h"
#include "core/PresentationState.h"
#include <modeling/ModelingCore.h>
#include <QTemporaryDir>
#include <QImage>
#include <memory>

struct ProjectViewState
{
    CameraParameters camera;
    double parallelScale = 1, pointSize = 2;
    bool cloudVisible = true, datumPlanes = true, originLock = true;
    int selectionMode = 0;
    ObjectPresentation object;
    RenderPresentation render;
};
struct ProjectSnapshot
{
    std::vector<modeling::Feature> features;
    TopoDS_Shape currentBody;
    const PointCloudDocument* cloud = nullptr;
    ProjectViewState view;
};
struct PreparedProject
{
    std::shared_ptr<QTemporaryDir> assets;
    modeling::ModelingCore model;
    std::unique_ptr<PointCloudDocument> cloud = std::make_unique<PointCloudDocument>();
    TopoDS_Shape currentBody;
    ProjectViewState view;
    QImage texture;
};

// Versioned single-file archive. Save commits atomically; load never touches live
// state and returns only a fully checked, independently rebuilt project.
class ProjectStore
{
  public:
    static bool save(const QString& path, const ProjectSnapshot& state, QString& error);
    static std::unique_ptr<PreparedProject> load(const QString& path, QString& error);
};
