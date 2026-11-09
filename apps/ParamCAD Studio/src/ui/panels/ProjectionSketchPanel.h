#pragma once

#include <reconstruction/recognition/ProjectionExtrusion.h>

#include <QWidget>

class ProjectionSketchPanel final : public QWidget
{
public:
    ProjectionSketchPanel(std::vector<reconstruction::ProjectionSketchInput> inputs,
                          const reconstruction::ProjectionOperation* preferred, QWidget* parent = nullptr);
};
