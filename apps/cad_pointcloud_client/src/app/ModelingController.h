#pragma once

#include "app/DemoModelIds.h"

#include <modeling/Feature.h>
#include <modeling/ModelCommand.h>
#include <modeling/ModelingCore.h>

#include <TopoDS_Shape.hxx>

#include <QObject>
#include <QString>

#include <memory>
#include <vector>

// Parameters of the built-in demo model, in millimeters.
struct DemoModelParameters {
    double width = 50.0;
    double height = 30.0;
    double extrusionDepth = 30.0;
    double holeCenterX = 25.0;
    double holeCenterY = 15.0;
    double holeRadius = 5.0;
    double cutDepth = 10.0;
};

// Front-end orchestration layer. It turns UI values into modeling commands and
// publishes the resulting body; it contains no geometry code of its own.
class ModelingController final : public QObject
{
    Q_OBJECT

public:
    explicit ModelingController(QObject* parent = nullptr);

    bool createDemoModel(const DemoModelParameters& parameters);
    bool updateDemoModel(const DemoModelParameters& parameters);

    const TopoDS_Shape& bodyShape() const;
    const std::vector<modeling::Feature>& features() const;
    const QString& lastError() const noexcept;

signals:
    void modelRebuilt();
    void modelError(const QString& message);

private:
    bool addFeatureCommand(const modeling::FeatureParams& params,
                           modeling::FeatureId& createdId);
    bool execute(const modeling::ModelCommand& command,
                 modeling::FeatureId* createdId = nullptr);
    bool rebuildAndPublish();

    std::unique_ptr<modeling::ModelingCore> m_core;
    demo::DemoModelIds m_ids;
    QString m_lastError;
};
