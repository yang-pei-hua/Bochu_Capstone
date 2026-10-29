#pragma once
#include <QJsonObject>
#include <QStringList>

// Reference labels are written separately; the PLY contains positions/normals only.
void sampleStepCloud(const QStringList& args, QJsonObject& report);
void inspectStepTopology(const QStringList& args,QJsonObject& report);
