#pragma once
#include <QJsonObject>
#include <QStringList>
// Runs the desktop point-cloud worker with no model or correspondence input.
void validatePointCloudCad(const QStringList& args,QJsonObject& report);
