#pragma once
#include <QJsonObject>
#include <QStringList>
void validateSurfaceCoverage(const QStringList& args,QJsonObject& report);
void fitObservedSurface(const QStringList& args, QJsonObject& report);
