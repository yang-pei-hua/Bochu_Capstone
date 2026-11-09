#pragma once

#include <QJsonObject>

struct CadReconstructionResult;

QJsonObject projectionDebugReport(const QString& sourcePath, const CadReconstructionResult& result);
