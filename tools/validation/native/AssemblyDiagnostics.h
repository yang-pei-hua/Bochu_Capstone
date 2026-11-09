#pragma once
#include "io/PlyReader.h"
#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <QJsonObject>
#include <QStringList>
QJsonObject writeAssemblyDiagnostics(const QString& reportPath,const PlyCloud& observations,
    const reconstruction::AssemblyReconstructionResult& result,const QString& unit);
QJsonObject projectionAxisDiagnostics(const reconstruction::ProjectionExtrusionResult& fit);
// Offline parameter probes; uses the same observation-only assembly implementation.
void probeAssemblyBoundaries(const QStringList& args,QJsonObject& report);
