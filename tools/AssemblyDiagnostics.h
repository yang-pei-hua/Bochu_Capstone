#pragma once
#include "io/PlyReader.h"
#include <reconstruction/recognition/AssemblyReconstruction.h>
#include <QJsonObject>
QJsonObject writeAssemblyDiagnostics(const QString& reportPath,const PlyCloud& observations,
    const reconstruction::AssemblyReconstructionResult& result,const QString& unit);
