#pragma once

#include <QString>
class TopoDS_Shape;

// Adapts core STEP serialization to Qt's Unicode paths and atomic replacement.
class StepModelExporter
{
public:
    static bool write(const TopoDS_Shape& shape, const QString& path, QString& error);
};
