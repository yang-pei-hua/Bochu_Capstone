#pragma once

#include <QString>
#include <vtkSmartPointer.h>

class vtkPolyData;

// Extension point for future STEP/OpenCASCADE and mesh loaders.
// No concrete loader is provided in the UI skeleton.
class ModelLoader
{
public:
    virtual ~ModelLoader() = default;

    virtual bool canLoad(const QString& filePath) const = 0;
    virtual vtkSmartPointer<vtkPolyData> load(const QString& filePath, QString& errorMessage) = 0;
};
