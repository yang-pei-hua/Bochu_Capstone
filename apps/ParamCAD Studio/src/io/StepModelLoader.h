#pragma once

#include "io/ModelLoader.h"

// STEP (.step / .stp) importer built on OpenCASCADE's STEPControl_Reader.
//
// The reader runs the whole way to a TopoDS_Shape: the file is read, its roots
// are transferred into a shape, and that shape is validated before it is handed
// back. Every step is checked, so a file that reads badly, transfers no roots or
// yields a null shape is reported as a failure instead of showing up as an empty
// model.
//
// The returned shape is moved so that its centre of mass sits on the origin: a
// STEP file carries the placement of the CAD system that wrote it, and the
// viewport orbits and frames around the origin. The placement the file had is
// therefore not preserved in the shape - distance and size are untouched.
class StepModelLoader final : public ModelLoader
{
public:
    bool canLoad(const QString& filePath) const override;
    bool load(const QString& filePath, LoadedModel& model,
              QString& errorMessage) override;
};
