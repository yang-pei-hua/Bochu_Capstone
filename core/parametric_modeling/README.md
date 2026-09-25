# CadModelCore

`CadModelCore` is the Qt/VTK-free V0 parametric modeling core. Its persistent
state is a feature history made of STL/POD values; OpenCASCADE shapes are
derived from that history by `PartDocument::rebuild()`.

## Build

The repository-local MSVC x64 SDK is stored at `deps/occt-8.0.1`. Build and
run the smoke test from the repository root with:

```powershell
cmake -S core/parametric_modeling -B core/parametric_modeling/build `
  -G "Visual Studio 17 2022" -A x64 `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/parametric_modeling/build --config Release
ctest --test-dir core/parametric_modeling/build -C Release --output-on-failure
```

The test writes `01_base.step`, `02_modified.step`, `03_cut.step`, and
`04_upstream_rebuilt.step` below `build/test-output`.
