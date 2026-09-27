# Geometric Reconstruction

This module is the Qt/VTK-free boundary between observed point clouds and the
parametric modeling core. It keeps three kinds of state separate:

1. immutable source observations in `PointStore`;
2. fitted `SurfaceEvidence` that references source `PointId` values;
3. accepted semantic primitives committed to `ModelingCore` through an atomic
   `ModelPatch`.

The first vertical slice intentionally recognizes one complete, axis-aligned,
millimeter-scale box. It exists to validate the full contract:

```text
synthetic surface points
  -> six PlaneEvidence records
  -> BoxCandidate
  -> BoxPrimitive model state
  -> OpenCASCADE B-Rep
  -> STEP
```

It does not infer an Extrude history. The next implementation step is to replace
the limited recognizer with plane fitting plus relation analysis while keeping
the candidate and evidence contracts stable.

## Build and test

From the repository root:

```powershell
cmake -S core/geometric_reconstruction `
  -B core/geometric_reconstruction/build-msvc `
  -G "Visual Studio 17 2022" -A x64 `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/geometric_reconstruction/build-msvc --config Release
ctest --test-dir core/geometric_reconstruction/build-msvc `
  -C Release --output-on-failure
```

The test writes `test-output/synthetic_box.step`.
