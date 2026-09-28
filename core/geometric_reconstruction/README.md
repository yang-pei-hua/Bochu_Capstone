# Geometric Reconstruction

This module is the Qt/VTK-free boundary between observed point clouds and the
parametric modeling core. It keeps three kinds of state separate:

1. immutable source observations in `PointStore`;
2. fitted `SurfaceEvidence` that references source `PointId` values;
3. accepted semantic primitives committed to `ModelingCore` through an atomic
   `ModelPatch`.

The baseline vertical slice recognizes one complete, axis-aligned,
millimeter-scale box. The general path uses CGAL Efficient RANSAC,
covariance-based plane refinement, and parallel/perpendicular relation analysis
to recover a fully oriented box from noisy observations with outliers.

Primitive proposal has a project-owned boundary backed directly by CGAL 6.2.1.
CGAL detects planes, cylinders, spheres, cones, and tori in one Efficient RANSAC
run; its reordered support indices are mapped back to stable `PointId` values.
Project-side code normalizes the analytic parameters, refines planes with PCA,
refines cylinder and sphere radii over all support points, suppresses duplicate
depth layers, and produces typed evidence for all five primitives. No CGAL type
crosses the public API and there is no native RANSAC fallback.

Both paths validate the same contract:

```text
synthetic surface points
  -> six PlaneEvidence records
  -> BoxCandidate
  -> BoxPrimitive model state
  -> OpenCASCADE B-Rep
  -> STEP
```

It does not infer an Extrude history. `AxisAlignedBoxRecognizer` remains as a
small deterministic baseline; `reconstructBox()` is the current reconstruction
entry point. Neither implementation depends on Qt, VTK, PCL, or client state.

Before detection, `preprocessPointCloud()` provides the scale-aware input path:

- known metre input is converted to the modeling core's millimetres;
- voxel downsampling retains an original representative `PointId` per voxel;
- statistical K-nearest-neighbor filtering removes isolated observations;
- PCA neighborhoods estimate missing normals and local curvature.

The client preserves PLY `nx/ny/nz` attributes and reads the unit from the
adjacent reconstruction manifest. A cloud without metric metadata requires an
explicit user confirmation before its source units are interpreted as
millimetres.

The next steps are partial-visibility handling, point-cloud component
segmentation, primitive-specific robust refinement, and global relations.

## Build and test

From the repository root:

```powershell
cmake -S core/geometric_reconstruction `
  -B core/geometric_reconstruction/build-msvc-cgal `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/geometric_reconstruction/build-msvc-cgal --config Release
ctest --test-dir core/geometric_reconstruction/build-msvc-cgal `
  -C Release --output-on-failure
```

Install `cgal:x64-windows` and `eigen3:x64-windows` in that vcpkg root before
configuration. The test writes `test-output/synthetic_box.step`.

CGAL Shape Detection is GPL-3.0-or-later or commercially licensed. Distribution
of a closed-source build requires the appropriate commercial-license decision.
