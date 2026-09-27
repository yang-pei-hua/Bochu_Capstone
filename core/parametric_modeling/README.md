# CadModelCore

`CadModelCore` is the Qt/VTK-free V0 parametric modeling core. Its persistent
state is a feature history made of STL/POD values; OpenCASCADE shapes are
derived from that history by `PartDocument::rebuild()`.

## Authoring capabilities

- Sketch entities: reference points, lines, rectangles, and circles.
- Open sketches are valid document state; `validateSketch()` reports whether a
  sketch currently forms an extrudable/cuttable profile.
- Profiles support unordered closed line loops and one outer loop with inner
  holes. Reference points never participate in profile construction.
- Sketch-entity add/edit/remove commands allocate and validate entity IDs in
  the core.
- Extrusion supports NewBody, Join, Cut, and Intersect. Multiple NewBody
  features are returned as a compound through `bodyShape()`.
- Datum, offset-datum, explicit `Plane3d`, and Extrude StartFace/EndFace sketch
  references. An explicit plane lets a caller sketch on an arbitrary planar face
  of the current body; it is a world-space snapshot and does not follow that
  face when an upstream feature changes.
- Feature suppression commands, dependency queries, safe/cascade removal, and
  document clearing.

Stable naming of arbitrary boolean-result faces and versioned Feature Graph
serialization are intentionally not part of this revision; both require a
separate persistent-schema design.

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
