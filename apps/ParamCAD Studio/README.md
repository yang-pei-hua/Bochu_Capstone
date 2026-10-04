# CAD & Point Cloud Studio

Windows desktop application built with C++17, Qt 6 Widgets, CMake, MSVC 2022 x64,
VTK, and OpenCASCADE. The viewport renders the body produced by the parametric
modeling core (`core/parametric_modeling`), so sketches, extrudes, and cuts are
created through the core's command API and displayed as a tessellated solid.
Point-cloud preprocessing and CGAL Efficient RANSAC live in
`core/geometric_reconstruction` and expose only project-owned evidence types.

## Requirements

- Visual Studio 2022 with **Desktop development with C++**
- CMake 3.24 or newer
- Qt 6.5 or newer, built for MSVC 2022 x64
- The repository-local VTK 9.7.0 Release SDK at `../../deps/vtk-9.7.0` (built for the same
  compiler and Qt version, including `GUISupportQt`)
- The repository-local OpenCASCADE 8.0.1 MSVC x64 SDK at
  `../../deps/occt-8.0.1`
- vcpkg with `cgal:x64-windows` and `eigen3:x64-windows` installed

Qt and VTK must use the same architecture and compatible MSVC runtimes. A
MinGW Qt package cannot be linked into the MSVC build.

## Configure and build

Run from this directory in PowerShell. `CMakeLists.txt` automatically discovers
the ignored local VTK SDK, so only the Qt package root must be supplied:

```powershell
cmake --preset msvc-debug `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DCMAKE_PREFIX_PATH="D:/Qt/6.11.2/msvc2022_64"
cmake --build --preset msvc-release
```

If a cache is ever suspected of being stale, delete the `build/` directory and
re-run the commands above rather than creating a second build tree — only
`build/msvc-debug` is built and packaged.

For VS Code, select the `msvc-debug` configure preset and the Release build
configuration. The bundled SDK currently contains Release VTK libraries.

The executable is written to `build/msvc-debug/Release/ParamCAD Studio.exe`.
The bundled Qt, VTK, and OpenCASCADE SDKs are not on `PATH`, and Windows resolves
dependency DLLs from the executable's own directory first, so a CMake
`POST_BUILD` step stages every required runtime DLL (plus the Qt platform, style,
image-format, and TLS plugins) next to the executable. Launch it directly:

```powershell
& "./build/msvc-debug/Release/ParamCAD Studio.exe"
```

Do not copy or move the executable out of `build/msvc-debug/Release`: it must stay
beside the staged DLLs, otherwise startup fails with errors such as
`找不到 vtkInteractionWidgets-9.7.dll`.

The binary SDKs under the repository-root `deps/` directory are intentionally
excluded from Git and AI context. See [`../../deps/README.md`](../../deps/README.md)
for the canonical dependency layout and verification commands.

## Windows release packages

The packaging entry point builds the Release client once and emits two ZIP
packages from the same staging logic:

```powershell
& .\packaging\package-client.ps1 `
  -QtRoot "D:\Qt\6.11.2\msvc2022_64" `
  -VcpkgRoot "D:\dev\vcpkg" `
  -Variant All `
  -Clean
```

- `*-full-cuda.zip` contains the GUI runtime, reconstruction scripts,
  embeddable Python, and COLMAP and works without a dependency download.
- `*-bootstrap.zip` contains the independently runnable GUI and a checked
  dependency installer. Run `Install-Dependencies.cmd` after extraction or
  installation to download the pinned Python and COLMAP archives.

Both packages include `Install-ParamCADStudio.cmd`, which installs for the
current user under `%LOCALAPPDATA%\Programs\ParamCAD Studio` and creates a Start
menu shortcut without requiring administrator privileges. Pass
`-ColmapFlavor nocuda` to the packager or dependency installer for the smaller
CPU-only COLMAP runtime.

Qt, VTK, OpenCASCADE, CGAL's GMP runtime, and the MSVC runtime DLLs are always
staged beside the GUI executable because it cannot start without them. CGAL
itself is compiled into the application. The package does not ship the
corresponding development headers, import libraries, or CMake metadata.
Pinned URLs and SHA-256 values live in
[`../../packaging/dependencies.json`](../../packaging/dependencies.json).

## Client architecture

`src/app/MainWindow` is the composition root: it creates the widgets and command
controllers, lays out docks/menus/toolbars, connects scene navigation and camera
display controls, and presents shared logs and status messages. Editing state
and feature-specific signal connections live in the following controllers:

| Controller | Responsibility |
| --- | --- |
| `SketchWorkspaceController` | Plane selection, sketch session/drafts, entity edits, extrude/cut, feature deletion, rebuild feedback and camera restoration |
| `PointCloudWorkspaceController` | Routes panel intents to the cloud document, coordinates asynchronous fitting and photo reconstruction, and guards CAD candidate commits |
| `CaptureWorkspaceController` | Screenshot/photo/orbit intents, capture output settings, shot list, batch feedback and capture distance synchronization |
| `ModelViewController` | Imported B-Rep lifetime, STEP import, model visibility, texture controls and performance capture entry |

Each workspace receives a small `*Ui` aggregate of borrowed view pointers and the
command controllers it needs. Workspaces do not depend on `MainWindow` or each
other. `WorkspaceController` supplies common Qt signals for logs, status and
scene framing; the window routes framing notifications to capture distance
synchronization. Existing strings retain their `MainWindow` translation context.

`ModelingController`, `CaptureController` and `ReconstructionController` remain
the command/process layer. `core/` holds client document and camera state,
`io/` handles adapters/readers, `rendering/` maps geometry to VTK actors, and
`ui/` contains views and viewport interaction. Geometry algorithms continue to
live in the repository's core libraries.

`MainWindow` also creates one `PointCloudDocument` and injects it into the
point-cloud workspace and `VTKViewer`. The document owns the source path and,
through the existing `PointCloudSelection` spatial/edit engine, source
observations, selection, deletion masks, regions, relationships, undo history
and a monotonic revision. Callers receive read-only state or owned working-cloud
snapshots with original point IDs. Selection gestures leave the revision intact;
semantic edits, annotation loads, undo, replacement and clear advance it. Failed
loads preserve the previous document. Annotation JSON and PLY formats are unchanged.

Document commands need only Qt Core and VTK's data/spatial-index modules, without
widgets or rendering. `VTKViewer` converts picking/box gestures into document
selection commands and subscribes to document changes to rebuild its render
cache. The workspace sends edits, file operations and analysis results directly
to the document; it retains no duplicate source path. The viewport owns selection
mode, visibility, box handles and camera state. Injected documents must outlive
their views; a standalone viewport can create its own document.

Qt parents own widgets, workspaces and command controllers. The window deletes
workspaces and joins their workers before deleting the viewport, then the cloud
document. The point-cloud workspace waits for its analysis worker before destruction. Worker results still require the
same document revision before they can be applied. `CapturePlan.h` provides the
shared 62-view plan for interactive and validation captures.

`CadReconstructionTask` owns a point-cloud snapshot and runs preprocessing and
solid recognition on a worker thread. Its progress and results are delivered on
the GUI thread; it never accesses widgets or the modeling document. The workspace
checks cloud, modeling-core and displayed-body revisions before committing, so
point edits, reloads, sketch changes and STEP imports invalidate pending results.
Surface analysis and CAD recognition are serialized within a workspace.

While CAD reconstruction runs, **File → Cancel CAD Reconstruction** and the
relationship panel's **取消 CAD 重建** button cancel the task. Cancellation takes
effect between calculation stages and before result delivery; it does not abort
a geometry-library call midway. The model stays unchanged by cancelled or stale
results, and window destruction joins the worker before destroying its views.

With `CAD_CLIENT_BUILD_TESTS=ON`, `workspace_interaction_test` drives the actual
window's sketch/extrude/cut, import/texture and capture connections, checks the
validation output and exercises destruction while analysis is pending. The
existing `point_cloud_interaction_test` covers cloud edits, stale results and
attaching/destroying a view around an independently edited document.
`point_cloud_document_test` exercises the document without a render window:
source identity, signals, snapshots and original IDs, annotation/PLY round trips,
undo, failed loads and revision invalidation on replacement/clear.
`cad_reconstruction_task_test` covers worker results, dimensional accuracy,
duplicate requests, cancellation (including queued completion), failure paths
and destruction. Workspace interaction also covers cancellation and invalidation
by cloud/model edits, STEP body replacement and edits during scale confirmation.
When metric validation clouds are available, the task test also accepts PLY paths
to replay them through the worker (from this directory):

```powershell
& "./build/msvc-debug/Release/cad_reconstruction_task_test.exe" `
  "../../outputs/performance-validation-box-through-hole/dense_cloud.ply" `
  "../../outputs/performance-validation-l-bracket-gusset/dense_cloud.ply"
```

## Current scope

- Dockable Scene, Properties, Modeling, and Console panels
- Parametric model built through `ModelingCore` commands: rectangle sketch,
  extrude, top-face sketch, and a through-cut hole, rendered via a read-only
  OpenCASCADE → VTK tessellation adapter
- Modeling panel that edits sketch/extrude/cut parameters and rebuilds the body
- VTK viewport with trackball interaction and orientation axes
- Object transform/display controls with live actor updates
- Editable VTK camera parameters and six standard views
- Perspective/orthographic switching
- Background, lighting, output-size controls, and PNG capture
- Reconstruct page (Tools → 点云重建) that runs
  `core/reconstruction/reconstruct.py` on a folder of multi-view images and
  loads the resulting PLY into the viewport, coexisting with the CAD body
- Scale-aware point-cloud preprocessing and CGAL detection of planes,
  cylinders, spheres, cones, and tori
- Menus, toolbar, pipeline placeholder navigation, dark theme, and status bar
- STEP/STP model import (File → Open Model) driven by `StepModelLoader`, which
  keeps the OpenCASCADE B-Rep alongside the mesh the viewport draws
  - a loaded part is translated so its centre of mass sits on the origin - the
    volume centre, or the bounding-box centre for a shape that encloses no
    volume - because the viewport orbits and frames around the origin. The
    placement the file was exported with is not kept; sizes and distances are
    unchanged
- `ModelLoader` extension point for the formats still to come

`manifest.json` version 2 is the authoritative aggregate camera file. Every
`shots[]` entry contains the image name plus complete position, target, up
vector, longitude/latitude, distance, field of view, and projection. The root
`render {width, height}` records the render-window size the output images were
stretched from, which lets reconstruction derive separate horizontal and
vertical focal lengths. Per-image `shot_NNN.json` sidecars are still written for
backwards compatibility. Reconstruction prefers manifest v2 and falls back to
the sidecars for older capture folders.

## Point-cloud selection and template annotations

Open a PLY with **File → Open Point Cloud**, then use **Properties → 点云**
(also opened by selecting the Point Cloud scene node).

- **单点选择**: left click picks a point; Ctrl/Shift + left click adds or removes
  individual points. A plain click replaces the selection.
- **三维框选**: drag the face handles to resize the box, drag a face to rotate it,
  or Shift + left drag to translate it. Click **选取框内点** to select points inside
  the actual 3D volume, including its depth. **框选追加到现有选区** keeps previous
  points; **重置选择框** restores the initial box around the cloud centre.
- Set **邻域半径（点云单位）**, then click **选取附近点 / 向外扩展一轮**. Each press
  unions the Euclidean-radius neighbourhoods of all points selected before that
  press. Newly added points become seeds on the next press. The radius uses the
  original PLY coordinate units; its initial value is 2% of the bounding diagonal.
- Choose a cuboid, sphere, cylinder, cone, plane, or ring template and an optional group
  name, then click **将选中点归入此分组**. Choose an existing group to append points,
  recall its selection, or delete it. Each point has one group; assigning it again
  replaces its previous membership. Selection is orange; groups use distinct
  palette colours. Manual template assignment creates a named point collection;
  fitting its surface is a separate operation in the relationship page.
- **保存标注… / 载入标注…** writes/reads a JSON sidecar with original point indices,
  named groups, fitted parameters, relationships, deletion states, and a SHA-256
  fingerprint of the point positions. Version 2 also remembers deleted automatic
  relationships so updating the graph does not recreate them; version 1 group
  annotations remain readable.
  Import verifies coordinates and ordering before replacing the current labels.
  Labels reset when a new cloud is loaded; save them before changing clouds.

Middle drag rotates the view, right drag pans, and the wheel zooms in both
selection modes. Escape or a right click without dragging returns to model
interaction. Sketch editing also exits point-cloud selection. Capture excludes
the selection box and annotation colours, then restores the editing state.

## Editable point-cloud relationships

The left **结构** dock has two pages: **点云关系** for source point collections and
surface evidence, and **CAD 特征** for the existing parametric modeling history.
Selecting a collection highlights its points; selecting a relationship highlights
both endpoint collections. Expand a collection to see its fitted parameters,
residual, confidence, paginated original point IDs and relationships. The point
lookup field accepts a zero-based ID from the original PLY, including after edits.

- On an imported PLY, click **识别未归属点** to detect plane, sphere, cylinder, cone
  and torus surfaces in the background and map their support back to the original
  points. Photo reconstruction starts this analysis automatically; CAD fitting
  also starts it when at least 20 points remain unassigned. Existing collections
  are preserved, and unknown points remain under **未归属点**. Results are discarded
  if the document changes during analysis. A cuboid collection remains a manual
  whole-object label; its individual faces can be identified as plane collections.
- Under **集合**, edit a name/template, append the current selection, replace a
  collection with the selection, or **重拟合此集合**. Changing memberships marks the
  fit stale and invalidates automatic relationships; a successful refit updates
  parameters while keeping the collection and its point IDs. Refitting requires
  at least 20 points and a supported surface covering at least 70% of the group.
- Under **关系**, add, edit or remove parallel, perpendicular, nearby/adjacent,
  intersecting, coaxial, tangent, equal-radius and coplanar relationships. Inferred
  relationships are geometric evidence under the current tolerances, rather than
  exact CAD boundary topology or constraints that move geometry. Nearby/adjacent
  means sampled point sets are within the distance tolerance. Manual relationships
  remain explicit annotations even when the surface fit changes.
- **分析设置** controls fit distance, relationship distance and angular tolerances.
  Distances use the original PLY coordinate units. Click **更新关系** after adjusting
  settings; previously deleted relationships remain suppressed.
- **删除关系** keeps both collections and their points. **删除集合** removes its
  relationships and releases its points to the unassigned collection.
  **删除集合内所有点** and **删除当前选中的点** remove observations from the working
  cloud: rendering, picking, box selection, expansion, subsequent surface/CAD
  fitting and export all exclude them. These edits support **撤销** (20 steps).
- **保存关系…** saves the same version 2 sidecar as Properties. To restore an edit
  session, open the original PLY and load this sidecar. **导出当前工作点云…** writes a
  separate binary PLY containing only active points, preserving normals/colours
  and writing a unit metadata sidecar. Export compacts IDs and does not replace
  the original PLY; relationship sidecars belong to the original cloud.

With `CAD_CLIENT_BUILD_TESTS=ON`, `point_cloud_selection_test` covers selection and
annotations, `point_cloud_relations_test` covers edits/persistence/deletions,
`point_cloud_analysis_test` covers fitting and original point support after voxel
downsampling, and `point_cloud_interaction_test` exercises the actual window,
point picking, Properties, relationship actions, background results, camera
gestures and capture using a hidden Qt/VTK window (requires working OpenGL).

## Outputs

Everything the client writes stays inside the repository's `outputs/` folder,
grouped by type and then by the timestamp of the batch:

```
outputs/captures/<yyyyMMdd_HHmmss>/         shot_NNN.png + shot_NNN.json + manifest.json
outputs/reconstructions/<yyyyMMdd_HHmmss>/  cloud.ply + camera_info.json + workspace/
outputs/screenshots/<yyyyMMdd_HHmmss>.png   single-frame capture from the toolbar
```

A second capture or run within the same second gets a `-2` suffix. The project
root is found by walking up from the executable until
`core/reconstruction/reconstruct.py` appears; the `QSettings` key
`reconstruction/repoRoot` overrides it. Both panels still allow the folder to be
changed, and if the root cannot be located the fallback is `outputs/` beside the
executable - never the user's Pictures folder.

## Performance-validation capture

The Capture panel's one-click sweep and the deterministic batch entry point use
the same 62-view spherical plan: 12 longitudes at each of -60, -30, 0, +30, and
+60 degrees latitude, followed by one exact top and one exact bottom view. Every
camera looks at the centred model origin. The batch entry point imports one STEP
file, optionally projects a feature-rich texture, and writes `images/`,
`camera_info.json`, and `capture_report.json` directly below the given case
directory:

```powershell
& ".\build\msvc-debug\Release\ParamCAD Studio.exe" `
  --performance-capture ".\part.step" `
  --output ".\outputs\part-validation" `
  --texture ".\texture.png"
```

The case directory may already contain an `input/` folder, but `images/` must
not exist. This prevents an old capture from being silently overwritten. Pass
the generated `images/` and `camera_info.json` to `reconstruct.py`, and keep its
output and workspace in the same case directory to avoid scattering artifacts
across the interactive `captures/` and `reconstructions/` roots.

## Deferred work

- Interactive sketching, face picking, and SolidWorks-style mouse and keyboard
  mapping
- Undo/redo and a persistent feature tree with in-place editing
- STEP export from the client
- Real OBJ/PLY mesh parsing and persistent project save/load
- Point-cloud meshing, CAD alignment, and AI editing
- Production scene graph, selection, task progress, FPS, and mesh stats
