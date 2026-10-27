# CAD & Point Cloud Studio

Windows desktop application built with C++17, Qt 6 Widgets, CMake, MSVC 2022 x64,
VTK, and OpenCASCADE. The viewport renders the body produced by the parametric
modeling core (`core/parametric_modeling`), so sketches, extrudes, and cuts are
created through the core's command API and displayed as a tessellated solid.
Point-cloud preprocessing and CGAL Efficient RANSAC live in
`core/geometric_reconstruction` and expose only project-owned evidence types.

Version 0.2.2 adds bounded Chinese text parameter commands in the feature tab:
preview edits to boxes, cylinders and through holes, validate the dependency
rebuild on a separate model, then apply through the existing undoable editor.
Examples, unit/anchor semantics and limits are documented in
[`Text_Parameter_Editing.md`](../../docs/Text_Parameter_Editing.md).

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

The application icon is embedded in both the Windows executable and the Qt
window. Editable SVG, PNG and multi-size ICO assets live in `resources/icons/`.
To regenerate them, run `python tools/generate_client_icon.py` from the repository
root with Pillow installed, then rebuild the client. Desktop shortcuts should
target the executable above and use its Release directory as the working directory.

### Keyboard navigation and file drops

With the viewport focused, use the **arrow keys** to rotate around the current
camera target. Tap for one step or hold a key to rotate repeatedly. **Ctrl+Up**
increases the step and **Ctrl+Down** decreases it for both axes. The default is
5 degrees per press; the status bar displays the current step (0.1–45 degrees).
Text fields, numeric inputs, lists and menus keep their normal arrow-key behavior.
Loading a file focuses the viewport; use Tab to return to it after editing a field.

Drag one local **STEP/STP or PLY** file into any part of the window to load it with
the existing importer and fit the view. Extension case, spaces and Unicode paths
are supported. PLY imports stay centered on the cloud even when it is away from
the origin. File drops do not start CAD reconstruction. Folders, remote URLs and
multiple-file drops are rejected; the source file is never moved.

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

Select a Box, Through Hole or Cylinder in **CAD 特征 → Features** to edit its
dimensions in **Properties → 特征**. Box dimensions use its local X/Y/Z axes:
the center anchor resizes symmetrically, while a fixed face permits only its
normal dimension. **在视口选择参照面 / 中心** displays the selected primitive's
six colored semantic faces and a yellow center marker, even inside fused geometry.
Rotate the view to pick a face, or choose it from the anchor list; Esc cancels picking.
Cylinder height supports fixed base, top or center; its radius remains concentric.
Through-hole diameter edits preserve the world-space axis. Other features retain
their model coordinates; automatic attachment or edge-distance constraints are not
introduced by resizing.

**应用** commits one atomic CAD transaction; **取消** restores pending fields.
**Edit → 撤销 CAD 编辑 / 重做 CAD 编辑** undo/redo successful CAD changes (up to
50 undo steps per session). Failed commands/rebuilds retain both parameters and
geometry, and stale edits cannot overwrite a newer revision. `.pcad` retains the
committed dimensions, anchors and manual-edit protection, but not undo stacks.
Template-triggered reconstruction pauses when manually edited reconstructed
features exist. Explicit **Reconstruct CAD** asks before replacing them; results
started before a new edit or undo are discarded. STEP import and project replacement
clear CAD undo history; imported STEP geometry has no parametric feature editor.

Planar picks on the current parametric Box and Cylinder attach sketches to
`FeatureId + FaceRole` (six box faces and cylinder base/top). The core resolves
support planes from the owner's current parameters; dependent sketches and their
extrusions/cuts follow resizing, safe deletion reports the entire dependency
chain, and cascade removes it. The preview uses the same resolver. A pick must
match the outward plane and finite primitive bounds; ambiguous coplanar owners
and imported STEP faces remain fixed planes, explicitly reported in the status.
These are source-feature support planes, not persistent IDs of arbitrary faces
created by boolean operations. Existing fixed-plane sketches remain fixed.

Display placement applies equally to bodies, sketches and datum planes. Picking
and drawing remove that placement before creating CAD data, so display rotation
and nonuniform scale do not change model dimensions.

The Scene tree lists the current model, viewport camera and point cloud. Right-click
a model or point cloud to hide/show it or delete it from the project. Hiding preserves
its data; deleting a model clears its feature history, and deleting a point cloud clears
its observations and annotations. Source files are preserved. Empty asset leaves are
omitted, and importing or creating new assets restores their entries.

`src/app/MainWindow` is the composition root: it creates the widgets and command
controllers, lays out docks/menus/toolbars, connects scene navigation and camera
display controls, and presents shared logs and status messages. Editing state
and feature-specific signal connections live in the following controllers:

| Controller | Responsibility |
| --- | --- |
| `SketchWorkspaceController` | Plane selection, sketch session/drafts, entity edits, extrude/cut, feature deletion, rebuild feedback and camera restoration |
| `PointCloudWorkspaceController` | Routes panel intents to the cloud document, coordinates asynchronous fitting and photo reconstruction, and guards CAD candidate commits |
| `CaptureWorkspaceController` | Screenshot/photo/orbit intents, capture output settings, shot list, batch feedback and capture distance synchronization |
| `ModelViewController` | Imported/current B-Rep lifetime, STEP import/export, model visibility, texture controls and performance capture entry |
| `ProjectController` | Project actions, state snapshots, staged restoration, asset lifetime and project path |

Each workspace receives a small `*Ui` aggregate of borrowed view pointers and the
command controllers it needs. Workspaces do not depend on `MainWindow` or each
other. `WorkspaceController` supplies common Qt signals for logs, status and
scene framing; the window routes framing notifications to capture distance
synchronization. Existing strings retain their `MainWindow` translation context.

`ModelingController`, `CaptureController` and `ReconstructionController` remain
the command/process layer. `core/` holds client document and camera state,
`io/` handles adapters/readers/writers and the versioned project archive,
`rendering/` maps geometry to VTK actors, and
`ui/` contains views and viewport interaction. Geometry algorithms continue to
live in the repository's core libraries.

`MainWindow` also creates one `PointCloudDocument` and injects it into the
point-cloud workspace and `VTKViewer`. The document owns the source path, scale metadata and,
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

The toolbar's **Reconstruct CAD** command (also under **Tools**) reconstructs
parametric CAD from the loaded point cloud. It replaces the inactive Geometry
Fitting placeholder. **File** contains opening, closing, saving and exporting;
**Apply Texture** is under **Tools**.

While CAD reconstruction runs, **Tools → Cancel CAD Reconstruction** and the
relationship panel's **取消 CAD 重建** button cancel the task. Cancellation takes
effect between calculation stages and before result delivery; it does not abort
a geometry-library call midway. The model stays unchanged by cancelled or stale
results, and window destruction joins the worker before destroying its views.

**File → Export STEP...** writes the current native CAD geometry in model
coordinates (millimetres), from sketch/extrude/cut, point-cloud reconstruction
or STEP import. The entry is disabled until a body exists and after **Close
Model**. `.step` and `.stp` are accepted, with `.step` added when needed. The
save dialog supplies the standard overwrite confirmation; cancelling leaves
files and the model untouched. If the body changes while choosing a file,
the export is rejected and can be restarted for the new model.

`ModelViewController` retains an OCCT handle from `VTKViewer::bodyShapeChanged`,
so STEP import cannot accidentally export an older body in the modeling core.
The viewport remains a render cache. `io/StepModelExporter` adapts the core's
stream-based AP214 STEP serializer to `QSaveFile`, supporting Unicode paths and
atomic replacement even when writing or committing fails. STEP import also uses
a Unicode-aware input stream. Export contains CAD geometry; camera/display
transforms, textures, point-cloud annotations and editable feature history are
not serialized into STEP. Project persistence remains a separate operation.

With `CAD_CLIENT_BUILD_TESTS=ON`, `workspace_interaction_test` drives the actual
window's sketch/extrude/cut, import/texture and capture connections, checks the
validation output and exercises destruction while analysis is pending. The
existing `point_cloud_interaction_test` covers cloud edits, stale results and
attaching/destroying a view around an independently edited document.
`point_cloud_document_test` exercises the document without a render window:
source identity, signals, snapshots and original IDs, annotation/PLY round trips,
undo, failed loads and revision invalidation on replacement/clear.
`step_model_exporter_test` checks stream failures, Chinese paths, solid/through-hole
round trips and preservation of an existing file when replacement is blocked.
Workspace interaction drives the export menu/dialog for modeled, imported and
reconstructed bodies, extension handling, cancellation and stale/closed models.
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

To replay a metric cloud through the actual window's **Reconstruct CAD** action,
including automatic surface annotation and relationship inference, run:

```powershell
& "./build/msvc-debug/Release/workspace_interaction_test.exe" --replay-cloud `
  "../../outputs/performance-validation-box-through-hole/dense_cloud.ply"
```

The hidden-window replay reports stage timings and the longest GUI event dispatch,
checks the resulting B-Rep and rejects a dispatch lasting five seconds or more.
Relationship adjacency uses radius-limited point queries, so distant dense patches
do not trigger unrestricted nearest-neighbour searches on the GUI thread.

## Current scope

- Dockable Scene, Properties, Modeling, and Console panels
- Parametric model built through `ModelingCore` commands: rectangle sketch,
  extrude, top-face sketch, and a through-cut hole, rendered via a read-only
  OpenCASCADE → VTK tessellation adapter
- Modeling panel that edits sketch/extrude/cut parameters and rebuilds the body
- VTK viewport with trackball interaction and orientation axes
- STEP export of the current native CAD model with Unicode paths and atomic replacement
- Self-contained `.pcad` project save/load with editable feature history, native
  body, original point cloud, annotations, texture and view state
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

## Project save and restore

Use **File → Save** (`Ctrl+S`) to create or update a `.pcad` project, **Save Project
As...** (`Ctrl+Shift+S`) to choose a new destination, and **Open Project...**
(`Ctrl+Shift+O`) to restore it. Opening replaces the current workspace.

The project embeds the editable feature history (stable IDs, names, suppression,
sketch planes/entities and all supported primitive parameters), the current
native B-Rep, the original full cloud including deleted observations, point
deletion masks, regions, relationships, selection, scale metadata and texture.
Camera position/target/up, projection, orthographic scale, origin lock, object
transform/display, cloud visibility/size/mode, datum visibility, background,
lighting, texture projection and capture output size are also restored. The
original source files can be moved or deleted after saving. Native B-Rep storage
preserves placement and the independently imported/displayed body even when it
differs from the feature graph.

Undo history, pending sketch gestures and capture shot lists are transient. A
successful open resets the sketch session and point edit undo. Save requires a
feature history that can rebuild; fix invalid features before saving.

`ProjectStore` stages assets in a temporary directory and writes through
`QSaveFile`, so a failed save preserves the previous file. The version 2 archive (with version 1 read compatibility)
contains an 8-byte `PCADPRJ1` signature, a big-endian 32-bit JSON manifest length,
the manifest, then its ordered assets. Asset sizes and SHA-256 hashes are checked
before decoding; unknown versions, truncated data, invalid geometry/history or
annotations fail before touching live state. Texture decoding also completes
before replacement. The controller retains extracted files for the workspace's
lifetime, cancels pending CAD/surface work and advances both document revisions
so old results cannot overwrite a loaded project. Opening is unavailable during
capture or photo reconstruction.

`project_store_test` covers portable round trips, original point IDs and scale,
editable histories, independent native body placement, views/textures, failed
save protection and malformed archives. `project_interaction_test` uses a hidden
Qt/VTK window to exercise real Save/Save As/Open actions, continued editing,
failed-load preservation, session reset, empty projects and stale CAD results.

## Point-cloud workspace in Properties

Select a row in the left **CAD 特征** list to open **Properties → 特征**.
The read-only details show the current core parameters in millimetres and model
coordinates, before Object display transforms. Box details include local dimensions,
origin, axes, fusion mode and an equivalent rectangle/extrusion recipe. Sketches
show their resolved plane and local/world contour coordinates. Extrudes and cuts
include their source sketch, depth/termination, actual direction and boolean operation;
click the source sketch link to inspect it. Curved primitives and through holes show
their centres/axes and radii. Right-click the details to copy the complete text.
Rebuilding refreshes the selected feature without switching the active Properties
page; deleting it clears the details. These parameters describe the accepted
decomposition, not a recovered original authoring history or a SolidWorks feature export.

Open a PLY with **File → Open Point Cloud** or select the Point Cloud scene node.
All cloud controls live in **Properties → 点云**; the left dock holds **CAD 特征**.
The shared header shows the source file, units, active/deleted point counts and
selection. Four pages separate the workflows:

- **选点**: model interaction, single-point picking or a 3D selection box.
  Ctrl/Shift + left click changes point selection. Box controls appear only in
  box mode: drag faces/handles to resize, drag the body to rotate, and Shift-drag
  to translate. **扩展一轮** uses the neighbourhood radius in original PLY units.
  Enter a zero-based original point ID and press Enter to locate it.
  **选区操作** offers new collections, unassignment, point deletion, undo and clear.
- **集合**: the collection tree is the single editing target. Left click highlights
  its members; double click edits its name/template. Right click offers editing,
  appending/replacing members, template reconstruction, removing the collection
  while keeping points, or removing its points from the working cloud. Right
  clicking preserves the existing 3D selection so append/replace uses the points
  picked by the user. **选区归属** creates or changes membership without a second
  target selector. Templates support boxes, planes, spheres, cylinders, cones and
  tori. Expand fitted parameters, residual/confidence and paginated original IDs.
- **关系**: left click highlights both endpoint collections; right click edits or
  deletes a relationship. **添加关系…** and **更新关系** support parallel,
  perpendicular, nearby/adjacent, intersecting, coaxial, tangent, equal-radius and
  coplanar evidence. Relationships are annotations/evidence and do not move CAD
  geometry. Deleted automatic relationships stay suppressed on later updates.
- **设置**: surface-fit, relationship-distance and angular tolerances. Distance
  values use the original point cloud's coordinate units.

The footer keeps **识别表面**, **重建 CAD** and **更多** visible. Surface recognition
only processes unassigned points and preserves existing collections. CAD rebuild
uses the working cloud and locked templates; its drop-down offers reconstruction
with the selected collection's template. Progress and cancellation remain in the
same workspace. Metric template/membership edits keep automatic reconstruction;
unknown scale requires confirmation through **重建 CAD**. Failed, cancelled or
stale jobs preserve the current CAD. Templates must have sufficient observed
support; missing caps or thickness are not invented. Generated features are
replaced atomically, independent authored history is retained, and authored
dependencies on generated features block replacement.

**更多 → 保存点云标注… / 载入点云标注…** uses the existing version 2 JSON format:
original point IDs, groups, fitted parameters, relationships, deletion masks,
template locks and a coordinate fingerprint. Version 1 remains readable.
Load the matching original PLY before restoring annotations. **导出工作点云…**
writes a separate binary PLY containing active points, colours/normals and a unit
sidecar; exported IDs are compacted, while annotations belong to the original.
Cloud edits support 20 undo steps through **撤销点云编辑**. Project files retain
the source cloud, annotations, templates and reconstruction provenance.

Middle drag rotates, right drag pans and the wheel zooms in both selection modes.
Escape or a right click without dragging in the viewport returns to model
interaction. Sketch mode exits cloud selection. Capture restores the editing
state after omitting the selection box and annotation colours.

The point_cloud_interaction_test exercises the unified Properties pages, actual
picking, creation dialogs, right-click deletion/append with preserved selection,
undo, original-ID lookup, stale background results and hidden-window previews.
The workspace_interaction_test covers guided reconstruction, cancellation and
invalidated CAD results; project_interaction_test verifies project restoration.
The point selection, document, relations and analysis tests cover persistence,
source IDs, deletion masks and evidence mapping without the full GUI.

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

- Stable naming of arbitrary boolean-result faces and references to trimmed
  boundaries; broader feature parameter editing and geometric constraints
- Cross-session undo history and general project-schema migrations
- Real OBJ/PLY mesh parsing
- Point-cloud meshing, CAD alignment, and AI editing
- Production scene graph, selection, task progress, FPS, and mesh stats
