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

- `*-offline-cuda.zip` contains the GUI runtime, reconstruction scripts,
  embeddable Python, and COLMAP and works without a dependency download.
- `*-bootstrap.zip` contains the independently runnable GUI and a checked
  dependency installer. Run `Install-Dependencies.cmd` after extraction or
  installation to download the pinned Python and COLMAP archives.

Both packages include `Install-ParamCADStudio.cmd`, which installs for the
current user under `%LOCALAPPDATA%\Programs\ParamCAD Studio` and creates a Start
menu shortcut without requiring administrator privileges. Pass
`-ColmapFlavor nocuda` to the packager or dependency installer for the smaller
CPU-only COLMAP runtime.

Qt, VTK, OpenCASCADE, and the MSVC runtime DLLs are always staged beside the
GUI executable because it cannot start without them. The package does not ship
the corresponding development headers, import libraries, or CMake metadata.
Pinned URLs and SHA-256 values live in
[`../../packaging/dependencies.json`](../../packaging/dependencies.json).

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
- Abstract `ModelLoader` extension point for future STEP/OBJ/PLY loaders

Each captured photo gets a `shot_NNN.json` sidecar next to it. Besides the
camera pose it records `render {width, height}` - the render-window size the
output image was stretched from. Reconstruction uses it to derive separate
horizontal and vertical focal lengths; sidecars without it fall back to square
pixels.

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

## Deferred work

- Interactive sketching, face picking, and SolidWorks-style mouse and keyboard
  mapping
- Undo/redo and a persistent feature tree with in-place editing
- STEP export from the client
- Real STEP/OBJ/PLY parsing and persistent project save/load
- Point-cloud meshing, CAD alignment, and AI editing
- Production scene graph, selection, task progress, FPS, and mesh stats
