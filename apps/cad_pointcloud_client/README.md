# CAD & Point Cloud Studio — UI Skeleton

Windows desktop UI skeleton built with C++17, Qt 6 Widgets, CMake, MSVC 2022 x64,
and VTK. The application currently displays and edits a VTK demo cube; the
reconstruction, fitting, AI, and real model-loading algorithms are intentionally
outside this milestone.

## Requirements

- Visual Studio 2022 with **Desktop development with C++**
- CMake 3.24 or newer
- Qt 6.5 or newer, built for MSVC 2022 x64
- The local VTK 9.7.0 Release SDK at `deps/vtk-9.7.0` (built for the same
  compiler and Qt version, including `GUISupportQt`)

Qt and VTK must use the same architecture and compatible MSVC runtimes. A
MinGW Qt package cannot be linked into the MSVC build.

## Configure and build

Run from this directory in PowerShell. `CMakeLists.txt` automatically discovers
the ignored local VTK SDK, so only the Qt package root must be supplied:

```powershell
cmake --preset msvc-debug `
  -DCMAKE_PREFIX_PATH="D:/Qt/6.11.2/msvc2022_64"
cmake --build --preset msvc-release
```

To create an independent build directory that cannot reuse an older VTK cache:

```powershell
cmake -S . -B build/vtk-9.7-release `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_PREFIX_PATH="D:/Qt/6.11.2/msvc2022_64"
cmake --build build/vtk-9.7-release --config Release --parallel 8
```

For VS Code, select the `msvc-debug` configure preset and the Release build
configuration. The bundled SDK currently contains Release VTK libraries.

Before running outside Visual Studio, make Qt and VTK runtime DLLs available on
`PATH`, or deploy them next to the executable. For Qt:

```powershell
& "D:/Qt/6.11.2/msvc2022_64/bin/windeployqt.exe" `
  "build/vtk-9.7-release/Release/CadPointCloudClient.exe"
```

Then add the local VTK and Qt DLL directories to `PATH` and run:

```powershell
$env:Path = "D:/Qt/6.11.2/msvc2022_64/bin;$PWD/deps/vtk-9.7.0/bin;$env:Path"
./build/vtk-9.7-release/Release/CadPointCloudClient.exe
```

`deps/vtk-9.7.0` is intentionally excluded from Git and AI context because it
is a large generated runtime/development dependency.

## Current scope

- Dockable Scene, Properties, and Console panels
- VTK viewport with trackball interaction, demo cube, and orientation axes
- Object transform/display controls with live actor updates
- Editable VTK camera parameters and six standard views
- Perspective/orthographic switching
- Background, lighting, output-size controls, and PNG capture
- Menus, toolbar, pipeline placeholder navigation, dark theme, and status bar
- Abstract `ModelLoader` extension point for future STEP/OBJ/PLY loaders

## Deferred work

- Real STEP/OBJ/PLY parsing and OpenCASCADE integration
- Persistent project save/load
- Multi-view dataset and camera-pose generation
- Point-cloud reconstruction, geometric fitting, and AI editing
- Production scene graph, selection, undo/redo, task progress, FPS, and mesh stats
