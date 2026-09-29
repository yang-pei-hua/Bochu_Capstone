# Local native dependencies

All large binary dependencies live below this single repository-root directory.
Their versioned binary folders are ignored by Git; this README and the
verification script are tracked.

## Canonical layout

```text
deps/
├── occt-8.0.1/   OpenCASCADE 8.0.1, MSVC x64 Release
├── vtk-9.7.0/    VTK 9.7.0, MSVC x64 Release with Qt 6 support
└── colmap/        COLMAP 4.2.0 Windows x64 CUDA runtime
```

The whole native stack must use the same ABI:

```text
MSVC 2022 + x64 + Release + dynamic CRT
```

Do not use a MinGW OCCT package with the Qt/VTK desktop application.

## Expected anchors

```text
deps/occt-8.0.1/cmake/OpenCASCADEConfig.cmake
deps/occt-8.0.1/win64/vc14/lib/TKernel.lib
deps/occt-8.0.1/win64/vc14/bin/TKernel.dll
deps/vtk-9.7.0/lib/cmake/vtk-9.7/vtk-config.cmake
deps/colmap/bin/colmap.exe
```

Run the non-recursive dependency check from the repository root:

```powershell
& .\deps\verify.ps1
```

## OpenCASCADE

Use the official `opencascade-8.0.1-vc14-64-combined.zip` archive. The verified
inner archive SHA-256 is:

```text
C60CE7C136ED389B06E05C0D2FFDF40F72928BAF45EBB136FC2D52318DD30B4A
```

Extract its `opencascade-8.0.1-vc14-64` directory as `deps/occt-8.0.1`.
The OCCT distribution was built with optional third-party support, so the
runtime DLLs required by the linked modeling/STEP toolkits are placed beside
the `TK*.dll` files in `win64/vc14/bin`. This keeps the application runtime
PATH to a single OCCT directory.

Point CMake at:

```powershell
-DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
```

## VTK

Install the MSVC x64 Release SDK, built with Qt 6 `GUISupportQt`, directly as
`deps/vtk-9.7.0`. Its `bin` directory must be on `PATH` when starting the
desktop client outside Visual Studio.

## COLMAP

The reproducible download command and SHA/version verification procedure are
documented in [`../docs/colmap_installation.md`](../docs/colmap_installation.md).
Install the extracted runtime directly as `deps/colmap`.
