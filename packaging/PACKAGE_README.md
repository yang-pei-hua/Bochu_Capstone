# ParamCAD Studio Windows package

This package is portable: extract it and run `ParamCAD Studio.exe` directly.
To install it for the current Windows user and add a Start menu shortcut, run:

```powershell
.\Install-ParamCADStudio.cmd
```

The installer writes to `%LOCALAPPDATA%\Programs\ParamCAD Studio` and does not
require administrator privileges.

## Package variants

- **offline** already includes the embeddable Python runtime and COLMAP. It is
  ready for reconstruction immediately after extraction or installation.
- **bootstrap** keeps those large optional components out of the archive. From
  the extracted or installed folder, download the pinned, SHA-256-verified
  runtime with:

```powershell
.\Install-Dependencies.cmd
```

The default COLMAP payload includes CUDA for dense reconstruction. Machines
without a supported NVIDIA GPU can install the smaller CPU build instead:

```powershell
.\Install-Dependencies.cmd -ColmapFlavor nocuda
```

The GUI runtime itself always includes the Qt, VTK, OpenCASCADE, and MSVC DLLs
required to start. The optional payload only contains Python and COLMAP. Full
development SDKs (`include/`, `.lib`, and CMake package files) are deliberately
not shipped to end users.
