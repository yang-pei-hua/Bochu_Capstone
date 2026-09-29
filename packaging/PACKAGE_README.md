# ParamCAD Studio Windows package

This package is portable: extract it and run `ParamCAD Studio.exe` directly.
To install it for the current Windows user and add a Start menu shortcut, run:

```powershell
.\Install-ParamCADStudio.cmd
```

The installer writes to `%LOCALAPPDATA%\Programs\ParamCAD Studio` and does not
require administrator privileges.

## Package variants

- **full** already includes every end-user runtime: the GUI runtime,
  embeddable Python, and COLMAP. It is ready for reconstruction immediately
  after extraction or installation and does not download anything.
- **bootstrap** keeps those large optional components out of the archive. From
  the extracted folder, download the pinned, SHA-256-verified runtime with:

```powershell
.\Install-Dependencies.cmd
```

After installing the bootstrap package, run the same command from its installed
location:

```powershell
& "$env:LOCALAPPDATA\Programs\ParamCAD Studio\Install-Dependencies.cmd"
```

Alternatively, install the bootstrap package and fetch its dependencies in one
command:

```powershell
.\Install-ParamCADStudio.cmd -InstallDependencies
```

The default COLMAP payload includes CUDA for dense reconstruction. Machines
without a supported NVIDIA GPU can install the smaller CPU build instead:

```powershell
.\Install-Dependencies.cmd -ColmapFlavor nocuda
```

The GUI runtime itself always includes the Qt, VTK, OpenCASCADE, CGAL/GMP, and
MSVC DLLs required to start. CGAL is compiled into the application; its required
`gmp-10.dll` is present in both variants. The downloadable optional payload only
contains Python and COLMAP. Full development SDKs (`include/`, `.lib`, and CMake
package files) are deliberately not shipped to end users.
