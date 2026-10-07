# Third-party components

The release bundles runtime binaries, not development SDKs. Original third-party
notices included in runtime payloads are preserved. Additional license texts from
the producer's installed dependencies are under `docs/licenses/`.

| Component | Role | Upstream |
| --- | --- | --- |
| Qt | Desktop UI and platform/image plugins | https://www.qt.io/ |
| VTK | Point-cloud visualization | https://vtk.org/ |
| OpenCASCADE | CAD geometry and STEP I/O | https://dev.opencascade.org/ |
| CGAL | Geometry recognition; linked into the client | https://www.cgal.org/ |
| GMP / MPFR / Eigen | Geometry arithmetic | https://gmplib.org/ / https://www.mpfr.org/ / https://eigen.tuxfamily.org/ |
| CPython | Bundled reconstruction interpreter | https://www.python.org/ |
| COLMAP | Image-to-point-cloud reconstruction | https://colmap.github.io/ |
| Microsoft Visual C++ runtime | App-local x64 C/C++ runtime | https://learn.microsoft.com/cpp/windows/deployment-in-visual-cpp |

Versions and build provenance are recorded in `package-manifest.json`. CPython's
license is distributed inside its embedded runtime; COLMAP's upstream runtime
payload retains its runtime files and original notices; developer artifacts are
excluded. ParamCAD's project
license is `docs/licenses/ParamCAD-MIT.txt`.
