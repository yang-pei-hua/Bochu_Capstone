# Repository instructions

- Treat `core/reconstruction/install/colmap/` as an opaque, local runtime dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only the exact `bin/colmap.exe` path for execution or basic file metadata when a task explicitly requires COLMAP runtime validation.

- Treat `apps/cad_pointcloud_client/deps/vtk-9.7.0/` as an opaque, local VTK SDK dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only exact files required for CMake configuration, linking, runtime validation, or basic file metadata.
