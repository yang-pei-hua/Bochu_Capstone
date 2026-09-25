# Repository instructions

- Treat `deps/colmap/` as an opaque, local runtime dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only the exact `bin/colmap.exe` path for execution or basic file metadata when a task explicitly requires COLMAP runtime validation.

- Treat `deps/vtk-9.7.0/` as an opaque, local VTK SDK dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only exact files required for CMake configuration, linking, runtime validation, or basic file metadata.

- Treat `deps/occt-8.0.1/` as an opaque, local OpenCASCADE SDK dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only exact files required for CMake configuration, linking, runtime validation, or basic file metadata.
