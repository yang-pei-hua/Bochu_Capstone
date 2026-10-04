"""Locate and launch the desktop client without blocking the terminal."""

import argparse
from pathlib import Path
import subprocess

from .paths import REPOSITORY_ROOT


CLIENT_LOCATIONS = (
    "apps/ParamCAD Studio/build/msvc-debug/Release/ParamCAD Studio.exe",
    "apps/ParamCAD Studio/build/msvc-debug/Debug/ParamCAD Studio.exe",
    "build/paramcad-validation/Release/ParamCAD Studio.exe",
    "build/paramcad-studio-cgal/Release/ParamCAD Studio.exe",
    "build/msvc-debug/Release/ParamCAD Studio.exe",
)


def locate_studio(override: str | None = None) -> Path:
    candidates = (
        [Path(override).expanduser().resolve()]
        if override else [REPOSITORY_ROOT / relative for relative in CLIENT_LOCATIONS]
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise RuntimeError(
        "ParamCAD Studio executable not found. Build the Release client or use "
        f"pcad studio --exe <path>. Searched:\n  {searched}"
    )


def run(args: argparse.Namespace) -> int:
    executable = locate_studio(args.exe)
    if args.dry_run:
        print(f"Would open: {executable}")
        return 0
    process = subprocess.Popen(
        [str(executable)], cwd=executable.parent,
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        creationflags=(
            getattr(subprocess, "DETACHED_PROCESS", 0)
            | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        ),
    )
    print(f"Started ParamCAD Studio (PID {process.pid}): {executable}")
    return 0
