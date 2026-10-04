"""Clean only generated outputs while preserving repository documentation."""

import argparse
import os
from pathlib import Path
import shutil
import stat
import sys

from .paths import REPOSITORY_ROOT


PRESERVED_NAMES = frozenset({".gitignore", "readme.md"})


def is_link(path: Path) -> bool:
    is_junction = getattr(os.path, "isjunction", None)
    return path.is_symlink() or bool(is_junction and is_junction(path))


def locate_outputs() -> Path:
    outputs = REPOSITORY_ROOT / "outputs"
    if not (REPOSITORY_ROOT / "AGENTS.md").is_file():
        raise RuntimeError(f"repository marker is missing: {REPOSITORY_ROOT / 'AGENTS.md'}")
    if is_link(outputs):
        raise RuntimeError(f"refusing to clean a linked outputs directory: {outputs}")
    if not outputs.is_dir():
        raise RuntimeError(f"outputs directory does not exist: {outputs}")
    if outputs.resolve() != REPOSITORY_ROOT.resolve() / "outputs":
        raise RuntimeError(f"outputs resolves outside the repository: {outputs}")
    for name in (".gitignore", "README.md"):
        marker = outputs / name
        if not marker.is_file() or is_link(marker):
            raise RuntimeError(f"required outputs marker is missing or unsafe: {marker}")
    return outputs


def is_preserved(path: Path) -> bool:
    return path.name.casefold() in PRESERVED_NAMES


def remove_readonly(function: object, path: str, _error: object) -> None:
    """Retry a Windows read-only path once."""
    os.chmod(path, stat.S_IWRITE)
    function(path)  # type: ignore[operator]


def remove_entry(path: Path) -> None:
    # Unlink symlinks and Windows junctions rather than visiting their targets.
    if path.is_symlink():
        path.unlink()
    elif is_link(path):
        os.rmdir(path)
    elif path.is_dir():
        shutil.rmtree(path, onerror=remove_readonly)
    else:
        try:
            path.unlink()
        except PermissionError:
            path.chmod(stat.S_IWRITE)
            path.unlink()


def run(args: argparse.Namespace) -> int:
    outputs = locate_outputs()
    targets = sorted(
        (entry for entry in outputs.iterdir() if not is_preserved(entry)),
        key=lambda entry: entry.name.casefold(),
    )
    # Check all targets before recursive deletion. Linked entries are allowed
    # because remove_entry removes the link itself.
    for target in targets:
        if target.parent.resolve() != outputs.resolve():
            raise RuntimeError(f"cleanup target is outside outputs: {target}")
        if not is_link(target) and not target.resolve().is_relative_to(outputs.resolve()):
            raise RuntimeError(f"cleanup target resolves outside outputs: {target}")
    if not targets:
        print(f"Nothing to remove from {outputs}")
        return 0

    action = "Would remove" if args.dry_run else "Removing"
    print(f"{action} {len(targets)} top-level entries from {outputs}:")
    for target in targets:
        print(f"  {target.name}")
    if args.dry_run:
        return 0

    failures: list[tuple[Path, Exception]] = []
    for target in targets:
        try:
            remove_entry(target)
        except Exception as error:
            failures.append((target, error))
    if failures:
        for target, error in failures:
            print(f"error: failed to remove {target}: {error}", file=sys.stderr)
        print(f"Cleanup incomplete: {len(failures)} of {len(targets)} entries failed.", file=sys.stderr)
        return 1
    print(f"Cleanup complete. Preserved {outputs / '.gitignore'} and {outputs / 'README.md'}.")
    return 0
