#!/usr/bin/env python3
"""Remove generated artifacts from the repository's outputs directory.

Only outputs/.gitignore and outputs/README.md are preserved. The target is
derived from this script's location and cannot be redirected with a command-line
argument, which prevents an accidental cleanup of an unrelated directory.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import stat
import sys


PRESERVED_NAMES = frozenset({".gitignore", "readme.md"})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Delete everything directly below outputs/ except .gitignore and "
            "README.md. Directories are removed recursively."
        )
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="show the top-level entries that would be removed without deleting them",
    )
    return parser.parse_args()


def locate_outputs() -> Path:
    repository_root = Path(__file__).resolve().parent.parent
    outputs = repository_root / "outputs"

    if not (repository_root / "AGENTS.md").is_file():
        raise RuntimeError(f"repository marker is missing: {repository_root / 'AGENTS.md'}")
    if outputs.is_symlink():
        raise RuntimeError(f"refusing to clean a symbolic-link outputs directory: {outputs}")
    if not outputs.is_dir():
        raise RuntimeError(f"outputs directory does not exist: {outputs}")

    for name in (".gitignore", "README.md"):
        marker = outputs / name
        if not marker.is_file() or marker.is_symlink():
            raise RuntimeError(f"required outputs marker is missing or unsafe: {marker}")
    return outputs


def is_preserved(path: Path) -> bool:
    return path.name.casefold() in PRESERVED_NAMES


def remove_readonly(function: object, path: str, _error: object) -> None:
    """Let shutil.rmtree retry a Windows read-only path once."""

    os.chmod(path, stat.S_IWRITE)
    function(path)  # type: ignore[operator]


def remove_entry(path: Path) -> None:
    # Never traverse a symlink or Windows junction. Remove the link itself.
    if path.is_symlink():
        path.unlink()
        return
    is_junction = getattr(os.path, "isjunction", None)
    if is_junction and is_junction(path):
        os.rmdir(path)
        return
    if path.is_dir():
        shutil.rmtree(path, onerror=remove_readonly)
    else:
        try:
            path.unlink()
        except PermissionError:
            path.chmod(stat.S_IWRITE)
            path.unlink()


def main() -> int:
    args = parse_args()
    try:
        outputs = locate_outputs()
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    targets = sorted(
        (entry for entry in outputs.iterdir() if not is_preserved(entry)),
        key=lambda entry: entry.name.casefold(),
    )
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
        except Exception as error:  # report every failed top-level target
            failures.append((target, error))

    if failures:
        for target, error in failures:
            print(f"error: failed to remove {target}: {error}", file=sys.stderr)
        print(
            f"Cleanup incomplete: {len(failures)} of {len(targets)} entries failed.",
            file=sys.stderr,
        )
        return 1

    print(
        f"Cleanup complete. Preserved {outputs / '.gitignore'} and "
        f"{outputs / 'README.md'}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
