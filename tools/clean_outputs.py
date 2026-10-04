#!/usr/bin/env python3
"""Compatibility entry point; prefer pcad clean output [--dry-run]."""

import sys

from pcad_cli.cli import main


if __name__ == "__main__":
    raise SystemExit(main(["clean", "output", *sys.argv[1:]]))
