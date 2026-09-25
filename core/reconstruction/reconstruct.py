#!/usr/bin/env python3
"""Convenience launcher that works without installing a Python package."""

from colmap_reconstruction.cli import main


if __name__ == "__main__":
    raise SystemExit(main())

