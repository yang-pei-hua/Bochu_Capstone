"""Repository paths shared by pcad commands, independent of the current directory."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
