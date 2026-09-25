"""Command-line entry point for headless reconstruction."""

from __future__ import annotations

import argparse
import sys

from .camera_info import CameraInfoError
from .pipeline import ReconstructionError, reconstruct


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Reconstruct a colored PLY point cloud with headless COLMAP.",
    )
    parser.add_argument("--images", required=True, help="Directory containing input images")
    parser.add_argument("--output", required=True, help="Output .ply file")
    parser.add_argument("--camera-info", help="Optional CameraInfo JSON (intrinsics and optional poses)")
    parser.add_argument("--workspace", help="Empty working directory (default: beside output)")
    parser.add_argument("--colmap", help="Path to the COLMAP executable")
    parser.add_argument("--dense", action="store_true", help="Run GPU PatchMatch and output a dense cloud")
    parser.add_argument(
        "--matcher",
        choices=("exhaustive", "sequential"),
        default="exhaustive",
        help="Use sequential for ordered/video frames; otherwise exhaustive",
    )
    parser.add_argument("--gpu", action="store_true", help="Use GPU for SIFT extraction and matching")
    parser.add_argument(
        "--input-unit",
        choices=("unknown", "mm", "m"),
        default="unknown",
        help="Unit of supplied camera translations; estimated SfM remains scale-ambiguous",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = reconstruct(
            args.images,
            args.output,
            camera_info_path=args.camera_info,
            workspace=args.workspace,
            colmap_executable=args.colmap,
            dense=args.dense,
            matcher=args.matcher,
            use_gpu=args.gpu,
            input_unit=args.input_unit,
        )
    except (CameraInfoError, ReconstructionError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(f"point cloud: {result.point_cloud}")
    print(f"points: {result.point_count}")
    print(f"camera mode: {result.camera_mode}")
    print(f"manifest: {result.manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

