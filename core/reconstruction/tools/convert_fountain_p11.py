#!/usr/bin/env python3
"""Convert the Fountain-P11 GraphView bundle-adjustment solution to CameraInfo."""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path


def _png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError(f"not a supported PNG image: {path}")
    return struct.unpack(">II", header[16:24])


def _rotation_vector_to_qvec(vector: list[float]) -> list[float]:
    angle = math.sqrt(sum(value * value for value in vector))
    if angle < 1e-15:
        return [1.0, 0.0, 0.0, 0.0]
    scale = math.sin(angle / 2.0) / angle
    qvec = [math.cos(angle / 2.0), *(value * scale for value in vector)]
    if qvec[0] < 0:
        qvec = [-value for value in qvec]
    return qvec


def convert(dataset_dir: Path, output: Path, images_dir: Path | None = None) -> None:
    graph_path = dataset_dir / "solution.graph"
    solution_path = dataset_dir / "solution.txt"
    order_path = dataset_dir / "order.txt"

    graph_lines = graph_path.read_text(encoding="utf-8").splitlines()
    vertex_lines = [line for line in graph_lines if line.startswith("VERTEX_")]
    solution_lines = solution_path.read_text(encoding="utf-8").splitlines()
    image_names = [line.strip().replace("\\", "/") for line in order_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(vertex_lines) != len(solution_lines):
        raise ValueError(
            f"vertex/solution count mismatch: {len(vertex_lines)} != {len(solution_lines)}"
        )

    cameras: list[tuple[list[float], list[float], list[float]]] = []
    for graph_line, solution_line in zip(vertex_lines, solution_lines):
        graph_fields = graph_line.split()
        if graph_fields[0] != "VERTEX_CAM":
            continue
        solution = [float(value) for value in solution_line.split()]
        if len(solution) != 6:
            raise ValueError(f"camera solution must have 6 values: {solution_line}")
        # GraphView stores optimized world-to-camera translation followed by
        # the world-to-camera SO(3) rotation vector in solution.txt.
        tvec = solution[:3]
        qvec = _rotation_vector_to_qvec(solution[3:])
        intrinsics = [float(value) for value in graph_fields[9:13]]
        cameras.append((qvec, tvec, intrinsics))

    if len(cameras) != len(image_names):
        raise ValueError(f"camera/image count mismatch: {len(cameras)} != {len(image_names)}")
    if not cameras:
        raise ValueError("no cameras found")

    image_root = images_dir if images_dir is not None else dataset_dir
    first_intrinsics = cameras[0][2]
    first_size = _png_size(image_root / image_names[0])
    for name, (_, _, intrinsics) in zip(image_names, cameras):
        if _png_size(image_root / name) != first_size:
            raise ValueError(f"image size differs from first image: {name}")
        if any(abs(left - right) > 1e-9 for left, right in zip(intrinsics, first_intrinsics)):
            raise ValueError(f"camera intrinsics differ from first camera: {name}")

    width, height = first_size
    camera_info = {
        "schema_version": 1,
        "source": {
            "dataset": "Fountain-P11",
            "camera": "Canon D60, 20mm",
            "calibration": "GraphView optimized bundle-adjustment solution",
        },
        "cameras": {
            "camera_0": {
                "model": "PINHOLE",
                "width": width,
                "height": height,
                "params": first_intrinsics,
            }
        },
        "images": [
            {
                "name": name,
                "camera": "camera_0",
                "world_to_camera": {"qvec": qvec, "tvec": tvec},
            }
            for name, (qvec, tvec, _) in zip(image_names, cameras)
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(camera_info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_dir", type=Path, help="Extracted Fountain-P11 GraphView directory")
    parser.add_argument("output", type=Path, help="Output CameraInfo JSON")
    parser.add_argument("--images-dir", type=Path, help="Image directory, if separated from calibration files")
    args = parser.parse_args()
    images_dir = args.images_dir.resolve() if args.images_dir else None
    convert(args.dataset_dir.resolve(), args.output.resolve(), images_dir)
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
