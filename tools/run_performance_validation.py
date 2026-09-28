#!/usr/bin/env python3
"""Run the complete ParamCAD performance-validation pipeline.

The pipeline keeps every generated artifact inside one case directory:

1. Snapshot the STEP model and texture.
2. Run the deterministic 62-view client capture.
3. Validate the aggregate camera manifest and CameraInfo.
4. Run COLMAP reconstruction (GPU dense by default, CPU sparse on --no-gpu).
5. Run the point-cloud preprocessing and primitive-fitting validation executable.
6. Write machine-readable reports and a concise README.

COLMAP's dense PatchMatch implementation requires CUDA. Therefore --no-gpu
uses the CPU feature/matching path and produces a sparse point cloud instead of
silently using the GPU during the dense stage.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sqlite3
import struct
import subprocess
import sys
import time
from typing import Any, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
OUTPUTS_ROOT = REPOSITORY_ROOT / "outputs"
RECONSTRUCTION_SCRIPT = REPOSITORY_ROOT / "core" / "reconstruction" / "reconstruct.py"
DEFAULT_COLMAP = REPOSITORY_ROOT / "deps" / "colmap" / "bin" / "colmap.exe"
EXPECTED_CAPTURE_COUNT = 62
EXPECTED_RING_ELEVATIONS = (-60, -30, 0, 30, 60)
EXPECTED_AZIMUTHS = tuple(range(0, 360, 30))

CLIENT_CANDIDATES = (
    REPOSITORY_ROOT / "build" / "paramcad-validation" / "Release" / "ParamCAD Studio.exe",
    REPOSITORY_ROOT / "build" / "paramcad-studio-cgal" / "Release" / "ParamCAD Studio.exe",
    REPOSITORY_ROOT / "build" / "msvc-debug" / "Release" / "ParamCAD Studio.exe",
)


class PipelineError(RuntimeError):
    """A user-actionable pipeline failure."""


@dataclass(frozen=True)
class CommandResult:
    elapsed_seconds: float
    output: str


class PipelineLog:
    def __init__(self, path: Path) -> None:
        self.path = path

    def write(self, message: str) -> None:
        line = f"[{datetime.now().astimezone().isoformat(timespec='seconds')}] {message}"
        print(line, flush=True)
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run STEP capture, CameraInfo validation, COLMAP reconstruction, "
            "point-cloud preprocessing/fitting, and report generation."
        ),
    )
    parser.add_argument("--model", required=True, help="Input STEP/STP model")
    parser.add_argument("--texture", required=True, help="Input surface texture image")
    parser.add_argument(
        "--output",
        help=(
            "Case directory below outputs/; default: "
            "outputs/performance-validation-<model>-<timestamp>"
        ),
    )
    gpu = parser.add_mutually_exclusive_group()
    gpu.add_argument(
        "--gpu",
        dest="gpu",
        action="store_true",
        default=True,
        help="Use GPU SIFT and CUDA dense reconstruction (default)",
    )
    gpu.add_argument(
        "--no-gpu",
        dest="gpu",
        action="store_false",
        help="Use CPU SIFT and sparse reconstruction; COLMAP has no CPU PatchMatch path",
    )
    parser.add_argument(
        "--matcher",
        choices=("exhaustive", "sequential"),
        default="exhaustive",
        help="COLMAP feature matcher (default: exhaustive)",
    )
    parser.add_argument("--client", help="ParamCAD Studio executable override")
    parser.add_argument("--colmap", help="COLMAP executable override")
    parser.add_argument(
        "--fit-executable",
        help="point_cloud_io_test executable override",
    )
    parser.add_argument(
        "--progress-seconds",
        type=int,
        default=60,
        help="Print a heartbeat while a stage is running (default: 60; 0 disables)",
    )
    return parser.parse_args(argv)


def resolve_input(value: str, label: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise PipelineError(f"{label} does not exist or is not a file: {path}")
    return path


def ensure_step_model(path: Path) -> None:
    if path.suffix.casefold() not in {".step", ".stp"}:
        raise PipelineError(f"model must be a STEP/STP file: {path}")


def slug(value: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9._-]+", "-", value).strip("-._")
    return normalized or "model"


def resolve_case_directory(argument: str | None, model: Path) -> Path:
    if argument:
        path = Path(argument).expanduser()
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        path = path.resolve()
    else:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = OUTPUTS_ROOT / f"performance-validation-{slug(model.stem)}-{timestamp}"

    try:
        path.relative_to(OUTPUTS_ROOT.resolve())
    except ValueError as error:
        raise PipelineError(f"output must be below {OUTPUTS_ROOT.resolve()}: {path}") from error
    if path.exists():
        if not path.is_dir():
            raise PipelineError(f"output exists but is not a directory: {path}")
        if any(path.iterdir()):
            raise PipelineError(f"output directory is not empty: {path}")
    path.mkdir(parents=True, exist_ok=True)
    return path


def find_client(explicit: str | None) -> Path:
    if explicit:
        return resolve_input(explicit, "client executable")
    for candidate in CLIENT_CANDIDATES:
        if candidate.is_file():
            return candidate.resolve()
    matches = sorted(
        (path for path in (REPOSITORY_ROOT / "build").glob("*/Release/ParamCAD Studio.exe") if path.is_file()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if matches:
        return matches[0].resolve()
    raise PipelineError("cannot find ParamCAD Studio.exe; pass --client")


def find_fit_executable(explicit: str | None, client: Path) -> Path:
    if explicit:
        return resolve_input(explicit, "fit executable")
    beside_client = client.parent / "point_cloud_io_test.exe"
    if beside_client.is_file():
        return beside_client.resolve()
    matches = sorted(
        (path for path in (REPOSITORY_ROOT / "build").glob("*/Release/point_cloud_io_test.exe") if path.is_file()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if matches:
        return matches[0].resolve()
    raise PipelineError("cannot find point_cloud_io_test.exe; pass --fit-executable")


def find_colmap(explicit: str | None) -> Path:
    if explicit:
        return resolve_input(explicit, "COLMAP executable")
    if DEFAULT_COLMAP.is_file():
        return DEFAULT_COLMAP.resolve()
    raise PipelineError(f"cannot find COLMAP at {DEFAULT_COLMAP}; pass --colmap")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PipelineError(f"cannot read valid JSON from {path}: {error}") from error


def command_tail(output: str, lines: int = 30) -> str:
    return "\n".join(output.splitlines()[-lines:])


def hidden_process_options() -> dict[str, Any]:
    if os.name != "nt":
        return {}
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    return {
        "startupinfo": startup,
        "creationflags": subprocess.CREATE_NO_WINDOW,
    }


def run_command(
    stage: str,
    arguments: Sequence[str | Path],
    log_path: Path,
    pipeline_log: PipelineLog,
    progress_seconds: int,
) -> CommandResult:
    command = [str(value) for value in arguments]
    pipeline_log.write(f"{stage}: started; details -> {log_path.name}")
    started = time.monotonic()
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    with log_path.open("w", encoding="utf-8", errors="replace") as log_stream:
        log_stream.write(f"command: {subprocess.list2cmdline(command)}\n\n")
        log_stream.flush()
        process = subprocess.Popen(
            command,
            cwd=REPOSITORY_ROOT,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            env=environment,
            **hidden_process_options(),
        )
        heartbeat = progress_seconds if progress_seconds > 0 else None
        while True:
            try:
                exit_code = process.wait(timeout=heartbeat)
                break
            except subprocess.TimeoutExpired:
                elapsed = time.monotonic() - started
                pipeline_log.write(f"{stage}: still running ({elapsed:.0f}s)")

    elapsed = time.monotonic() - started
    output = log_path.read_text(encoding="utf-8", errors="replace")
    if exit_code != 0:
        raise PipelineError(
            f"{stage} failed with exit code {exit_code}; log: {log_path}\n"
            f"{command_tail(output)}"
        )
    pipeline_log.write(f"{stage}: completed in {elapsed:.3f}s")
    return CommandResult(elapsed_seconds=elapsed, output=output)


def png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise PipelineError(f"capture is not a valid PNG: {path}")
    return struct.unpack(">II", header[16:24])


def vec3(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise PipelineError(f"{field} must be a three-number array")
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise PipelineError(f"{field} must be a three-number array") from error
    if not all(math.isfinite(component) for component in result):
        raise PipelineError(f"{field} contains a non-finite number")
    return result  # type: ignore[return-value]


def validate_capture(case: Path) -> dict[str, Any]:
    images_directory = case / "images"
    manifest = read_json(images_directory / "manifest.json")
    camera_info = read_json(case / "camera_info.json")
    capture_report = read_json(case / "capture_report.json")
    if manifest.get("version") != 2 or not isinstance(manifest.get("shots"), list):
        raise PipelineError("capture manifest must be version 2 with shots[]")

    shots = manifest["shots"]
    png_files = sorted(images_directory.glob("*.png"))
    camera_images = camera_info.get("images")
    cameras = camera_info.get("cameras")
    if not isinstance(camera_images, list):
        raise PipelineError("camera_info.json is missing images[]")
    if not isinstance(cameras, dict) or not cameras:
        raise PipelineError("camera_info.json is missing cameras{}")
    if (
        len(shots) != EXPECTED_CAPTURE_COUNT
        or len(png_files) != len(shots)
        or len(camera_images) != len(shots)
    ):
        raise PipelineError(
            "capture count mismatch: "
            f"expected={EXPECTED_CAPTURE_COUNT}, PNG={len(png_files)}, "
            f"manifest={len(shots)}, CameraInfo={len(camera_images)}"
        )

    radii: list[float] = []
    target_max_abs = 0.0
    views: set[tuple[int, int]] = set()
    elevations: list[float] = []
    for index, shot in enumerate(shots):
        if not isinstance(shot, dict) or not isinstance(shot.get("camera"), dict):
            raise PipelineError(f"manifest shot {index} has no camera object")
        camera = shot["camera"]
        position = vec3(camera.get("position"), f"shots[{index}].camera.position")
        target = vec3(camera.get("target"), f"shots[{index}].camera.target")
        radii.append(math.dist(position, target))
        target_max_abs = max(target_max_abs, *(abs(component) for component in target))
        azimuth = float(camera.get("azimuthDeg", shot.get("azimuthDeg", 0.0)))
        elevation = float(camera.get("elevationDeg", shot.get("elevationDeg", 0.0)))
        rounded_azimuth = int(round(azimuth)) % 360
        rounded_elevation = int(round(elevation))
        views.add((rounded_elevation, rounded_azimuth))
        elevations.append(elevation)
        image_name = shot.get("image")
        if not isinstance(image_name, str) or not (images_directory / image_name).is_file():
            raise PipelineError(f"manifest shot {index} references a missing image")

    if target_max_abs > 1.0e-9:
        raise PipelineError(f"one or more cameras do not target the origin: max={target_max_abs}")
    radius_min = min(radii)
    radius_max = max(radii)
    if radius_max - radius_min > max(1.0e-9, radius_max * 1.0e-9):
        raise PipelineError(f"camera orbit radius is inconsistent: {radius_min}..{radius_max}")

    width, height = png_size(png_files[0])
    for path in png_files[1:]:
        if png_size(path) != (width, height):
            raise PipelineError(f"capture image size differs from the first image: {path}")

    expected_views = {
        (elevation, azimuth)
        for elevation in EXPECTED_RING_ELEVATIONS
        for azimuth in EXPECTED_AZIMUTHS
    }
    expected_views.update({(90, 0), (-90, 0)})
    if views != expected_views:
        missing = sorted(expected_views - views)
        unexpected = sorted(views - expected_views)
        raise PipelineError(
            f"capture plan mismatch; missing={missing}, unexpected={unexpected}"
        )

    png_names = {path.name for path in png_files}
    camera_info_names: set[str] = set()
    for index, image in enumerate(camera_images):
        if not isinstance(image, dict):
            raise PipelineError(f"camera_info images[{index}] must be an object")
        name = image.get("name")
        camera_key = image.get("camera")
        if not isinstance(name, str) or not isinstance(camera_key, str):
            raise PipelineError(f"camera_info images[{index}] has an invalid name or camera")
        if camera_key not in cameras:
            raise PipelineError(f"camera_info images[{index}] references unknown camera {camera_key}")
        camera_info_names.add(name)
    if camera_info_names != png_names:
        raise PipelineError("CameraInfo image names do not exactly match the captured PNG files")
    for camera_key, camera in cameras.items():
        if not isinstance(camera, dict):
            raise PipelineError(f"camera_info camera {camera_key} must be an object")
        if camera.get("width") != width or camera.get("height") != height:
            raise PipelineError(
                f"camera_info camera {camera_key} size does not match PNG size "
                f"{width}x{height}"
            )

    return {
        "images": len(png_files),
        "posed_images": len(camera_images),
        "image_width": width,
        "image_height": height,
        "manifest_version": manifest["version"],
        "camera_distance": radius_min,
        "radius_min": radius_min,
        "radius_max": radius_max,
        "maximum_absolute_target_coordinate": target_max_abs,
        "all_cameras_target_origin": True,
        "azimuths_deg": list(EXPECTED_AZIMUTHS),
        "elevations_deg": elevations,
        "capture_plan_verified": True,
        "client_report": capture_report,
    }


def ply_vertex_count(path: Path) -> int:
    try:
        with path.open("rb") as stream:
            for _ in range(200):
                line = stream.readline()
                if not line:
                    break
                decoded = line.decode("ascii", errors="strict").strip()
                match = re.fullmatch(r"element vertex (\d+)", decoded)
                if match:
                    return int(match.group(1))
                if decoded == "end_header":
                    break
    except (OSError, UnicodeDecodeError) as error:
        raise PipelineError(f"cannot read PLY header from {path}: {error}") from error
    raise PipelineError(f"PLY header has no vertex count: {path}")


def database_statistics(database: Path) -> dict[str, Any]:
    try:
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        cursor = connection.cursor()
        image_count = int(cursor.execute("SELECT COUNT(*) FROM images").fetchone()[0])
        keypoint = cursor.execute(
            "SELECT COALESCE(SUM(rows),0), COALESCE(MIN(rows),0), "
            "COALESCE(MAX(rows),0), COALESCE(AVG(rows),0) FROM keypoints"
        ).fetchone()
        raw_pairs, raw_matches = cursor.execute(
            "SELECT COUNT(*), COALESCE(SUM(rows),0) FROM matches WHERE rows > 0"
        ).fetchone()
        verified_pairs, verified_inliers = cursor.execute(
            "SELECT COUNT(*), COALESCE(SUM(rows),0) FROM two_view_geometries WHERE rows > 0"
        ).fetchone()
    except sqlite3.Error as error:
        raise PipelineError(f"cannot inspect COLMAP database {database}: {error}") from error
    finally:
        if "connection" in locals():
            connection.close()
    return {
        "database_images": image_count,
        "keypoints_total": int(keypoint[0]),
        "keypoints_min": int(keypoint[1]),
        "keypoints_max": int(keypoint[2]),
        "keypoints_mean": float(keypoint[3]),
        "raw_matched_pairs": int(raw_pairs),
        "raw_matches_total": int(raw_matches),
        "verified_pairs": int(verified_pairs),
        "verified_inliers_total": int(verified_inliers),
    }


def find_sparse_model(workspace: Path) -> Path:
    preferred = workspace / "sparse" / "triangulated"
    if preferred.is_dir():
        return preferred
    sparse = workspace / "sparse"
    for directory in sorted((path for path in sparse.iterdir() if path.is_dir()), key=lambda p: p.name):
        if (directory / "images.bin").is_file() or (directory / "images.txt").is_file():
            return directory
    raise PipelineError(f"cannot find a COLMAP sparse model below {sparse}")


def parse_model_analyzer(output: str) -> dict[str, Any]:
    fields: dict[str, tuple[str, type[int] | type[float]]] = {
        "Registered images": ("registered_images", int),
        "Points": ("sparse_points", int),
        "Observations": ("sparse_observations", int),
        "Mean track length": ("mean_track_length", float),
        "Mean observations per image": ("mean_observations_per_image", float),
        "Mean reprojection error": ("mean_reprojection_error_px", float),
    }
    result: dict[str, Any] = {}
    for label, (key, converter) in fields.items():
        suffix = r"px" if label == "Mean reprojection error" else ""
        match = re.search(rf"{re.escape(label)}:\s*([0-9.eE+-]+){suffix}", output)
        if match:
            result[key] = converter(match.group(1))
    if "registered_images" not in result:
        raise PipelineError("could not parse registered image count from COLMAP model_analyzer")
    return result


def collect_reconstruction_statistics(
    case: Path,
    colmap: Path,
    gpu_requested: bool,
    elapsed_seconds: float,
    pipeline_log: PipelineLog,
    progress_seconds: int,
) -> dict[str, Any]:
    workspace = case / "workspace"
    reconstruction = read_json(workspace / "reconstruction.json")
    database = database_statistics(workspace / "database.db")
    analyzer = run_command(
        "COLMAP model analysis",
        [colmap, "model_analyzer", "--path", find_sparse_model(workspace)],
        case / "model_analyzer.log",
        pipeline_log,
        progress_seconds,
    )
    model = parse_model_analyzer(analyzer.output)
    colmap_log_path = workspace / "colmap.log"
    colmap_log = colmap_log_path.read_text(encoding="utf-8", errors="replace")
    depth_directory = workspace / "dense" / "stereo" / "depth_maps"
    photometric = len(list(depth_directory.glob("*.photometric.bin"))) if depth_directory.is_dir() else 0
    geometric = len(list(depth_directory.glob("*.geometric.bin"))) if depth_directory.is_dir() else 0
    cuda_mentions = colmap_log.count("cudacc.cc")
    feature_gpu = "--FeatureExtraction.use_gpu 1" in colmap_log
    matching_gpu = "--FeatureMatching.use_gpu 1" in colmap_log
    gpu_verified = gpu_requested and feature_gpu and matching_gpu and cuda_mentions > 0
    if gpu_requested and not gpu_verified:
        raise PipelineError(
            "GPU mode was requested but CUDA execution could not be verified in workspace/colmap.log"
        )

    point_cloud = Path(reconstruction["point_cloud"])
    result = {
        "engine": reconstruction.get("engine"),
        "camera_mode": reconstruction.get("camera_mode"),
        "density": reconstruction.get("density"),
        "unit": reconstruction.get("unit"),
        "gpu_requested": gpu_requested,
        "gpu_verified": gpu_verified,
        "feature_extraction_gpu_verified": feature_gpu,
        "feature_matching_gpu_verified": matching_gpu,
        "cuda_sweep_log_mentions": cuda_mentions,
        "fatal_error_count": len(
            re.findall(r"CUDA.*error|Check failed|FATAL", colmap_log, flags=re.IGNORECASE)
        ),
        "photometric_depth_maps": photometric,
        "geometric_depth_maps": geometric,
        "point_count": ply_vertex_count(point_cloud),
        "point_cloud": point_cloud.name,
        "point_cloud_bytes": point_cloud.stat().st_size,
        "point_cloud_sha256": sha256(point_cloud),
        "elapsed_seconds": elapsed_seconds,
    }
    result.update(database)
    result.update(model)
    return result


def parse_fitting_output(output: str, elapsed_seconds: float, point_cloud: Path) -> dict[str, Any]:
    preprocessing = re.search(
        r"Preprocessed\s+(\d+)\s+->\s+(\d+)\s+points\s+"
        r"\(voxel removed\s+(\d+),\s+outliers removed\s+(\d+),\s+"
        r"normals estimated\s+(\d+)\)\.",
        output,
    )
    proposals = re.search(
        r"Primitive proposals:\s+(\d+) planes,\s+(\d+) cylinders,\s+"
        r"(\d+) spheres,\s+(\d+) cones,\s+(\d+) tori,\s+"
        r"(\d+) unassigned points\.",
        output,
    )
    no_proposals = re.search(r"No primitive proposals:\s+(.+)", output)
    if (
        not preprocessing
        or (not proposals and not no_proposals)
        or "Point-cloud I/O test passed." not in output
    ):
        raise PipelineError("could not parse the point-cloud fitting validation output")

    recognized = re.search(
        r"Recognized box\s+(.+?),\s+confidence\s+([0-9.eE+-]+)\.", output
    )
    not_recognized = re.search(r"No box recognized:\s+(.+)", output)
    proposal_report = {
        "planes": int(proposals.group(1)) if proposals else 0,
        "cylinders": int(proposals.group(2)) if proposals else 0,
        "spheres": int(proposals.group(3)) if proposals else 0,
        "cones": int(proposals.group(4)) if proposals else 0,
        "tori": int(proposals.group(5)) if proposals else 0,
        "unassigned_points": int(proposals.group(6)) if proposals else int(preprocessing.group(2)),
        "detection_error": no_proposals.group(1).strip() if no_proposals else None,
    }
    report: dict[str, Any] = {
        "schema": "paramcad.fitting-report",
        "version": 1,
        "source_point_cloud": point_cloud.name,
        "exit_code": 0,
        "elapsed_seconds": elapsed_seconds,
        "preprocessing": {
            "input_points": int(preprocessing.group(1)),
            "output_points": int(preprocessing.group(2)),
            "voxel_duplicates_removed": int(preprocessing.group(3)),
            "statistical_outliers_removed": int(preprocessing.group(4)),
            "normals_estimated": int(preprocessing.group(5)),
        },
        "box_recognition": {
            "recognized": recognized is not None,
            "description": recognized.group(1) if recognized else None,
            "confidence": float(recognized.group(2)) if recognized else None,
            "reason": not_recognized.group(1).strip() if not_recognized else None,
        },
        "primitive_proposals": proposal_report,
        "status": "io_and_detection_pipeline_passed",
        "log": "fitting.log",
    }
    report["complete_cad_fit"] = bool(recognized)
    return report


def relative_to_case(path: Path, case: Path) -> str:
    return path.resolve().relative_to(case.resolve()).as_posix()


def write_readme(
    case: Path,
    capture: dict[str, Any],
    reconstruction: dict[str, Any],
    fitting: dict[str, Any],
    model_snapshot: Path,
    texture_snapshot: Path,
) -> None:
    preprocessing = fitting["preprocessing"]
    proposals = fitting["primitive_proposals"]
    mode = "GPU 稠密" if reconstruction["gpu_requested"] else "CPU 稀疏"
    lines = [
        f"# {case.name}",
        "",
        "本目录由 `tools/run_performance_validation.py` 自动生成，所有输入快照、拍摄图片、",
        "CameraInfo、COLMAP 工作区、点云、拟合日志和报告均保存在本目录内。",
        "",
        "## 摘要",
        "",
        f"- 模式：{mode}",
        f"- 拍摄图片：{capture['images']} 张，全部相机对准原点",
        f"- COLMAP 注册图片：{reconstruction.get('registered_images', 0)} 张",
        f"- 输出点云：`{reconstruction['point_cloud']}`，{reconstruction['point_count']} 点",
        f"- 预处理：{preprocessing['input_points']} -> {preprocessing['output_points']} 点",
        f"- 去重：移除 {preprocessing['voxel_duplicates_removed']} 点",
        f"- 离群点过滤：移除 {preprocessing['statistical_outliers_removed']} 点",
        (
            "- 图元候选："
            f"{proposals['planes']} 平面、{proposals['cylinders']} 圆柱、"
            f"{proposals['spheres']} 球、{proposals['cones']} 圆锥、"
            f"{proposals['tori']} 圆环"
        ),
        f"- 完整箱体拟合：{'成功' if fitting['complete_cad_fit'] else '未识别'}",
        "",
        "## 输入快照",
        "",
        f"- `{relative_to_case(model_snapshot, case)}`",
        f"- `{relative_to_case(texture_snapshot, case)}`",
        "",
        "## 报告与日志",
        "",
        "- `validation_report.json`：完整机器可读摘要",
        "- `capture_report.json`：客户端拍摄报告",
        "- `camera_info.json`：传给 COLMAP 的相机内外参",
        "- `workspace/reconstruction.json`：COLMAP 重建清单",
        "- `workspace/colmap.log`：COLMAP 完整日志",
        "- `fitting_report.json` / `fitting.log`：点云预处理和拟合结果",
        "- `pipeline.log`：端到端阶段日志",
        "",
    ]
    (case / "README.md").write_text("\n".join(lines), encoding="utf-8")


def directory_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def run_pipeline(args: argparse.Namespace) -> Path:
    if args.progress_seconds < 0:
        raise PipelineError("--progress-seconds must be zero or positive")
    model = resolve_input(args.model, "model")
    ensure_step_model(model)
    texture = resolve_input(args.texture, "texture")
    client = find_client(args.client)
    colmap = find_colmap(args.colmap)
    fit_executable = find_fit_executable(args.fit_executable, client)
    if not RECONSTRUCTION_SCRIPT.is_file():
        raise PipelineError(f"reconstruction script is missing: {RECONSTRUCTION_SCRIPT}")

    case = resolve_case_directory(args.output, model)
    pipeline_log = PipelineLog(case / "pipeline.log")
    stage = "initialization"
    try:
        pipeline_log.write(f"case: {case}")
        pipeline_log.write(f"mode: {'GPU dense' if args.gpu else 'CPU sparse'}")

        input_directory = case / "input"
        input_directory.mkdir()
        model_snapshot = input_directory / f"model-{model.name}"
        texture_snapshot = input_directory / f"texture-{texture.name}"
        shutil.copy2(model, model_snapshot)
        shutil.copy2(texture, texture_snapshot)
        input_report = {
            "model": {
                "source": str(model),
                "snapshot": relative_to_case(model_snapshot, case),
                "bytes": model_snapshot.stat().st_size,
                "sha256": sha256(model_snapshot),
            },
            "texture": {
                "source": str(texture),
                "snapshot": relative_to_case(texture_snapshot, case),
                "bytes": texture_snapshot.stat().st_size,
                "sha256": sha256(texture_snapshot),
            },
        }

        stage = "capture"
        capture_result = run_command(
            "62-view capture",
            [
                client,
                "--performance-capture",
                model_snapshot,
                "--output",
                case,
                "--texture",
                texture_snapshot,
            ],
            case / "capture.log",
            pipeline_log,
            args.progress_seconds,
        )
        capture = validate_capture(case)
        capture["elapsed_seconds"] = capture_result.elapsed_seconds
        pipeline_log.write(
            f"capture validation: {capture['images']} images, target max abs "
            f"{capture['maximum_absolute_target_coordinate']:.3g}"
        )

        stage = "reconstruction"
        point_cloud = case / ("dense_cloud.ply" if args.gpu else "sparse_cloud.ply")
        reconstruction_command: list[str | Path] = [
            sys.executable,
            RECONSTRUCTION_SCRIPT,
            "--images",
            case / "images",
            "--camera-info",
            case / "camera_info.json",
            "--input-unit",
            "mm",
            "--matcher",
            args.matcher,
            "--output",
            point_cloud,
            "--workspace",
            case / "workspace",
            "--colmap",
            colmap,
        ]
        if args.gpu:
            reconstruction_command.extend(["--dense", "--gpu"])
        reconstruction_result = run_command(
            "COLMAP reconstruction",
            reconstruction_command,
            case / "reconstruction.log",
            pipeline_log,
            args.progress_seconds,
        )
        reconstruction = collect_reconstruction_statistics(
            case,
            colmap,
            args.gpu,
            reconstruction_result.elapsed_seconds,
            pipeline_log,
            args.progress_seconds,
        )
        pipeline_log.write(
            f"reconstruction validation: {reconstruction['point_count']} points, "
            f"{reconstruction['registered_images']} registered images"
        )

        stage = "fitting"
        fitting_result = run_command(
            "point-cloud preprocessing and fitting",
            [fit_executable, point_cloud],
            case / "fitting.log",
            pipeline_log,
            args.progress_seconds,
        )
        fitting = parse_fitting_output(
            fitting_result.output,
            fitting_result.elapsed_seconds,
            point_cloud,
        )
        write_json(case / "fitting_report.json", fitting)

        stage = "reporting"
        report = {
            "schema": "paramcad.performance-validation-report",
            "version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "status": "completed",
            "mode": "gpu_dense" if args.gpu else "cpu_sparse",
            "inputs": input_report,
            "executables": {
                "client": str(client),
                "colmap": str(colmap),
                "fit": str(fit_executable),
                "python": sys.executable,
            },
            "capture": capture,
            "colmap": reconstruction,
            "fitting": fitting,
            "artifacts": {
                "images": "images/",
                "camera_info": "camera_info.json",
                "capture_report": "capture_report.json",
                "point_cloud": point_cloud.name,
                "workspace": "workspace/",
                "colmap_log": "workspace/colmap.log",
                "fitting_log": "fitting.log",
                "fitting_report": "fitting_report.json",
            },
        }
        write_json(case / "validation_report.json", report)
        write_readme(case, capture, reconstruction, fitting, model_snapshot, texture_snapshot)
        report["case_bytes"] = directory_size(case)
        write_json(case / "validation_report.json", report)
        pipeline_log.write(
            f"pipeline completed: {point_cloud.name}, {reconstruction['point_count']} points"
        )
        return case
    except Exception as error:
        failure = {
            "schema": "paramcad.performance-validation-failure",
            "version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "status": "failed",
            "stage": stage,
            "error": str(error),
        }
        write_json(case / "pipeline_failure.json", failure)
        pipeline_log.write(f"pipeline failed during {stage}: {error}")
        if isinstance(error, PipelineError):
            raise
        raise PipelineError(f"unexpected failure during {stage}: {error}") from error


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_arguments(argv)
    try:
        case = run_pipeline(args)
    except PipelineError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(f"validation case: {case}")
    print(f"report: {case / 'validation_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
