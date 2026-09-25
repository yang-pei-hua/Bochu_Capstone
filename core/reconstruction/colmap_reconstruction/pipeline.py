"""Headless COLMAP orchestration for Layer 2 reconstruction."""

from __future__ import annotations

import json
import os
import shutil
import sqlite3
import struct
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from .camera_info import CameraInfo, load_camera_info


class ReconstructionError(RuntimeError):
    """Raised for a failed or invalid reconstruction."""


@dataclass(frozen=True)
class ReconstructionResult:
    point_cloud: Path
    workspace: Path
    manifest: Path
    point_count: int
    camera_mode: str
    dense: bool


class _Runner:
    def __init__(self, executable: Path, log_path: Path) -> None:
        self.executable = executable
        self.log_path = log_path

    def run(self, command: str, *args: str | Path) -> None:
        invocation = [str(self.executable), command, *(str(arg) for arg in args)]
        with self.log_path.open("a", encoding="utf-8") as log:
            log.write("\n$ " + subprocess.list2cmdline(invocation) + "\n")
            log.flush()
            try:
                subprocess.run(invocation, check=True, stdout=log, stderr=subprocess.STDOUT)
            except FileNotFoundError as exc:
                raise ReconstructionError(f"COLMAP executable not found: {self.executable}") from exc
            except subprocess.CalledProcessError as exc:
                raise ReconstructionError(
                    f"COLMAP command {command!r} failed with exit code {exc.returncode}; see {self.log_path}"
                ) from exc


def find_colmap_executable(explicit: str | Path | None = None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("COLMAP_EXECUTABLE"):
        candidates.append(Path(os.environ["COLMAP_EXECUTABLE"]))
    on_path = shutil.which("colmap")
    if on_path:
        candidates.append(Path(on_path))
    root = Path(__file__).resolve().parents[1]
    project_root = Path(__file__).resolve().parents[3]
    candidates.extend(
        [
            project_root / "deps" / "colmap" / "bin" / "colmap.exe",
            project_root / "deps" / "colmap" / "bin" / "colmap",
            root / "build" / "colmap" / "src" / "colmap" / "exe" / "colmap.exe",
            root / "build" / "colmap" / "src" / "colmap" / "exe" / "colmap",
        ]
    )
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.is_file():
            return resolved
    raise ReconstructionError(
        "COLMAP executable was not found. Run build_colmap.ps1 or pass --colmap /path/to/colmap."
    )


def _prepare_workspace(workspace: Path) -> None:
    if workspace.exists() and not workspace.is_dir():
        raise ReconstructionError(f"workspace path is not a directory: {workspace}")
    if workspace.exists() and any(workspace.iterdir()):
        raise ReconstructionError(
            f"workspace is not empty: {workspace}. Choose a new --workspace path to avoid overwriting data."
        )
    workspace.mkdir(parents=True, exist_ok=True)


def _database_images(database: Path) -> dict[str, tuple[int, int, int, int]]:
    """Return name -> (image_id, camera_id, detected width, detected height)."""

    connection: sqlite3.Connection | None = None
    try:
        connection = sqlite3.connect(database)
        rows = connection.execute(
            "SELECT images.name, images.image_id, images.camera_id, cameras.width, cameras.height "
            "FROM images JOIN cameras USING(camera_id)"
        ).fetchall()
    except sqlite3.Error as exc:
        raise ReconstructionError(f"cannot read COLMAP database {database}: {exc}") from exc
    finally:
        if connection is not None:
            connection.close()
    return {str(name).replace("\\", "/"): (image_id, camera_id, width, height) for name, image_id, camera_id, width, height in rows}


def _apply_camera_info(database: Path, camera_info: CameraInfo) -> dict[str, tuple[int, int, int, int]]:
    database_images = _database_images(database)
    expected = set(camera_info.images)
    actual = set(database_images)
    if expected != actual:
        missing = sorted(actual - expected)
        extra = sorted(expected - actual)
        raise ReconstructionError(
            "CameraInfo image names do not match the extracted image set; "
            f"missing entries={missing[:10]}, unknown entries={extra[:10]}"
        )

    connection: sqlite3.Connection | None = None
    try:
        connection = sqlite3.connect(database)
        for name, (_, camera_id, detected_width, detected_height) in database_images.items():
            camera = camera_info.cameras[camera_info.images[name].camera_key]
            if (camera.width, camera.height) != (detected_width, detected_height):
                raise ReconstructionError(
                    f"CameraInfo size for {name} is {camera.width}x{camera.height}, "
                    f"but the image is {detected_width}x{detected_height}"
                )
            params_blob = sqlite3.Binary(struct.pack(f"<{len(camera.params)}d", *camera.params))
            connection.execute(
                "UPDATE cameras SET model=?, width=?, height=?, params=?, prior_focal_length=1 WHERE camera_id=?",
                (camera.model_id, camera.width, camera.height, params_blob, camera_id),
            )
        connection.commit()
    except sqlite3.Error as exc:
        raise ReconstructionError(f"cannot update COLMAP camera intrinsics: {exc}") from exc
    finally:
        if connection is not None:
            connection.close()
    return database_images


def _write_seed_model(
    output: Path,
    camera_info: CameraInfo,
    database_images: dict[str, tuple[int, int, int, int]],
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    camera_lines = [
        "# Camera list with one line of data per camera:",
        "# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]",
    ]
    written_camera_ids: set[int] = set()
    image_lines = [
        "# Image list with two lines of data per image:",
        "# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME",
        "# POINTS2D[] as (X Y POINT3D_ID)",
    ]
    for name in sorted(camera_info.images):
        image = camera_info.images[name]
        if image.pose is None:
            continue
        image_id, camera_id, _, _ = database_images[name]
        camera = camera_info.cameras[image.camera_key]
        if camera_id not in written_camera_ids:
            params = " ".join(format(value, ".17g") for value in camera.params)
            camera_lines.append(f"{camera_id} {camera.model} {camera.width} {camera.height} {params}")
            written_camera_ids.add(camera_id)
        pose = (*image.pose.qvec, *image.pose.tvec)
        pose_text = " ".join(format(value, ".17g") for value in pose)
        image_lines.extend([f"{image_id} {pose_text} {camera_id} {name}", ""])

    (output / "cameras.txt").write_text("\n".join(camera_lines) + "\n", encoding="utf-8")
    (output / "images.txt").write_text("\n".join(image_lines) + "\n", encoding="utf-8")
    (output / "points3D.txt").write_text("", encoding="utf-8")


def _select_largest_model(sparse_root: Path) -> Path:
    candidates: list[tuple[int, Path]] = []
    for directory in sparse_root.iterdir():
        if not directory.is_dir():
            continue
        point_file = next((directory / name for name in ("points3D.bin", "points3D.txt") if (directory / name).is_file()), None)
        camera_file_exists = any((directory / name).is_file() for name in ("cameras.bin", "cameras.txt"))
        if point_file and camera_file_exists:
            candidates.append((point_file.stat().st_size, directory))
    if not candidates:
        raise ReconstructionError(f"COLMAP did not produce a sparse model under {sparse_root}")
    return max(candidates, key=lambda item: item[0])[1]


def _ply_vertex_count(path: Path) -> int:
    if not path.is_file():
        raise ReconstructionError(f"COLMAP did not produce the requested point cloud: {path}")
    with path.open("rb") as stream:
        for _ in range(200):
            raw_line = stream.readline()
            if not raw_line:
                break
            line = raw_line.decode("ascii", errors="replace").strip()
            if line.startswith("element vertex "):
                try:
                    return int(line.split()[-1])
                except ValueError as exc:
                    raise ReconstructionError(f"invalid PLY vertex count in {path}") from exc
            if line == "end_header":
                break
    raise ReconstructionError(f"no PLY vertex count found in {path}")


def reconstruct(
    images: str | Path,
    output: str | Path,
    *,
    camera_info_path: str | Path | None = None,
    workspace: str | Path | None = None,
    colmap_executable: str | Path | None = None,
    dense: bool = False,
    matcher: str = "exhaustive",
    use_gpu: bool = False,
    input_unit: str = "unknown",
) -> ReconstructionResult:
    """Reconstruct a colored PLY point cloud from a directory of images."""

    image_path = Path(images).expanduser().resolve()
    output_path = Path(output).expanduser().resolve()
    if not image_path.is_dir():
        raise ReconstructionError(f"image directory does not exist: {image_path}")
    if output_path.suffix.lower() != ".ply":
        raise ReconstructionError("output path must have a .ply extension")
    if output_path.exists():
        raise ReconstructionError(f"output already exists; refusing to overwrite it: {output_path}")
    if matcher not in {"exhaustive", "sequential"}:
        raise ReconstructionError("matcher must be 'exhaustive' or 'sequential'")
    if input_unit not in {"unknown", "mm", "m"}:
        raise ReconstructionError("input_unit must be one of: unknown, mm, m")

    camera_info = load_camera_info(camera_info_path) if camera_info_path else None
    workspace_path = (
        Path(workspace).expanduser().resolve()
        if workspace
        else output_path.parent / f"{output_path.stem}_workspace"
    )
    _prepare_workspace(workspace_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    executable = find_colmap_executable(colmap_executable)
    log_path = workspace_path / "colmap.log"
    runner = _Runner(executable, log_path)
    database = workspace_path / "database.db"

    feature_args: list[str | Path] = [
        "--database_path", database,
        "--image_path", image_path,
        "--FeatureExtraction.use_gpu", "1" if use_gpu else "0",
    ]
    if camera_info:
        # A camera per image lets us safely apply arbitrary supplied intrinsics.
        feature_args.extend(["--camera_mode", "3"])
    runner.run("feature_extractor", *feature_args)

    database_images = _database_images(database)
    if len(database_images) < 2:
        raise ReconstructionError("at least two readable images are required")
    if camera_info:
        database_images = _apply_camera_info(database, camera_info)

    matcher_command = f"{matcher}_matcher"
    matching_option = "--FeatureMatching.use_gpu"
    runner.run(matcher_command, "--database_path", database, matching_option, "1" if use_gpu else "0")

    sparse_root = workspace_path / "sparse"
    sparse_root.mkdir()
    num_poses = camera_info.num_poses if camera_info else 0
    if num_poses == 0:
        runner.run(
            "mapper",
            "--database_path", database,
            "--image_path", image_path,
            "--output_path", sparse_root,
        )
        final_model = _select_largest_model(sparse_root)
        camera_mode = "estimated"
    else:
        assert camera_info is not None
        seed_model = workspace_path / "seed_model"
        _write_seed_model(seed_model, camera_info, database_images)
        triangulated = sparse_root / "triangulated"
        triangulated.mkdir()
        runner.run(
            "point_triangulator",
            "--database_path", database,
            "--image_path", image_path,
            "--input_path", seed_model,
            "--output_path", triangulated,
        )
        if num_poses == len(database_images):
            final_model = triangulated
            camera_mode = "provided"
        else:
            final_model = sparse_root / "final"
            final_model.mkdir()
            runner.run(
                "mapper",
                "--database_path", database,
                "--image_path", image_path,
                "--input_path", triangulated,
                "--output_path", final_model,
                "--Mapper.fix_existing_frames", "1",
            )
            camera_mode = "partial"

    if dense:
        dense_workspace = workspace_path / "dense"
        runner.run(
            "image_undistorter",
            "--image_path", image_path,
            "--input_path", final_model,
            "--output_path", dense_workspace,
            "--output_type", "COLMAP",
        )
        runner.run(
            "patch_match_stereo",
            "--workspace_path", dense_workspace,
            "--workspace_format", "COLMAP",
            "--PatchMatchStereo.geom_consistency", "1",
        )
        runner.run(
            "stereo_fusion",
            "--workspace_path", dense_workspace,
            "--workspace_format", "COLMAP",
            "--input_type", "geometric",
            "--output_path", output_path,
        )
    else:
        runner.run(
            "model_converter",
            "--input_path", final_model,
            "--output_path", output_path,
            "--output_type", "PLY",
        )

    point_count = _ply_vertex_count(output_path)
    if point_count <= 0:
        raise ReconstructionError(f"reconstruction produced an empty point cloud: {output_path}")
    if camera_mode == "estimated":
        output_unit = "arbitrary"
    else:
        output_unit = input_unit
    manifest_path = workspace_path / "reconstruction.json"
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "engine": "COLMAP 4.2.0",
        "images": str(image_path),
        "camera_info": str(Path(camera_info_path).resolve()) if camera_info_path else None,
        "camera_mode": camera_mode,
        "point_cloud": str(output_path),
        "point_count": point_count,
        "density": "dense" if dense else "sparse",
        "coordinate_system": "right-handed; COLMAP reconstruction world frame",
        "unit": output_unit,
        "note": (
            "Without supplied poses the reconstruction has arbitrary scale. "
            "With supplied poses it inherits the translation unit from CameraInfo."
        ),
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return ReconstructionResult(
        point_cloud=output_path,
        workspace=workspace_path,
        manifest=manifest_path,
        point_count=point_count,
        camera_mode=camera_mode,
        dense=dense,
    )
