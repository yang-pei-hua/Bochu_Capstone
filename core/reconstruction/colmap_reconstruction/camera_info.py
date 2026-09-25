"""Validation and coordinate conversion for the Layer 2 CameraInfo JSON."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class CameraInfoError(ValueError):
    """Raised when CameraInfo is incomplete or internally inconsistent."""


# COLMAP 4.2 camera model identifiers and parameter counts.
CAMERA_MODELS: dict[str, tuple[int, int]] = {
    "SIMPLE_PINHOLE": (0, 3),
    "PINHOLE": (1, 4),
    "SIMPLE_RADIAL": (2, 4),
    "RADIAL": (3, 5),
    "OPENCV": (4, 8),
    "OPENCV_FISHEYE": (5, 8),
    "FULL_OPENCV": (6, 12),
    "FOV": (7, 5),
    "SIMPLE_RADIAL_FISHEYE": (8, 4),
    "RADIAL_FISHEYE": (9, 5),
    "THIN_PRISM_FISHEYE": (10, 12),
    "RAD_TAN_THIN_PRISM_FISHEYE": (11, 16),
    "SIMPLE_DIVISION": (12, 4),
    "DIVISION": (13, 5),
    "SIMPLE_FISHEYE": (14, 3),
    "FISHEYE": (15, 4),
    "EUCM": (16, 6),
    "EQUIRECTANGULAR": (17, 2),
}


@dataclass(frozen=True)
class Camera:
    key: str
    model: str
    width: int
    height: int
    params: tuple[float, ...]

    @property
    def model_id(self) -> int:
        return CAMERA_MODELS[self.model][0]


@dataclass(frozen=True)
class Pose:
    """COLMAP world-to-camera pose using a Hamilton quaternion."""

    qvec: tuple[float, float, float, float]
    tvec: tuple[float, float, float]


@dataclass(frozen=True)
class CameraImage:
    name: str
    camera_key: str
    pose: Pose | None


@dataclass(frozen=True)
class CameraInfo:
    cameras: dict[str, Camera]
    images: dict[str, CameraImage]

    @property
    def num_poses(self) -> int:
        return sum(image.pose is not None for image in self.images.values())


def _number_tuple(value: Any, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise CameraInfoError(f"{field} must be an array of {length} numbers")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise CameraInfoError(f"{field} must contain only numbers") from exc
    if not all(math.isfinite(item) for item in result):
        raise CameraInfoError(f"{field} contains a non-finite number")
    return result


def _normalize_qvec(qvec: tuple[float, ...], field: str) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(item * item for item in qvec))
    if norm < 1e-12:
        raise CameraInfoError(f"{field} must not be the zero quaternion")
    return tuple(item / norm for item in qvec)  # type: ignore[return-value]


def _rotate(qvec: tuple[float, float, float, float], vector: tuple[float, ...]) -> tuple[float, float, float]:
    """Rotate a vector with a normalized Hamilton quaternion."""

    w, x, y, z = qvec
    vx, vy, vz = vector
    # Expanded q * (0, v) * conjugate(q).
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def _parse_pose(raw: dict[str, Any], field: str) -> Pose | None:
    world_to_camera = raw.get("world_to_camera")
    camera_to_world = raw.get("camera_to_world")
    bare_pose = "qvec" in raw or "tvec" in raw
    count = sum(item is not None for item in (world_to_camera, camera_to_world)) + int(bare_pose)
    if count == 0:
        return None
    if count != 1:
        raise CameraInfoError(
            f"{field} must specify exactly one of world_to_camera, camera_to_world, or bare qvec/tvec"
        )

    if bare_pose:
        qvec = _normalize_qvec(_number_tuple(raw.get("qvec"), 4, f"{field}.qvec"), f"{field}.qvec")
        tvec = _number_tuple(raw.get("tvec"), 3, f"{field}.tvec")
        return Pose(qvec=qvec, tvec=tvec)  # type: ignore[arg-type]

    if world_to_camera is not None:
        if not isinstance(world_to_camera, dict):
            raise CameraInfoError(f"{field}.world_to_camera must be an object")
        qvec = _normalize_qvec(
            _number_tuple(world_to_camera.get("qvec"), 4, f"{field}.world_to_camera.qvec"),
            f"{field}.world_to_camera.qvec",
        )
        tvec = _number_tuple(world_to_camera.get("tvec"), 3, f"{field}.world_to_camera.tvec")
        return Pose(qvec=qvec, tvec=tvec)  # type: ignore[arg-type]

    if not isinstance(camera_to_world, dict):
        raise CameraInfoError(f"{field}.camera_to_world must be an object")
    q_cw = _normalize_qvec(
        _number_tuple(camera_to_world.get("qvec"), 4, f"{field}.camera_to_world.qvec"),
        f"{field}.camera_to_world.qvec",
    )
    center = _number_tuple(camera_to_world.get("position"), 3, f"{field}.camera_to_world.position")
    q_wc = (q_cw[0], -q_cw[1], -q_cw[2], -q_cw[3])
    rotated_center = _rotate(q_wc, center)
    return Pose(qvec=q_wc, tvec=tuple(-item for item in rotated_center))  # type: ignore[arg-type]


def load_camera_info(path: str | Path) -> CameraInfo:
    """Load and validate CameraInfo from JSON.

    Image names use forward slashes and are relative to the input image folder.
    Poses may be omitted per image. At least two supplied poses are needed for
    known/partial-pose reconstruction.
    """

    source = Path(path)
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CameraInfoError(f"cannot read CameraInfo {source}: {exc}") from exc
    if not isinstance(raw, dict):
        raise CameraInfoError("CameraInfo root must be an object")

    raw_cameras = raw.get("cameras")
    if not isinstance(raw_cameras, dict) or not raw_cameras:
        raise CameraInfoError("cameras must be a non-empty object")
    cameras: dict[str, Camera] = {}
    for key, value in raw_cameras.items():
        field = f"cameras.{key}"
        if not isinstance(key, str) or not key or not isinstance(value, dict):
            raise CameraInfoError(f"{field} must be an object")
        model = str(value.get("model", "")).upper()
        if model not in CAMERA_MODELS:
            raise CameraInfoError(f"{field}.model is not supported by COLMAP 4.2: {model!r}")
        try:
            width = int(value["width"])
            height = int(value["height"])
        except (KeyError, TypeError, ValueError) as exc:
            raise CameraInfoError(f"{field}.width and height must be positive integers") from exc
        if width <= 0 or height <= 0:
            raise CameraInfoError(f"{field}.width and height must be positive integers")
        param_count = CAMERA_MODELS[model][1]
        params = _number_tuple(value.get("params"), param_count, f"{field}.params")
        cameras[key] = Camera(key=key, model=model, width=width, height=height, params=params)

    raw_images = raw.get("images")
    if not isinstance(raw_images, list) or not raw_images:
        raise CameraInfoError("images must be a non-empty array")
    images: dict[str, CameraImage] = {}
    for index, value in enumerate(raw_images):
        field = f"images[{index}]"
        if not isinstance(value, dict):
            raise CameraInfoError(f"{field} must be an object")
        name = str(value.get("name", "")).replace("\\", "/")
        camera_key = str(value.get("camera", ""))
        name_path = Path(name)
        if not name or name == "." or name_path.is_absolute() or ".." in name_path.parts:
            raise CameraInfoError(f"{field}.name must be a safe relative path")
        if name in images:
            raise CameraInfoError(f"duplicate image name: {name}")
        if camera_key not in cameras:
            raise CameraInfoError(f"{field}.camera refers to unknown camera {camera_key!r}")
        images[name] = CameraImage(name=name, camera_key=camera_key, pose=_parse_pose(value, field))

    if 0 < sum(image.pose is not None for image in images.values()) < 2:
        raise CameraInfoError("provide poses for at least two images, or omit all poses")
    return CameraInfo(cameras=cameras, images=images)
