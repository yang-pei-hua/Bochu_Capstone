from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from colmap_reconstruction.camera_info import CameraInfoError, load_camera_info


class CameraInfoTests(unittest.TestCase):
    def _load(self, value: dict):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            return load_camera_info(path)

    def test_camera_to_world_is_converted_to_colmap_pose(self) -> None:
        info = self._load(
            {
                "cameras": {
                    "c0": {"model": "PINHOLE", "width": 10, "height": 8, "params": [5, 5, 5, 4]}
                },
                "images": [
                    {
                        "name": "a.jpg",
                        "camera": "c0",
                        "camera_to_world": {
                            "qvec": [math.sqrt(0.5), 0, 0, math.sqrt(0.5)],
                            "position": [1, 0, 0],
                        },
                    },
                    {
                        "name": "b.jpg",
                        "camera": "c0",
                        "world_to_camera": {"qvec": [1, 0, 0, 0], "tvec": [0, 0, 0]},
                    },
                ],
            }
        )
        pose = info.images["a.jpg"].pose
        assert pose is not None
        self.assertAlmostEqual(pose.qvec[3], -math.sqrt(0.5))
        self.assertAlmostEqual(pose.tvec[0], 0.0, places=7)
        self.assertAlmostEqual(pose.tvec[1], 1.0, places=7)

    def test_rejects_a_single_pose(self) -> None:
        with self.assertRaisesRegex(CameraInfoError, "at least two"):
            self._load(
                {
                    "cameras": {
                        "c0": {"model": "SIMPLE_PINHOLE", "width": 10, "height": 8, "params": [5, 5, 4]}
                    },
                    "images": [
                        {"name": "a.jpg", "camera": "c0", "qvec": [1, 0, 0, 0], "tvec": [0, 0, 0]},
                        {"name": "b.jpg", "camera": "c0"},
                    ],
                }
            )

    def test_allows_intrinsics_without_poses(self) -> None:
        info = self._load(
            {
                "cameras": {
                    "c0": {"model": "SIMPLE_PINHOLE", "width": 10, "height": 8, "params": [5, 5, 4]}
                },
                "images": [
                    {"name": "a.jpg", "camera": "c0"},
                    {"name": "b.jpg", "camera": "c0"},
                ],
            }
        )
        self.assertEqual(info.num_poses, 0)


if __name__ == "__main__":
    unittest.main()

