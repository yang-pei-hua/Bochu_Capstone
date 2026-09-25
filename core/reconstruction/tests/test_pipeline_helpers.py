from __future__ import annotations

import json
import sqlite3
import struct
import tempfile
import unittest
from pathlib import Path

from colmap_reconstruction.camera_info import load_camera_info
from colmap_reconstruction.pipeline import _apply_camera_info, _ply_vertex_count, _write_seed_model


class PipelineHelperTests(unittest.TestCase):
    def test_intrinsics_are_written_to_database_and_seed_model(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "camera.json"
            config_path.write_text(
                json.dumps(
                    {
                        "cameras": {
                            "c0": {
                                "model": "PINHOLE",
                                "width": 640,
                                "height": 480,
                                "params": [500, 501, 320, 240],
                            }
                        },
                        "images": [
                            {"name": "a.jpg", "camera": "c0", "qvec": [1, 0, 0, 0], "tvec": [0, 0, 0]},
                            {"name": "b.jpg", "camera": "c0", "qvec": [1, 0, 0, 0], "tvec": [-1, 0, 0]},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            info = load_camera_info(config_path)
            database = root / "database.db"
            connection = sqlite3.connect(database)
            try:
                connection.execute(
                    "CREATE TABLE cameras(camera_id INTEGER PRIMARY KEY, model INTEGER, width INTEGER, "
                    "height INTEGER, params BLOB, prior_focal_length INTEGER)"
                )
                connection.execute(
                    "CREATE TABLE images(image_id INTEGER PRIMARY KEY, name TEXT, camera_id INTEGER)"
                )
                for image_id, name in enumerate(("a.jpg", "b.jpg"), start=1):
                    connection.execute(
                        "INSERT INTO cameras VALUES (?, 2, 640, 480, ?, 0)",
                        (image_id, sqlite3.Binary(struct.pack("<4d", 1, 2, 3, 4))),
                    )
                    connection.execute("INSERT INTO images VALUES (?, ?, ?)", (image_id, name, image_id))
                connection.commit()
            finally:
                connection.close()

            database_images = _apply_camera_info(database, info)
            connection = sqlite3.connect(database)
            try:
                model, params = connection.execute(
                    "SELECT model, params FROM cameras WHERE camera_id=1"
                ).fetchone()
            finally:
                connection.close()
            self.assertEqual(model, 1)
            self.assertEqual(struct.unpack("<4d", params), (500.0, 501.0, 320.0, 240.0))

            seed = root / "seed"
            _write_seed_model(seed, info, database_images)
            images_text = (seed / "images.txt").read_text(encoding="utf-8")
            self.assertIn("1 1 0 0 0 0 0 0 1 a.jpg", images_text)
            self.assertIn("2 1 0 0 0 -1 0 0 2 b.jpg", images_text)

    def test_reads_ascii_ply_vertex_count(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            ply = Path(directory) / "cloud.ply"
            ply.write_text("ply\nformat ascii 1.0\nelement vertex 42\nend_header\n", encoding="ascii")
            self.assertEqual(_ply_vertex_count(ply), 42)


if __name__ == "__main__":
    unittest.main()
