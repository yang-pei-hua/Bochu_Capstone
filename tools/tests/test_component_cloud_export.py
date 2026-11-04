"""Original-observation and numbering contracts for component review exports."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from export_component_clouds import (camera, partition, principal_frame, read_ply,
                                    render_contact_sheet, spread_labels, write_part)


class ComponentCloudExportTests(unittest.TestCase):
    def test_double_coordinates_normals_and_original_ids_round_trip(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"source.ply"
            path.write_text("ply\nformat ascii 1.0\nelement vertex 3\n"+
                            "".join("property double "+n+"\n" for n in ("x", "y", "z", "nx", "ny", "nz"))+
                            "property uchar red\nend_header\n"+
                            "1234.12345678901 2 3 0 0 1 73\n4 5 6 0 1 0 91\n7 8 9 1 0 0 127\n")
            source = read_ply(path)
            exported = Path(temp)/"cloud.ply"
            write_part(exported, source, np.array([0, 2]), 17)
            restored = read_ply(exported)
            for name in source.dtype.names:
                np.testing.assert_array_equal(restored[name], source[name][[0, 2]])
            np.testing.assert_array_equal(restored["source_point_id"], [1, 3])
            np.testing.assert_array_equal(restored["part_id"], [17, 17])
            self.assertEqual(restored.dtype["x"].itemsize, 8)

    def test_partition_includes_open_candidates_and_all_residual_points(self):
        report = {"parts": [{"id": 9, "point_ids": [4, 1], "closed": False},
                            {"id": 2, "point_ids": [3], "closed": True}],
                  "summary": {"unassigned_points": 2}}
        labels, groups, residual = partition(5, report)
        np.testing.assert_array_equal(labels, [9, 0, 2, 9, 0])
        self.assertEqual([p["id"] for p, _ in groups], [2, 9])
        np.testing.assert_array_equal(residual, [1, 4])
        report["parts"][1]["point_ids"] = [1]
        with self.assertRaisesRegex(ValueError, "more than one"):
            partition(5, report)
        report["parts"][1]["point_ids"] = [7]
        with self.assertRaisesRegex(ValueError, "Out-of-range"):
            partition(5, report)

    def test_preview_does_not_modify_exported_geometry(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source = np.zeros(4, dtype=[(n, "<f8") for n in ("x", "y", "z")])
            source["x"] = [100, 102, 102, 100]
            source["y"] = [-10, -10, -9, -9]
            source["z"] = [23, 23, 23, 23]
            write_part(directory/"cloud.ply", source, np.arange(4), 8)
            original = (directory/"cloud.ply").read_bytes()
            (directory/"cloud.json").write_text('{"unit":"mm"}')
            result = render_contact_sheet({"directory": temp, "name": "fixture_008", "part_id": 8,
                                           "closed": False, "unit": "mm", "preview_points": 100})
            self.assertEqual((directory/"cloud.ply").read_bytes(), original)
            self.assertEqual(result["points"], 4)
            metadata = json.loads((directory/"cloud.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["preview"]["view_count"], 12)
            with Image.open(directory/"views.png") as image:
                self.assertEqual(image.size, (1760, 1262))
            center, basis = principal_frame(np.column_stack([source[n] for n in ("x", "y", "z")]))
            np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-12)
            self.assertGreater(np.linalg.det(basis), .99)
            np.testing.assert_allclose(camera((1, 1, 1)).T @ camera((1, 1, 1)), np.eye(3), atol=1e-12)

    def test_locator_lanes_keep_colliding_numbers_separate(self):
        placed = spread_labels([(i, 100) for i in range(16)], 80, 850)
        heights = [y for _, y in placed]
        self.assertGreaterEqual(min(heights), 80)
        self.assertLessEqual(max(heights), 850)
        self.assertTrue(all(b-a >= 38.99 for a, b in zip(heights, heights[1:])))
        self.assertEqual({i for (i, _), _ in placed}, set(range(16)))


if __name__ == "__main__":
    unittest.main()
