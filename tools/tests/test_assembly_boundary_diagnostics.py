"""Geometric interpretation contracts for the offline boundary report."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.diagnostics.diagnose_assembly_boundaries import crosses_itself, metrics


class AssemblyBoundaryDiagnosticsTests(unittest.TestCase):
    def test_crossing_is_distinguished_from_concavity_and_holes(self):
        self.assertTrue(crosses_itself(np.array([[0, 0], [2, 2], [0, 2], [2, 0]], dtype=float)))
        self.assertFalse(crosses_itself(np.array([[0, 0], [4, 0], [4, 1], [1, 1], [1, 4], [0, 4]], dtype=float)))
        self.assertFalse(crosses_itself(np.array([[0, 0], [0, 4], [4, 4], [4, 0]], dtype=float)))

    def test_closed_contours_do_not_imply_a_closed_shell(self):
        outer = [[0, 0, 0], [4, 0, 0], [4, 4, 0], [0, 4, 0]]
        hole = [[1, 1, 0], [1, 2, 0], [2, 2, 0], [2, 1, 0]]
        patch = {"id": 1, "origin_mm": [0, 0, 0], "normal": [0, 0, 1], "x_direction": [1, 0, 0],
                 "planar": True, "spacing_mm": .5, "boundary_loops_mm": [outer, hole]}
        part = {"id": 1, "patch_ids": [1], "closed": False, "point_ids": [1, 2, 3],
                "reason": "Boundary graph has 8 unmatched and 0 inconsistent edges", "nonmanifold_edges": 0}
        data = {"patches": [patch], "parts": [part],
                "summary": {"working_points": 3, "open_edges": 8, "closed_parts": 0}}
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "parts.json").write_text(json.dumps(data))
            value = metrics(directory, 3)
            self.assertEqual(value["boundary_loops"], 2)
            self.assertEqual(value["open_candidate_segments"], 8)
            self.assertEqual(value["closed_candidate_segments"], 0)
            self.assertEqual(value["closed_observation_fraction"], 0)
            self.assertEqual(value["patches_with_disconnected_outer_loops"], 0)
            self.assertEqual(value["failure_reasons"], {"boundary_edge_mismatch": 1})
            patch["boundary_loops_mm"][1] = hole[::-1]
            (directory / "parts.json").write_text(json.dumps(data))
            self.assertEqual(metrics(directory, 3)["patches_with_disconnected_outer_loops"], 1)
            patch["boundary_before_closure"] = {"normal": [0, 0, 1],
                "boundary_loops_mm": [outer, hole], "revision_source": "convex_support_planes"}
            patch["boundary_loops_mm"] = [outer]
            data["summary"]["unassigned_working_points"] = 0
            (directory / "parts.json").write_text(json.dumps(data))
            value = metrics(directory, 3)
            self.assertEqual(value["boundary_basis"], "observed")
            self.assertEqual(value["boundary_loops"], 2)
            self.assertEqual(value["raw_boundary_segments"], 8)
            self.assertEqual(value["closure_boundary_segments"], 4)



if __name__ == "__main__":
    unittest.main()
