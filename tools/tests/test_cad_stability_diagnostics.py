"""Check that repeatability diagnostics detect real geometry changes."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_cad_validation_stability import geometry_parameters, parameter_spread


class StabilityDiagnosticTests(unittest.TestCase):
    def test_geometry_ignores_process_bookkeeping(self):
        report = {"cad": {"volume_mm3": 10, "step": "first.step"},
                  "elapsed_seconds": 2, "ransac_seed": 1}
        before = geometry_parameters(report)
        report.update(elapsed_seconds=3, ransac_seed=2)
        report["cad"]["step"] = "second.step"
        self.assertEqual(before, geometry_parameters(report))

    def test_spread_measures_hole_diameter_and_center(self):
        first = {"holes": [{"diameter_mm": 12, "axis_point_mm": [1, 2, 3]}]}
        second = {"holes": [{"diameter_mm": 12.1, "axis_point_mm": [1.2, 2, 3]}]}
        spread = parameter_spread([first, second])
        self.assertAlmostEqual(spread["holes[0].diameter_mm"]["spread"], 0.1)
        self.assertAlmostEqual(spread["holes[0].axis_point_mm[0]"]["spread"], 0.2)
        self.assertEqual(spread["holes[0].axis_point_mm[1]"]["spread"], 0)
        self.assertEqual(parameter_spread([]), {})


if __name__ == "__main__":
    unittest.main()
