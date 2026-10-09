"""Safety and provenance contracts for the independent cloud evaluator."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_cloud_reliability import evaluate, manifest_cases, stress_cloud, write_cloud


class CloudReliabilityTests(unittest.TestCase):
    def test_failed_accuracy_or_stale_step_cannot_pass(self):
        failed = {"status": "cad_accuracy_failed", "complete_cad_fit": False}
        self.assertFalse(evaluate(failed, 0, True, "accurate-or-reject")["passed"])
        rejected = {"status": "cad_not_recognized"}
        self.assertFalse(evaluate(rejected, 0, True, "reject")["passed"])
        self.assertFalse(evaluate(rejected, "timeout", False, "reject")["passed"])
        self.assertTrue(evaluate(rejected, 0, False, "reject")["passed"])

    def test_missing_attributes_are_reproducible_without_fitting(self):
        first = stress_cloud("torus", "missing-normals", 19)
        self.assertEqual(first, stress_cloud("torus", "missing-normals", 19))
        absent = sum(p[3:] == [0, 0, 0] for p in first[1])
        self.assertGreater(absent, .75*len(first[1]))
        self.assertLess(absent, .85*len(first[1]))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/"cloud.ply"
            write_cloud(path, first[1], omit_normals=True)
            self.assertNotIn("property float nx", path.read_text(encoding="ascii"))

    def test_recorded_dataset_requires_declared_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/"manifest.json"
            path.write_text(json.dumps({"cases": [{"id": "sample", "provenance": "synthetic"}]}))
            with self.assertRaisesRegex(ValueError, "provenance"):
                manifest_cases(path)

    def test_recorded_dataset_cannot_escape_output_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/"manifest.json"
            for name in ("..", "../outside", "D:/outside"):
                path.write_text(json.dumps({"cases": [{"id": name, "provenance": "real-capture", "source": "scanner"}]}))
                with self.subTest(name=name), self.assertRaisesRegex(ValueError, "directory name"):
                    manifest_cases(path)


if __name__ == "__main__":
    unittest.main()
