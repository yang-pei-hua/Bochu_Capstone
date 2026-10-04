"""Contracts for failure reporting and bounded, log-free diagnostics."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "core" / "reconstruction"))
from diagnose_performance_validation import diagnose
from colmap_reconstruction.pipeline import _Runner, ReconstructionError
import refit_performance_validation as replay


class ValidationDiagnosticsTests(unittest.TestCase):
    def saved_case(self, root):
        case = root / "case"
        case.mkdir()
        for name in ("cloud.ply", "model.step", "texture.png"):
            (case / name).write_text("test snapshot")
        (case / "reconstructed.step").write_text("previous candidate")
        (case / "fitting_report.json").write_text('{"status":"cad_exported"}')
        report = {"artifacts": {"point_cloud": "cloud.ply"},
                  "inputs": {"model": {"snapshot": "model.step"}, "texture": {"snapshot": "texture.png"}},
                  "executables": {"fit": str(case / "model.step")}, "capture": {}, "colmap": {}}
        (case / "validation_report.json").write_text(json.dumps(report))
        return case

    def test_native_exit_cannot_reuse_previous_fitting_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = self.saved_case(root)
            with patch.object(replay, "OUTPUTS_ROOT", root), patch.object(replay.subprocess, "run", return_value=subprocess.CompletedProcess([], 3221225477)):
                result = replay.refit(case)
            self.assertEqual(result["pipeline"]["status"], "failed")
            self.assertEqual(result["fitting"]["status"], "failed")
            self.assertEqual(result["fitting"]["exit_code"], 3221225477)
            self.assertFalse(result["fitting"]["complete_cad_fit"])
            self.assertFalse(result["step_exported"])

    def test_failed_recognition_replay_archives_previous_step(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = self.saved_case(root)

            def failed_recognition(*args, **kwargs):
                (case / "fitting_report.json").write_text(json.dumps({
                    "status": "cad_not_recognized", "complete_cad_fit": False,
                    "recognition": {"recognized": False, "failure_stage": "plane_detection"}}))
                return subprocess.CompletedProcess(args[0], 0)

            with patch.object(replay, "OUTPUTS_ROOT", root), patch.object(replay.subprocess, "run", failed_recognition), patch.object(replay, "write_readme"):
                result = replay.refit(case)
            self.assertFalse(result["step_exported"])
            self.assertFalse(result["fitting"]["complete_cad_fit"])
            self.assertEqual(len(list(case.glob("reconstructed-before-refit-*.step"))), 1)
            self.assertTrue((case / "validation_report.initial.json").is_file())

    def test_unrecognized_cad_is_preserved_without_reading_logs(self):
        with tempfile.TemporaryDirectory() as temporary:
            case = Path(temporary)
            (case / "pipeline_status.json").write_text(json.dumps({"status": "completed"}))
            (case / "fitting_report.json").write_text(json.dumps({
                "status": "cad_not_recognized", "complete_cad_fit": False,
                "recognition": {"recognized": False, "reason": "unsupported combination"}}))
            (case / "pipeline.log").write_text("must not enter default diagnostic")
            original = Path.open

            def guarded(path, *args, **kwargs):
                if path.suffix == ".log":
                    raise AssertionError("Default diagnostics opened a log")
                return original(path, *args, **kwargs)

            with patch.object(Path, "open", guarded):
                result = diagnose(case)
            self.assertFalse(result["fitting"]["complete_cad_fit"])
            self.assertFalse(result["step_exported"])
            self.assertNotIn("log_excerpt", result)

    def test_colmap_failure_publishes_failed_stage_with_exit_code(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            runner = _Runner(Path("colmap.exe"), directory / "colmap.log")
            with patch("colmap_reconstruction.pipeline.subprocess.run", side_effect=subprocess.CalledProcessError(9, "colmap")):
                with self.assertRaises(ReconstructionError):
                    runner.run("patch_match_stereo", "--workspace_path", directory)
            progress = json.loads((directory / "progress.json").read_text())
            self.assertEqual(progress["current"]["status"], "failed")
            self.assertEqual(progress["current"]["exit_code"], 9)
            self.assertEqual(progress["stages"][0]["command"], "patch_match_stereo")


if __name__ == "__main__":
    unittest.main()
