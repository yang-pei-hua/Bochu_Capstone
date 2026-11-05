"""Integration coverage for staged contract inputs and shared dependencies."""
import tempfile
import unittest
from pathlib import Path

from tools.reconstruction.run_point_cloud_cad import DEFAULT_ENGINE, read_json, write_json
from tools.contracts.run_reconstruction_contracts import DEFAULT_MANIFEST, main


@unittest.skipUnless(DEFAULT_ENGINE.is_file(), "Build cad_reconstruction_validation first")
class ReconstructionContractsTests(unittest.TestCase):
    def run_contracts(self, output, *args):
        code = main(["--executable", str(DEFAULT_ENGINE), "--output", str(output), *map(str, args)])
        return code, read_json(output / "summary.json")

    def test_full_suite_reuses_each_cloud_and_raw_stage(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "suite"
            code, report = self.run_contracts(output)
            self.assertEqual(code, 0, report["results"])
            self.assertEqual(len(report["results"]), 162)
            self.assertEqual(len(list((output / "datasets").glob("*/cloud.ply"))), 9)
            raw = [(r["dataset"], r["module"], r.get("phase")) for r in report["executions"]
                   if r["mode"] == "cloud"]
            self.assertEqual({module for _, module, _ in raw}, set(range(1, 35)))
            self.assertEqual(len(raw), 81)
            self.assertEqual(len(set(raw)), len(raw))

    def test_cases_for_one_stage_share_the_cloud_execution(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = read_json(DEFAULT_MANIFEST)
            gate = next(row for row in manifest["cases"] if row["id"] == "m18.two_boxes.extract")
            gate["upstream_gate"] = True
            other = {**gate, "id": "m18.two_boxes.working_budget", "upstream_gate": False,
                     "metrics": {"working_count": {"min": 1}}}
            manifest["cases"].append(other)
            altered = root / "manifest.json"
            write_json(altered, manifest)
            code, report = self.run_contracts(root / "shared", "--manifest", altered, "--mode", "cloud",
                                              "--case", gate["id"], "--case", other["id"])
            self.assertEqual(code, 0)
            self.assertEqual(len(report["results"]), 2)
            self.assertEqual(len(report["executions"]), 1)

    def test_replay_saved_stage_without_executing_upstream(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, report = self.run_contracts(root / "raw", "--mode", "cloud", "--case", "m19.two_boxes.group")
            self.assertEqual(code, 0, report["results"])
            saved = Path(report["results"][0]["artifact"])
            shared_cloud = root / "raw/datasets/two_boxes/cloud.ply"
            code, report = self.run_contracts(root / "replay", "--mode", "fixture", "--upstream", saved,
                                              "--cloud", shared_cloud, "--case", "m20.two_boxes.boundary")
            self.assertEqual(code, 0, report["results"])
            self.assertEqual([r["module"] for r in report["executions"]], [20])

            artifact = read_json(saved)
            artifact["source_fingerprint"] = "0"
            foreign = root / "foreign.json"
            write_json(foreign, artifact)
            code, report = self.run_contracts(root / "foreign", "--mode", "fixture", "--upstream", foreign,
                                              "--case", "m20.two_boxes.boundary")
            self.assertEqual(code, 1)
            self.assertIn("different point snapshot", report["results"][0]["error"])

    def test_analysis_artifact_replays_without_detection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, report = self.run_contracts(root / "analysis", "--mode", "cloud",
                                              "--case", "m06.surface_box.collect")
            self.assertEqual(code, 0, report["results"])
            self.assertEqual([r["module"] for r in report["executions"]], [1, 2, 3, 4, 6])
            evidence = Path(report["results"][0]["artifact"])
            code, report = self.run_contracts(root / "patches", "--mode", "fixture",
                                              "--upstream", evidence, "--case", "m07.surface_box.patches")
            self.assertEqual(code, 0, report["results"])
            self.assertEqual([r["module"] for r in report["executions"]], [7])
            self.assertEqual(report["results"][0]["metrics"]["patch_count"], 6)

    def test_recognition_replays_saved_host(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, report = self.run_contracts(root / "host", "--mode", "cloud",
                                              "--case", "m12.blind_hole.host")
            self.assertEqual(code, 0, report["results"])
            host = Path(report["results"][0]["artifact"])
            code, report = self.run_contracts(root / "holes", "--mode", "fixture", "--upstream", host,
                                              "--case", "m13.blind_hole.reject_short_wall")
            self.assertEqual(code, 0, report["results"])
            self.assertEqual([r["module"] for r in report["executions"]], [13])
            self.assertEqual(report["results"][0]["metrics"]["hole_count"], 0)

    def test_step_replays_saved_project_body(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, report = self.run_contracts(root / "project", "--mode", "fixture",
                                              "--case", "m32.blind_hole.pcad_roundtrip")
            self.assertEqual(code, 0, report["results"])
            saved = Path(report["results"][0]["artifact"])
            self.assertTrue(Path(read_json(saved)["project"]).is_file())
            code, report = self.run_contracts(root / "export", "--mode", "fixture", "--upstream", saved,
                                              "--case", "m33.blind_hole.step_roundtrip")
            self.assertEqual(code, 0, report["results"])
            self.assertEqual([r["module"] for r in report["executions"]], [33])
            artifact = read_json(Path(report["results"][0]["artifact"]))
            self.assertTrue(Path(artifact["step"]).is_file())

    def test_failed_upstream_blocks_cloud_mode_only(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = read_json(DEFAULT_MANIFEST)
            extract = next(row for row in manifest["cases"] if row["id"] == "m18.two_boxes.extract")
            extract["metrics"]["patch_count"] = 999
            altered = root / "manifest.json"
            write_json(altered, manifest)
            code, report = self.run_contracts(root / "blocked", "--manifest", altered, "--case", "m21.two_boxes.closure")
            self.assertEqual(code, 1)
            direct, blocked = report["results"]
            self.assertEqual(direct["status"], "passed")
            self.assertEqual(blocked["status"], "blocked_upstream")
            self.assertEqual(blocked["blocked_by_module"], 18)
            self.assertEqual([(r["mode"], r["module"]) for r in report["executions"]],
                             [("fixture", 21), ("cloud", 18)])


if __name__ == "__main__":
    unittest.main()
