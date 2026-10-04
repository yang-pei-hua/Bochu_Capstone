"""Verify the CAD export/accuracy branch on a trimmed box-with-hole cloud.

This is an isolated geometry contract test, never a substitute for the photo run.
"""
import json
import math
import os
import random
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
EXECUTABLE = ROOT / "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe"
CASE = ROOT / "data/performance-validation/box-through-hole"


@unittest.skipUnless(EXECUTABLE.is_file() and (CASE / "model.step").is_file(), "Build and generate standard fixtures first")
class CadValidationTests(unittest.TestCase):
    def test_box_and_hole_exports_and_passes_ground_truth(self):
        self.validate_cloud(exact=False)

    def validate_exact_cloud(self):
        return self.validate_cloud(exact=True)

    def test_exact_box_and_hole_exports_and_passes_ground_truth(self):
        self.validate_cloud(exact=True)

    @unittest.skipUnless((ROOT / "outputs/performance-validation-box-through-hole/dense_cloud.ply").is_file(),
                         "Saved photo validation cloud is required")
    def test_photo_cloud_keeps_one_hole_across_sample_streams(self):
        from check_cad_validation_stability import replay_case
        case = ROOT / "outputs/performance-validation-box-through-hole"
        with tempfile.TemporaryDirectory() as temporary:
            for seed in (2, 14):
                directory = Path(temporary) / str(seed)
                directory.mkdir()
                with self.subTest(seed=seed):
                    report = replay_case(case, directory, seed)
                    self.assertEqual(len(report["through_holes"]), 1)

    def validate_cloud(self, exact=False, seed=4500):
        p = json.loads((CASE / "case.json").read_text())["parameters"]
        x, y, z, r = p["size_x"], p["size_y"], p["size_z"], p["hole_diameter"] / 2
        points = []
        # Six exterior faces, with the two circular openings trimmed out.
        for axis, sizes in enumerate(((y, z), (x, z), (x, y))):
            for sign in (-1, 1):
                for a in range(int(sizes[0]) + 1):
                    for b in range(int(sizes[1]) + 1):
                        u, v = a - sizes[0]/2, b - sizes[1]/2
                        if axis == 2 and u*u + v*v < r*r:
                            continue
                        position = [0.0, 0.0, 0.0]
                        normal = [0.0, 0.0, 0.0]
                        position[axis] = sign * (x, y, z)[axis] / 2
                        other = [i for i in range(3) if i != axis]
                        position[other[0]], position[other[1]] = u, v
                        normal[axis] = sign
                        points.append((*position, *normal))
        for i in range(128):
            angle = 2 * math.pi * i / 128
            c, s = math.cos(angle), math.sin(angle)
            for j in range(int(z) + 1):
                points.append((r*c, r*s, j-z/2, -c, -s, 0))
        if not exact:
            rng = random.Random(4500)
            points = [tuple(value + rng.gauss(0, 0.005 if index < 3 else 0.0001)
                            for index, value in enumerate(point)) for point in points]
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cloud = directory / "cloud.ply"
            cloud.write_text("ply\nformat ascii 1.0\nelement vertex " + str(len(points)) + "\n"
                             + "".join("property float " + field + "\n" for field in ("x", "y", "z", "nx", "ny", "nz"))
                             + "end_header\n" + "".join(" ".join(map(str, point)) + "\n" for point in points))
            (directory / "reconstruction.json").write_text('{"unit":"mm"}')
            report_path, step = directory / "report.json", directory / "result.step"
            result = subprocess.run([str(EXECUTABLE), "fit", str(cloud), str(report_path), str(step),
                                     str(CASE / "model.step"), str(CASE / "case.json")], capture_output=True,
                                    env={**os.environ, "PCAD_RANSAC_SEED": str(seed)}, timeout=120)
            self.assertTrue(report_path.is_file(), {
                "exit_code": result.returncode,
                "diagnostic": (result.stdout + result.stderr).decode("utf-8", errors="replace")[-1200:]})
            report = json.loads(report_path.read_text())
            diagnostic = {key: report.get(key) for key in ("status", "stage", "recognition", "accuracy", "error")}
            crash_path = report_path.with_name(report_path.name + '.native-crash.json')
            if crash_path.is_file():
                diagnostic['native_crash'] = json.loads(crash_path.read_text())
            self.assertEqual(result.returncode, 0, diagnostic)
            self.assertEqual(report["status"], "cad_accuracy_passed", diagnostic)
            self.assertEqual(len(report["through_holes"]), 1)
            self.assertTrue(report["cad"]["step_round_trip_verified"])
            self.assertLess(report["accuracy"]["symmetric_difference_volume_fraction"], 0.01)
            self.assertTrue(step.is_file())
            return report


if __name__ == "__main__":
    unittest.main()
