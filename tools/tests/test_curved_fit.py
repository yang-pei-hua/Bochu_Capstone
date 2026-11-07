"""Complete curved solids must export accurately; partial scans must be rejected."""
import json
import math
from pathlib import Path
import random
import subprocess
import tempfile
import unittest

from tools.fixtures.generate_compound_test_clouds import ENGINE
from tools.fixtures.generate_curved_test_clouds import KINDS, definition, samples
from tools.validation.run_curved_fit_tests import run_case
from tools.diagnostics.check_cad_validation_stability import geometry_parameters


@unittest.skipUnless(ENGINE.is_file(),"Build cad_reconstruction_validation first")
class CurvedFitTests(unittest.TestCase):
    def check_shapes(self,noise=0,rotated=False,incomplete=False):
        with tempfile.TemporaryDirectory() as temporary:
            for kind in KINDS:
                with self.subTest(kind=kind,noise=noise,rotated=rotated,incomplete=incomplete):
                    result=run_case(Path(temporary)/kind,kind,noise,rotated,2,incomplete)
                    self.assertTrue(result["passed"],result)
                    if not incomplete:
                        self.assertTrue(result["accuracy"]["primitive_dimensions_pass"])
                        self.assertLess(result["accuracy"]["source_boundary_distance_max_mm"],.1)

    def test_exact_complete_primitives(self): self.check_shapes()
    def test_rotated_noisy_complete_primitives(self): self.check_shapes(.02,True)
    def test_missing_caps_and_incomplete_curves_are_rejected(self): self.check_shapes(incomplete=True)

    def test_observed_walls_export_without_caps(self):
        with tempfile.TemporaryDirectory() as temporary:
            for shape in ("cylinder", "frustum"):
                with self.subTest(shape=shape):
                    case = definition(shape, rotated=True)
                    points = samples(case, noise=.005, incomplete=True)
                    if shape == "cylinder":
                        points = [p[:3] + [-n for n in p[3:]] for p in points]
                    directory = Path(temporary) / shape
                    result, report, step = self.fit_observed(directory, points)
                    self.assertEqual(result.returncode, 0, report)
                    self.assertEqual(report["kind"], case["geometry"])
                    self.assertEqual(report["radial_normal_direction"], "inward" if shape == "cylinder" else "outward")
                    self.assertGreaterEqual(report["surface_coverage"], .98)
                    self.assertTrue(report["step_round_trip_verified"])
                    self.assertFalse(report["cad_constructed"])
                    self.assertEqual(report["solids"], 0)
                    self.assertTrue(step.is_file())
            half_wall = [point for index, point in enumerate(points) if index % 96 < 48]
            result, report, step = self.fit_observed(Path(temporary) / "half", half_wall)
            self.assertEqual(result.returncode, 0, report)
            self.assertLess(report["angular_span_rad"], 3.2)
            self.assertGreater(report["angular_span_rad"], 3)
            self.assertTrue(step.exists())
            bands = [point for index, point in enumerate(points) if index % 48 < 24]
            result, report, step = self.fit_observed(Path(temporary) / "disjoint", bands)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["face_count"], 2)
            self.assertEqual([face["kind"] for face in report["faces"]], ["cone", "cone"])
            self.assertLess(sum(face["angular_span_rad"] for face in report["faces"]), 3.2)
            self.assertGreaterEqual(report["surface_coverage"], .98)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_observed_planar_face_preserves_inner_boundary(self):
        points = [[x, y, 0, 0, 0, 1] for x in range(-15, 16) for y in range(-15, 16)
                  if x*x + y*y >= 49]
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "plane", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "plane")
            self.assertEqual(report["inner_loops"], 1)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_sparse_area_sampled_drill_tip(self):
        rng = random.Random(24)
        radius, height = 6, 3.5
        slant = math.hypot(radius, height)
        points = []
        for _ in range(475):
            t = math.sqrt(rng.random())
            angle = rng.uniform(0, math.tau)
            x, y = math.cos(angle), math.sin(angle)
            points.append([radius*t*x, radius*t*y, height*t, -height*x/slant, -height*y/slant, radius/slant])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "drill-tip", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "cone")
            self.assertEqual(report["radial_normal_direction"], "inward")
            self.assertEqual(report["solids"], 0)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_observed_cylinder_with_sloped_boundary(self):
        radius = 8
        points = []
        for i in range(30):
            angle = (i+.5)/30*1.6-.8
            x, y = math.cos(angle), math.sin(angle)
            for j in range(30):
                z = (j+.5)*.4
                if z <= 5+7*(i+.5)/30:
                    points.append([radius*x, radius*y, z, -x, -y, 0])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "sloped-wall", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "cylinder")
            self.assertEqual(report["trim_kind"], "observed_parameter_boundary")
            self.assertGreaterEqual(report["trim_coverage"], .98)
            self.assertEqual(report["solids"], 0)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_coaxial_walls_and_annular_faces_stay_open(self):
        walls, rims = [], []
        for i in range(64):
            a = (i+.5)*math.tau/64
            x, y = math.cos(a), math.sin(a)
            for j in range(40):
                t = (j+.1)/39.2
                walls.append([7*x, 7*y, 14*t, -x, -y, 0])
                walls.append([10*x, 10*y, 9+5*t, x, y, 0])
            for j in range(20):
                t = (j+.5)/20
                r = math.sqrt(7*7+(10*10-7*7)*t)
                rims.append((i,[r*x, r*y, 14, 0, 0, 1]))
                r = math.sqrt(10*10+(12*12-10*10)*t)
                rims.append((i,[r*x, r*y, 9, 0, 0, 1]))
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "coaxial", walls + [p for _, p in rims])
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "coaxial")
            self.assertEqual(report["face_count"], 4)
            self.assertEqual(report["solids"], 0)
            self.assertGreaterEqual(report["surface_coverage"], .98)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())
            incomplete = walls + [p for i, p in rims if i>=16]
            result, report, step = self.fit_observed(Path(temporary) / "missing-rim-sector", incomplete)
            self.assertEqual(result.returncode, 1, report)
            self.assertFalse(step.exists())

    def test_mixed_observed_faces_keep_independent_trims(self):
        points = [[-20+i*.4, -4+j*.4, 0, 0, 0, 1] for i in range(23) for j in range(20)]
        for i in range(32):
            a = (i+.5)*2.1/32-.8
            x, y = math.cos(a), math.sin(a)
            for j in range(20):
                t = (j+.5)/20
                points.append([5*x, 5*y, 9*t, x, y, 0])
                points.append([5*x, 5*y, 40+9*t, x, y, 0])
                z = 2+4*t
                points.append([17+z*x, z*y, z, x/math.sqrt(2), y/math.sqrt(2), -1/math.sqrt(2)])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "mixed", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "mixed")
            self.assertEqual(sorted(face["kind"] for face in report["faces"]), ["cone", "cylinder", "cylinder", "plane"])
            self.assertEqual(report["face_count"], 4)
            spans = sorted(face["axial_bounds_mm"] for face in report["faces"] if face["kind"] == "cylinder")
            self.assertGreater(spans[1][0]-spans[0][1], 20)
            self.assertEqual(report["solids"], 0)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())
            small_wall = []
            for i in range(9):
                a = (i+.5)*1.6/9-.8
                x, y = math.cos(a), math.sin(a)
                for j in range(7):
                    small_wall.append([30+4*x, 4*y, (j+.5)*.3, x, y, 0])
            result, report, step = self.fit_observed(Path(temporary) / "small-residual", points+small_wall)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["kind"], "mixed")
            self.assertEqual(report["face_count"], 5)
            self.assertTrue(any(abs(f.get("radius_mm",0)-4)<.01 for f in report["faces"]))
            scattered = [[40+i*2, 17+(i*i%67), 70+(i*i*i%107), 0, 1, 0] for i in range(100)]
            result, report, step = self.fit_observed(Path(temporary) / "unexplained", points+scattered)
            self.assertEqual(result.returncode, 1, report)
            self.assertFalse(step.exists())

    def test_mixed_observed_faces_include_toroidal_round(self):
        points = [[-20+i*.4, -4+j*.4, 0, 0, 0, 1] for i in range(23) for j in range(20)]
        for i in range(32):
            u = (i+.5)*1.6/32-.8
            cu, su = math.cos(u), math.sin(u)
            for j in range(20):
                v = (j+.5)*1.55/20
                cv, sv = math.cos(v), math.sin(v)
                radius = 12+2.5*cv
                points.append([radius*cu, radius*su, 7+2.5*sv, cu*cv, su*cv, sv])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "toroidal-round", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(sorted(f["kind"] for f in report["faces"]), ["plane", "torus"])
            torus = next(f for f in report["faces"] if f["kind"] == "torus")
            self.assertAlmostEqual(torus["major_radius_mm"], 12, delta=.01)
            self.assertAlmostEqual(torus["minor_radius_mm"], 2.5, delta=.01)
            self.assertEqual(report["solids"], 0)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_observed_cylinder_preserves_disconnected_angular_patches(self):
        radius = 6
        points = []
        for start, width in [(0.15, 1.55), (3.2, 2.25)]:
            for i in range(18):
                angle = start + (i+.5)*width/18
                x, y = math.cos(angle), math.sin(angle)
                for j in range(12):
                    points.append([radius*x, radius*y, (j+.5)*.7, x, y, 0])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "split-cylinder", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(report["face_count"], 2)
            self.assertGreaterEqual(report["surface_coverage"], .98)
            self.assertEqual([face["kind"] for face in report["faces"]], ["cylinder", "cylinder"])
            self.assertLess(sum(face["angular_span_rad"] for face in report["faces"]), 4.0)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def test_observed_spline_preserves_a_nonanalytic_transition(self):
        points = [[-30+i*.4, -4+j*.4, 0, 0, 0, 1] for i in range(24) for j in range(20)]
        for i in range(13):
            for j in range(13):
                x, y = -3+i*.5, -3+j*.5
                z = 7+.003*(x**3-y**3)+.12*x*y+.001*(x**4+y**4)
                dx = .009*x*x+.12*y+.004*x**3
                dy = -.009*y*y+.12*x+.004*y**3
                length = math.sqrt(1+dx*dx+dy*dy)
                points.append([x, y, z, -dx/length, -dy/length, 1/length])
        with tempfile.TemporaryDirectory() as temporary:
            result, report, step = self.fit_observed(Path(temporary) / "spline-transition", points)
            self.assertEqual(result.returncode, 0, report)
            self.assertEqual(sorted(face["kind"] for face in report["faces"]), ["bspline", "plane"])
            spline = next(face for face in report["faces"] if face["kind"] == "bspline")
            self.assertGreaterEqual(spline["holdout_coverage"], .95)
            self.assertLessEqual(spline["maximum_height_error_mm"], .05)
            self.assertTrue(report["step_round_trip_verified"])
            self.assertTrue(step.is_file())

    def fit_observed(self, directory, points):
        directory.mkdir()
        cloud = directory / "cloud.ply"
        header = f"ply\nformat ascii 1.0\nelement vertex {len(points)}\n"
        header += "".join(f"property float {field}\n" for field in ("x", "y", "z", "nx", "ny", "nz"))
        cloud.write_text(header + "end_header\n" + "".join(" ".join(map(str, p)) + "\n" for p in points), encoding="ascii")
        (directory / "cloud.json").write_text('{"unit":"mm"}', encoding="ascii")
        step, report = directory / "surface.step", directory / "report.json"
        result = subprocess.run([ENGINE, "fit-observed-surface", cloud, report, step, "0.05"],
                                capture_output=True, timeout=30)
        return result, json.loads(report.read_bytes()), step

    def test_unrecognized_replay_archives_previous_step(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/"cylinder"
            self.assertTrue(run_case(directory,"cylinder")["passed"])
            self.assertTrue((directory/"reconstructed.step").is_file())
            self.assertTrue(run_case(directory,"cylinder",incomplete=True)["passed"])
            self.assertFalse((directory/"reconstructed.step").exists())
            self.assertEqual(len(list(directory.glob("reconstructed.previous-*.step"))),1)

    def test_same_seed_reproduces_all_curved_geometry(self):
        with tempfile.TemporaryDirectory() as temporary:
            for kind in KINDS:
                with self.subTest(kind=kind):
                    geometries=[]
                    for index in range(3):
                        result=run_case(Path(temporary)/kind/str(index),kind,.02,True,2)
                        self.assertTrue(result["passed"],result)
                        report=json.loads(Path(result["report"]).read_text())
                        geometries.append(geometry_parameters(report))
                    self.assertEqual(geometries[0],geometries[1])
                    self.assertEqual(geometries[1],geometries[2])


if __name__=="__main__":unittest.main()
