"""Complete curved solids must export accurately; partial scans must be rejected."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from generate_compound_test_clouds import ENGINE
from generate_curved_test_clouds import KINDS
from run_curved_fit_tests import run_case
from check_cad_validation_stability import geometry_parameters


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

    def test_unrecognized_replay_archives_previous_step(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/"cylinder"
            self.assertTrue(run_case(directory,"cylinder")["passed"])
            self.assertTrue((directory/"reconstructed.step").is_file())
            self.assertTrue(run_case(directory,"cylinder",incomplete=True)["passed"])
            self.assertFalse((directory/"reconstructed.step").exists())
            self.assertEqual(len(list(directory.glob("reconstructed.previous-*.step"))),1)

    def test_same_seed_reproduces_all_curved_geometry(self):
        import json
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
