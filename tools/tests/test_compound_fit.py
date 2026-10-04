"""Ground-truth box-count, complete-shape and STEP regression contracts."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from generate_compound_test_clouds import ENGINE
from run_compound_fit_tests import run_suite


@unittest.skipUnless(ENGINE.is_file(),"Build cad_reconstruction_validation first")
class CompoundFitTests(unittest.TestCase):
    def check_suite(self,noise):
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            report=run_suite(Path(temporary),noise)
        for case in report['cases']:
            with self.subTest(case=case['case'],noise=noise):
                self.assertTrue(case['passed'],case)
                self.assertGreaterEqual(case['accuracy']['symmetric_difference_volume_fraction'],0)
                self.assertLessEqual(case['accuracy']['independent_volume_sampling']['upper_confidence_bound'],0.05)

    def test_exact_surfaces_select_minimum_count_and_export_correct_solids(self):
        self.check_suite(0)

    def test_noisy_surfaces_preserve_count_and_geometry(self):
        self.check_suite(0.02)


if __name__=='__main__':unittest.main()
