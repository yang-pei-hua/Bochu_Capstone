"""Check CLI dispatch, desktop launch and safe cleanup using temporary fixtures."""

from contextlib import redirect_stderr, redirect_stdout
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcad_cli import clean, studio
from pcad_cli.cli import main


class PcadTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "AGENTS.md").write_text("test repository")
        self.outputs = self.root / "outputs"
        self.outputs.mkdir()
        for name in (".gitignore", "README.md"):
            (self.outputs / name).write_text("preserve")
        self.root_patch = patch.object(clean, "REPOSITORY_ROOT", self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.console = io.StringIO()
        self.stdout = redirect_stdout(self.console)
        self.stdout.__enter__()
        self.addCleanup(self.stdout.__exit__, None, None, None)
        self.stderr = redirect_stderr(self.console)
        self.stderr.__enter__()
        self.addCleanup(self.stderr.__exit__, None, None, None)

    def make_artifacts(self):
        case = self.outputs / "case"
        case.mkdir()
        (case / "cloud.ply").write_text("generated")
        (self.outputs / "pipeline.log").write_text("generated")
        source = self.root / "data"
        source.mkdir()
        (source / "model.step").write_text("input")
        return case, source

    def test_clean_removes_generated_files_and_preserves_inputs_and_markers(self):
        _, source = self.make_artifacts()
        self.assertEqual(main(["clean", "output"]), 0)
        self.assertEqual({p.name for p in self.outputs.iterdir()}, {".gitignore", "README.md"})
        self.assertEqual((source / "model.step").read_text(), "input")
        self.assertEqual((self.outputs / "README.md").read_text(), "preserve")

    def test_dry_run_keeps_cloud_and_log(self):
        case, _ = self.make_artifacts()
        self.assertEqual(main(["clean", "output", "--dry-run"]), 0)
        self.assertTrue((case / "cloud.ply").is_file())
        self.assertTrue((self.outputs / "pipeline.log").is_file())

    def test_missing_marker_refuses_cleanup(self):
        case, _ = self.make_artifacts()
        (self.outputs / ".gitignore").unlink()
        self.assertEqual(main(["clean", "output"]), 2)
        self.assertTrue((case / "cloud.ply").is_file())

    def test_cleanup_failure_returns_nonzero_and_reports_target(self):
        self.make_artifacts()
        with patch.object(clean, "remove_entry", side_effect=PermissionError("locked")):
            self.assertEqual(main(["clean", "output"]), 1)
        self.assertIn("locked", self.console.getvalue())

    def test_outside_link_is_unlinked_without_touching_its_target(self):
        outside = self.root / "outside"
        outside.mkdir()
        cloud = outside / "cloud.ply"
        cloud.write_text("outside")
        link = self.outputs / "linked"
        if os.name == "nt":
            # Exact, temporary paths; cmd is used only to create the junction.
            result = subprocess.run(
                ["cmd", "/c", "mklink", "/J", str(link), str(outside)],
                capture_output=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            link.symlink_to(outside, target_is_directory=True)
        self.assertEqual(main(["clean", "output"]), 0)
        self.assertFalse(link.exists())
        self.assertEqual(cloud.read_text(), "outside")

    def test_linked_outputs_root_is_rejected(self):
        with patch.object(clean, "is_link", side_effect=lambda p: p == self.outputs):
            self.assertEqual(main(["clean", "output"]), 2)

    def test_studio_launch_uses_separate_process_and_executable_directory(self):
        executable = self.root / "Studio with spaces.exe"
        executable.write_text("fixture")
        with patch.object(studio.subprocess, "Popen") as launch:
            launch.return_value.pid = 123
            self.assertEqual(main(["studio", "--exe", str(executable)]), 0)
            self.assertEqual(launch.call_args.args[0], [str(executable)])
            self.assertEqual(launch.call_args.kwargs["cwd"], executable.parent)

    def test_studio_dry_run_finds_default_client_without_launching(self):
        executable = self.root / studio.CLIENT_LOCATIONS[0]
        executable.parent.mkdir(parents=True)
        executable.write_text("fixture")
        with patch.object(studio, "REPOSITORY_ROOT", self.root), patch.object(studio.subprocess, "Popen") as launch:
            self.assertEqual(main(["studio", "--dry-run"]), 0)
            launch.assert_not_called()
        self.assertIn(str(executable), self.console.getvalue())

    def test_missing_studio_returns_actionable_error(self):
        with patch.object(studio, "REPOSITORY_ROOT", self.root):
            self.assertEqual(main(["studio"]), 2)
        self.assertIn("--exe", self.console.getvalue())

    @unittest.skipUnless(os.name == "nt", "Windows command wrapper")
    def test_cmd_wrapper_forwards_arguments_and_failure_exit_code(self):
        wrapper = Path(__file__).resolve().parents[1] / "pcad.cmd"
        result = subprocess.run(
            ["cmd", "/d", "/c", str(wrapper), "studio", "--exe", str(self.root / "missing client.exe")],
            cwd=self.root, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("missing client.exe", result.stderr)
        help_result = subprocess.run(
            ["cmd", "/d", "/c", str(wrapper), "--help"],
            cwd=self.root, capture_output=True, text=True,
        )
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        self.assertIn("pcad", help_result.stdout)


if __name__ == "__main__":
    unittest.main()
