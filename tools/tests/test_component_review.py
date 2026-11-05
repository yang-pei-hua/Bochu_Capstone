"""Persistent classification, version identity, recovery and native UI contracts."""
import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

from tools.component_review.store import Dataset, ReviewLock, ReviewStore, digest


def fixture(root):
    rows = []
    for number in range(1, 5):
        name = f"sample_{number:03d}"
        directory = root/name;directory.mkdir()
        (directory/"cloud.ply").write_text(f"sample {number}", encoding="utf-8")
        (directory/"cloud.json").write_text('{"unit":"mm"}', encoding="utf-8")
        Image.new("RGB", (320, 240), (number*40, 120, 190)).save(directory/"views.png")
        rows.append({"name": name, "part_id": number, "points": number*100,
                     "closed": number==1, "directory": name, "cloud_sha256": digest(directory/"cloud.ply")})
    (root/"summary.json").write_text(json.dumps({"schema": "paramcad.component-cloud-review",
        "cases": [{"name": "sample", "exported_clouds": rows}]}), encoding="utf-8")
    Image.new("RGB", (1200, 800), "lightgray").save(root/"sample_locator.png")
    return Dataset(root)


class ReviewStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.dataset = fixture(self.root)
        self.store = ReviewStore(self.dataset)

    def tearDown(self):
        self.temp.cleanup()

    def test_three_decisions_resume_and_fit_set_follow_revisions(self):
        first, second, third, fourth = self.dataset.items
        self.store.decide(first, "problem", "多个部分混在一起", second.name)
        self.store.decide(second, "correct", "可以拟合, 保留法向", third.name)
        self.store.decide(third, "deferred", "待检查", fourth.name)
        reopened = ReviewStore(self.dataset)
        self.assertEqual(reopened.data["cursor"], fourth.name)
        self.assertEqual(reopened.counts(), {"pending": 1, "problem": 1, "correct": 1, "deferred": 1})
        fit = json.loads((self.store.directory/"fit_test_set.json").read_text(encoding="utf-8"))
        self.assertEqual([r["name"] for r in fit["cases"]], [second.name])
        with (self.store.directory/"decisions.csv").open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[1]["note"], "可以拟合, 保留法向")
        reopened.decide(second, "problem", "改判")
        fit = json.loads((self.store.directory/"fit_test_set.json").read_text(encoding="utf-8"))
        self.assertEqual(fit["count"], 0)
        self.assertTrue(self.store.path.with_suffix(".json.bak").is_file())

    def test_unsaved_decision_cannot_advance_after_atomic_write_failure(self):
        before = copy.deepcopy(self.store.data)
        real_replace = __import__("os").replace
        def fail_current(source, destination):
            if Path(destination) == self.store.path:
                raise OSError("simulated disk failure")
            return real_replace(source, destination)
        with patch("tools.component_review.store.os.replace", side_effect=fail_current):
            with self.assertRaises(OSError):
                self.store.decide(self.dataset.items[0], "correct", cursor=self.dataset.items[1].name)
        self.assertEqual(self.store.data, before)
        self.assertEqual(json.loads(self.store.path.read_text(encoding="utf-8")), before)

    def test_changed_cloud_is_not_silently_retained_in_fit_set(self):
        item = self.dataset.items[0]
        self.store.decide(item, "correct")
        item.cloud.write_text("changed observations", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "已改变"):
            self.store.decide(item, "correct")
        changed = Dataset(self.root)
        reopened = ReviewStore(changed)
        self.assertEqual(reopened.decision(changed.items[0]), "pending")
        self.assertEqual(json.loads((reopened.directory/"fit_test_set.json").read_text(encoding="utf-8"))["count"], 0)

    def test_pending_notes_and_cursor_survive_without_a_classification(self):
        self.store.checkpoint(self.dataset.items[0], "先记备注", self.dataset.items[1].name)
        reopened = ReviewStore(self.dataset)
        self.assertEqual(reopened.note(self.dataset.items[0]), "先记备注")
        self.assertEqual(reopened.decision(self.dataset.items[0]), "pending")
        self.assertEqual(reopened.data["cursor"], self.dataset.items[1].name)

    def test_corrupt_json_is_not_replaced(self):
        self.store.path.write_text("bad existing user data", encoding="utf-8")
        with self.assertRaises(ValueError):
            ReviewStore(self.dataset)
        self.assertEqual(self.store.path.read_text(encoding="utf-8"), "bad existing user data")

    def test_second_window_lock_releases_on_close(self):
        first = ReviewLock(self.store.directory)
        try:
            with self.assertRaises(RuntimeError):
                ReviewLock(self.store.directory)
        finally:
            first.close()
        reopened = ReviewLock(self.store.directory)
        reopened.close()


class ReviewUiTests(unittest.TestCase):
    def test_buttons_filters_zoom_and_resume_use_real_widgets(self):
        import tkinter as tk
        from tools.component_review.app import ReviewApp
        with tempfile.TemporaryDirectory() as temp:
            dataset = fixture(Path(temp));store = ReviewStore(dataset)
            root = tk.Tk();root.withdraw()
            try:
                app = ReviewApp(root, store);root.update()
                first, second, third, fourth = dataset.items
                self.assertEqual(app.current.name, first.name)
                app.problem_button.invoke();root.update()
                self.assertEqual(app.current.name, second.name)
                app.correct_button.invoke();root.update()
                self.assertEqual(app.current.name, third.name)
                app.defer_button.invoke();root.update()
                self.assertEqual(app.current.name, fourth.name)
                self.assertEqual(store.counts(), {"pending": 1, "problem": 1, "correct": 1, "deferred": 1})
                app.viewer.actual_size();app.viewer.zoom(4);root.update()
                self.assertEqual(app.viewer.scale, 4.)
                app.toggle_locator();root.update()
                self.assertEqual(app.mode, "locator")
                app.status_filter.set("划分正确");app.change_filter();root.update()
                self.assertEqual([i.name for i in app.visible], [second.name])
                app.problem_button.invoke();root.update()
                self.assertIsNone(app.current)
                self.assertEqual(json.loads((store.directory/"fit_test_set.json").read_text(encoding="utf-8"))["count"], 0)
                app.close()
            finally:
                try:root.destroy()
                except tk.TclError:pass
            self.assertIn(ReviewStore(dataset).data["cursor"], dataset.by_name)


if __name__ == "__main__":
    unittest.main()
