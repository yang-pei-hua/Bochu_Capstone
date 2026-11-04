"""Dataset identity and durable, per-component human decisions."""
from __future__ import annotations

import copy
import csv
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile


LABELS = {"pending": "未审核", "problem": "点云划分有问题",
          "correct": "点云划分正确（可以作为拟合的测试集）", "deferred": "暂不做决定"}


def timestamp():
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")


def digest(path: Path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            value.update(block)
    return value.hexdigest()


def atomic_bytes(path: Path, content: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.name+".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(handle, "wb") as stream:
            stream.write(content);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2)+"\n").encode("utf-8")


def child(root, value):
    path = (root/value).resolve()
    path.relative_to(root)
    return path


@dataclass(frozen=True)
class Item:
    name: str
    parent: str
    number: int
    points: int
    closed: bool
    directory: Path
    image: Path
    cloud: Path
    expected_hash: str
    actual_hash: str

    @property
    def current(self):
        return self.actual_hash == self.expected_hash


class Dataset:
    def __init__(self, root):
        self.root = Path(root).resolve()
        manifest = json.loads((self.root/"summary.json").read_text(encoding="utf-8"))
        if manifest.get("schema") != "paramcad.component-cloud-review":
            raise ValueError("请选择包含 summary.json 的候选点云检查包目录。")
        self.items = []
        self.locators = {}
        self.parents = []
        for case in manifest["cases"]:
            self.parents.append(case["name"])
            self.locators[case["name"]] = self.root/f"{case['name']}_locator.png"
            # Candidate numbers remain unchanged; residual 000 comes last.
            for row in sorted(case["exported_clouds"], key=lambda r: (r["part_id"] == 0, r["part_id"])):
                directory = child(self.root, row["directory"])
                cloud, image = directory/"cloud.ply", directory/"views.png"
                if not cloud.is_file() or not image.is_file():
                    raise ValueError(f"缺少点云或拼图：{row['name']}")
                self.items.append(Item(row["name"], case["name"], row["part_id"], row["points"],
                    row["closed"], directory, image, cloud, row["cloud_sha256"], digest(cloud)))
        self.by_name = {item.name: item for item in self.items}
        if not self.items or len(self.by_name) != len(self.items):
            raise ValueError("检查包为空或包含重复编号。")
        self.identity = hashlib.sha256("\n".join(f"{i.name}:{i.actual_hash}" for i in self.items).encode()).hexdigest()


class ReviewStore:
    def __init__(self, dataset: Dataset, directory=None):
        self.dataset = dataset
        self.directory = Path(directory).resolve() if directory else dataset.root/"review"
        self.path = self.directory/"decisions.json"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.last_export_error = ""
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if data.get("schema") != "paramcad.component-review-decisions" or not isinstance(data.get("records"), dict):
                raise ValueError("审核记录格式不正确，未覆盖已有文件。")
            if any(not isinstance(r, dict) or r.get("decision") not in LABELS for r in data["records"].values()):
                raise ValueError("审核记录包含无效判断，未覆盖已有文件。")
            self.data = data
        else:
            self.data = {"schema": "paramcad.component-review-decisions", "version": 1,
                         "dataset_root": str(dataset.root), "dataset_identity": dataset.identity,
                         "cursor": dataset.items[0].name, "records": {}, "updated_at": timestamp()}
            atomic_bytes(self.path, encoded(self.data))
        self.write_exports()

    def record(self, item):
        saved = self.data["records"].get(item.name)
        if saved and saved.get("cloud_sha256") == item.actual_hash:
            return saved
        return None

    def decision(self, item):
        record = self.record(item)
        return record["decision"] if record else "pending"

    def note(self, item):
        record = self.record(item)
        return record.get("note", "") if record else ""

    def counts(self):
        result = dict.fromkeys(LABELS, 0)
        for item in self.dataset.items:
            result[self.decision(item)] += 1
        return result

    def _publish(self, candidate):
        candidate["updated_at"] = timestamp()
        candidate["dataset_root"] = str(self.dataset.root)
        candidate["dataset_identity"] = self.dataset.identity
        # Do not advance in-memory state until the authoritative JSON is durable.
        if self.path.exists():
            atomic_bytes(self.path.with_suffix(".json.bak"), self.path.read_bytes())
        atomic_bytes(self.path, encoded(candidate))
        self.data = candidate
        self.write_exports()

    def decide(self, item, decision, note="", cursor=None):
        if decision not in ("problem", "correct", "deferred", "pending"):
            raise ValueError("无效审核判断。")
        if decision == "correct" and (not item.current or digest(item.cloud) != item.expected_hash):
            raise ValueError("点云文件已改变，与拼图来源不一致；请重新生成拼图后再纳入测试集。")
        candidate = copy.deepcopy(self.data)
        candidate["records"][item.name] = {
            "name": item.name, "parent_case": item.parent, "part_id": item.number,
            "decision": decision, "label": LABELS[decision], "note": note,
            "cloud": item.cloud.relative_to(self.dataset.root).as_posix(),
            "image": item.image.relative_to(self.dataset.root).as_posix(),
            "cloud_sha256": item.actual_hash, "point_count": item.points,
            "algorithm_closed": item.closed, "reviewed_at": timestamp()}
        if cursor is not None:
            candidate["cursor"] = cursor
        self._publish(candidate)

    def checkpoint(self, item, note, cursor):
        if note != self.note(item):
            self.decide(item, self.decision(item), note, cursor)
        elif self.data.get("cursor") != cursor:
            candidate = copy.deepcopy(self.data)
            candidate["cursor"] = cursor
            self._publish(candidate)

    def write_exports(self):
        # These are derived views. A failed export never discards saved JSON.
        try:
            csv_file = io.StringIO(newline="")
            fields = ["name", "parent_case", "part_id", "decision", "label", "eligible_fit_test",
                      "note", "cloud", "image", "point_count", "algorithm_closed", "reviewed_at"]
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            accepted = []
            for item in self.dataset.items:
                saved = self.record(item) or {}
                decision = saved.get("decision", "pending")
                eligible = decision == "correct" and item.current
                cloud = item.cloud.relative_to(self.dataset.root).as_posix()
                image = item.image.relative_to(self.dataset.root).as_posix()
                writer.writerow({"name": item.name, "parent_case": item.parent, "part_id": item.number,
                    "decision": decision, "label": LABELS[decision], "eligible_fit_test": eligible,
                    "note": saved.get("note", ""), "cloud": cloud, "image": image,
                    "point_count": item.points, "algorithm_closed": item.closed,
                    "reviewed_at": saved.get("reviewed_at", "")})
                if eligible:
                    accepted.append({"name": item.name, "parent_case": item.parent, "part_id": item.number,
                                     "cloud": cloud, "image": image, "cloud_sha256": item.actual_hash,
                                     "unit_sidecar": item.cloud.with_suffix(".json").relative_to(self.dataset.root).as_posix(),
                                     "note": saved.get("note", "")})
            atomic_bytes(self.directory/"decisions.csv", csv_file.getvalue().encode("utf-8-sig"))
            atomic_bytes(self.directory/"fit_test_set.json", encoded({
                "schema": "paramcad.human-reviewed-fit-test-set", "version": 1,
                "dataset_root": str(self.dataset.root), "path_base": "dataset_root",
                "updated_at": self.data["updated_at"], "count": len(accepted), "cases": accepted}))
            self.last_export_error = ""
        except OSError as error:
            self.last_export_error = str(error)


class ReviewLock:
    """OS-held lock: closing/crashing releases it without stale lock-file cleanup."""
    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.stream = (directory/"review.lock").open("a+b")
        self.stream.seek(0, os.SEEK_END)
        if self.stream.tell() == 0:
            self.stream.write(b"0");self.stream.flush()
        self.stream.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.stream.close()
            raise RuntimeError("此检查包的审核窗口已经打开，请使用现有窗口。") from None

    def close(self):
        self.stream.close()
