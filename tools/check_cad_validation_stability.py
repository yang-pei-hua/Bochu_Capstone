#!/usr/bin/env python3
"""Repeat CAD fitting and report failures and geometric parameter spread.

Reads structured reports only. Saved photo clouds can be replayed with --case;
this does not repeat photograph capture or COLMAP reconstruction.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools" / "tests"))
from test_cad_validation import CadValidationTests, EXECUTABLE
from refit_performance_validation import local_artifact
from run_performance_validation import hidden_process_options


def geometry_parameters(report: dict) -> dict:
    """Exclude run paths, timings and proposal bookkeeping from repeatability."""
    cad = report.get("cad", {})
    compound = report.get("compound_recognition", {})
    return {
        "box_dimensions_mm": report.get("box_recognition", {}).get("dimensions_mm"),
        "holes": [{key: hole[key] for key in
                   ("diameter_mm", "axis_point_mm", "axis_direction")}
                  for hole in report.get("through_holes", [])],
        "cad_bounds_mm": cad.get("bounding_box_mm"),
        "cad_volume_mm3": cad.get("volume_mm3"),
        "boxes": compound.get("boxes"),
        "prisms": compound.get("prisms"),
    }


def parameter_spread(geometries: list[dict]) -> dict:
    def flatten(value, prefix=""):
        if isinstance(value, dict):
            for key, item in value.items():
                yield from flatten(item, f"{prefix}.{key}".lstrip("."))
        elif isinstance(value, list):
            for index, item in enumerate(value):
                yield from flatten(item, f"{prefix}[{index}]")
        elif isinstance(value, (float, int)):
            yield prefix, value
    rows = [dict(flatten(item)) for item in geometries]
    if not rows:
        return {}
    keys = set.intersection(*(set(row) for row in rows))
    return {key: {"minimum": min(row[key] for row in rows),
                  "maximum": max(row[key] for row in rows),
                  "spread": max(row[key] for row in rows) - min(row[key] for row in rows)}
            for key in sorted(keys)}


def replay_case(case: Path, directory: Path, seed: int) -> dict:
    manifest = json.loads((case / "validation_report.json").read_text(encoding="utf-8"))
    report_path = directory / "fitting_report.json"
    command = [str(EXECUTABLE), "fit",
               str(local_artifact(case, manifest["artifacts"]["point_cloud"])),
               str(report_path), str(directory / "reconstructed.step")]
    expected = manifest["inputs"].get("expected")
    if not expected:
        raise ValueError("Saved case must have source-model accuracy expectations")
    command.extend([str(local_artifact(case, manifest["inputs"]["model"]["snapshot"])),
                    str(local_artifact(case, expected["snapshot"]))])
    with (directory / "fitting.log").open("wb") as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=120,
                                 env={**os.environ, "PCAD_RANSAC_SEED": str(seed)},
                                 **hidden_process_options())
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}
    if process.returncode or report.get("complete_cad_fit") is not True:
        diagnostic = {key: report.get(key) for key in ("status", "stage", "error", "accuracy")}
        diagnostic["exit_code"] = process.returncode
        crash = report_path.with_name(report_path.name + ".native-crash.json")
        if crash.is_file():
            diagnostic["native_crash"] = json.loads(crash.read_text(encoding="utf-8"))
        raise RuntimeError(json.dumps(diagnostic, separators=(",", ":")))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=10)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--exact", action="store_true", help="Use noiseless box/hole samples")
    source.add_argument("--case", type=Path, help="Replay a saved validation case's metric cloud")
    parser.add_argument("--seed", type=int, default=4500)
    parser.add_argument("--seed-sweep", action="store_true", help="Increment seed each run; assess accuracy across seeds")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cad-validator-stability.json")
    args = parser.parse_args()
    if not 1 <= args.runs <= 100:
        parser.error("--runs must be between 1 and 100")
    if not 0 <= args.seed <= 0xFFFFFFFF - (args.runs - 1 if args.seed_sweep else 0):
        parser.error("seeds must be unsigned 32-bit integers")
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=args.output.stem + "-runs-", dir=args.output.parent))
    results, geometries = [], []
    fixture = CadValidationTests()
    for index in range(args.runs):
        seed = args.seed + (index if args.seed_sweep else 0)
        directory = work / f"run-{index + 1:03d}"
        directory.mkdir()
        item = {"run": index + 1, "seed": seed, "passed": False}
        try:
            report = (replay_case(args.case.resolve(), directory, seed) if args.case else
                      fixture.validate_cloud(exact=args.exact, seed=seed))
            (directory / "fitting_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            geometry = geometry_parameters(report)
            geometries.append(geometry)
            item.update(passed=True, geometry=geometry,
                        fingerprint=hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest())
        except (AssertionError, OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
            item["diagnostic"] = str(error)[:1800]
        results.append(item)
    variants = len({item["fingerprint"] for item in results if item["passed"]})
    passed = sum(item["passed"] for item in results)
    summary = {"schema": "paramcad.validator-stability", "runs": args.runs,
               "input": str(args.case.resolve()) if args.case else ("exact_noiseless" if args.exact else "seeded_noise"),
               "seed": args.seed, "seed_sweep": args.seed_sweep, "passed": passed,
               "geometry_variants": variants, "repeatable": variants == 1 and passed == args.runs,
               "parameter_spread": parameter_spread(geometries),
               "reports": str(work), "results": results}
    summary["success"] = passed == args.runs and (args.seed_sweep or summary["repeatable"])
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    diagnostic = {key: value for key, value in summary.items() if key != "results"}
    diagnostic["report"] = str(args.output)
    failures = [item for item in results if not item["passed"]]
    if failures:
        diagnostic["failures"] = failures[:3]
    print(json.dumps(diagnostic, separators=(",", ":")))
    return 0 if summary["success"] else 2


if __name__ == "__main__":
    sys.exit(main())
