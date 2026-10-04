#!/usr/bin/env python3
"""Replay CAD fitting from a saved validation case, without capture or COLMAP.

Preserves the initial reports and uses the original input snapshots/metric cloud.
Prints only the maintained structured diagnostic, never engine logs.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from diagnose_performance_validation import diagnose
from run_performance_validation import OUTPUTS_ROOT, read_json, write_json, hidden_process_options, write_readme, directory_size


def local_artifact(case: Path, relative: str) -> Path:
    path = (case / relative).resolve()
    path.relative_to(case)
    if not path.is_file():
        raise ValueError(f"Missing saved artifact: {path}")
    return path


def refit(case: Path, executable: Path | None = None) -> dict:
    case = case.resolve()
    case.relative_to(OUTPUTS_ROOT.resolve())
    report_path = case / "validation_report.json"
    report = read_json(report_path)
    cloud = local_artifact(case, report["artifacts"]["point_cloud"])
    model = local_artifact(case, report["inputs"]["model"]["snapshot"])
    expected = report["inputs"].get("expected")
    engine = (executable or Path(report["executables"]["fit"])).resolve()
    if not engine.is_file():
        raise ValueError(f"Validation executable missing: {engine}")
    for path in (report_path, case / "fitting_report.json"):
        initial = path.with_name(path.stem + ".initial.json")
        if not initial.exists():
            shutil.copy2(path, initial)
    command = [str(engine), "fit", str(cloud), str(case / "fitting_report.json"), str(case / "reconstructed.step")]
    if expected:
        command.extend([str(model), str(local_artifact(case, expected["snapshot"]))])
    step = case / "reconstructed.step"
    if step.is_file():
        # Preserve the previous export; a failed replay must not expose it as
        # the current run's reconstructed STEP.
        step.rename(case / f"reconstructed-before-refit-{time.time_ns()}.step")
    fitting_path = case / "fitting_report.json"
    if fitting_path.is_file():
        fitting_path.rename(case / f"fitting-before-refit-{time.time_ns()}.json")
    write_json(case / "pipeline_status.json", {"status": "running", "stage": "fitting"})
    started = time.monotonic()
    with (case / "refitting.log").open("w", encoding="utf-8") as stream:
        process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, **hidden_process_options())
    exit_code = process.returncode
    if fitting_path.is_file():
        fit = read_json(fitting_path)
    else:
        exit_code = exit_code or 2
        fit = {"schema": "paramcad.cad-validation", "version": 1, "status": "failed",
               "complete_cad_fit": False, "error": f"Validator exited with code {process.returncode} without writing a report"}
    fit["exit_code"] = exit_code
    if exit_code:
        fit["status"] = "failed"
        fit["complete_cad_fit"] = False
        fit.setdefault("error", f"Validator exited with code {exit_code} at stage {fit.get('stage', 'unknown')}")
    fit["process_elapsed_seconds"] = time.monotonic()-started
    fit["replayed_at"] = datetime.now(timezone.utc).isoformat()
    fit["log"] = "refitting.log"
    write_json(case / "fitting_report.json", fit)
    report["fitting"] = fit
    report["last_refit_at"] = fit["replayed_at"]
    report["executables"]["fit"] = str(engine)
    report["status"] = "failed" if exit_code else "completed"
    report["artifacts"]["fitting_log"] = "refitting.log"
    report["artifacts"]["initial_report"] = "validation_report.initial.json"
    if not exit_code:
        texture = local_artifact(case, report["inputs"]["texture"]["snapshot"])
        write_readme(case, report["capture"], report["colmap"], fit, model, texture)
    report["case_bytes"] = directory_size(case)
    write_json(report_path, report)
    write_json(case / "pipeline_status.json", {"status": report["status"], "stage": "fitting" if exit_code else "reporting",
                                             "cad_status": fit["status"], "cad_passed": fit.get("complete_cad_fit", False)})
    return diagnose(case)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", nargs="+", type=Path)
    parser.add_argument("--executable", type=Path)
    args = parser.parse_args()
    exit_code = 0
    try:
        for case in args.case:
            diagnostic = refit(case, args.executable)
            print(json.dumps(diagnostic, ensure_ascii=False, separators=(",", ":")))
            if diagnostic["pipeline"]["status"] == "failed":
                exit_code = 2
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(json.dumps({"status": "refit_failed", "error": str(error)}), file=sys.stderr)
        return 2
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
