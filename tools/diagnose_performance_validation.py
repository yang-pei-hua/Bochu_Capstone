#!/usr/bin/env python3
"""Print compact validation diagnostics. Logs are opt-in, bounded, and never scanned by default."""
from __future__ import annotations
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import subprocess


def read(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"diagnostic_error": str(error)}


def diagnose(case: Path, log_tail: int = 0) -> dict:
    status = read(case / "pipeline_status.json")
    result: dict = {"case": case.name, "pipeline": status}
    capture = read(case / "capture_report.json")
    if capture:
        result["capture"] = {key: capture.get(key) for key in ("imageCount", "width", "height", "topology", "timing")}
    progress = read(case / "workspace" / "progress.json")
    if progress:
        current = progress.get("current", {})
        result["colmap_stage"] = {key: current.get(key) for key in ("command", "status", "exit_code", "elapsed_seconds")}
        if current.get("status") == "running" and current.get("started_at"):
            from datetime import datetime, timezone
            result["colmap_stage"]["elapsed_seconds"] = round((datetime.now(timezone.utc) - datetime.fromisoformat(current["started_at"])).total_seconds(), 1)
        result["completed_colmap_stages"] = [s["command"] for s in progress.get("stages", []) if s.get("status") == "completed"]
        depth = case / "workspace" / "dense" / "stereo" / "depth_maps"
        if depth.is_dir():
            result["depth_maps"] = {
                "photometric": sum(1 for _ in depth.glob("*.photometric.bin")),
                "geometric": sum(1 for _ in depth.glob("*.geometric.bin")),
                "expected_each": capture.get("imageCount"),
            }
    reconstruction = read(case / "workspace" / "reconstruction.json")
    if reconstruction:
        result["point_cloud"] = {key: reconstruction.get(key) for key in ("point_count", "density", "unit", "camera_mode")}
    fit = read(case / "fitting_report.json")
    if fit:
        result["fitting"] = {key: fit.get(key) for key in ("status", "stage", "exit_code", "ransac_seed", "complete_cad_fit", "recognition", "compound_recognition", "preprocessing", "processed_bounds_mm", "detection_options", "primitive_proposals", "box_recognition", "through_holes", "accuracy", "elapsed_seconds", "error") if key in fit}
        result["step_exported"] = (case / "reconstructed.step").is_file()
    if fit.get("curved_solids"):
        result["fitting"]["curved_solids"] = fit["curved_solids"]
    crash = read(case / "fitting_report.json.native-crash.json")
    if crash and fit.get("complete_cad_fit") is not True:
        result["native_crash"] = {"exception_code": crash.get("exception_code"),
                                  "stack": crash.get("stack", [])[:6]}
    failure = read(case / "pipeline_failure.json")
    if failure:
        result["failure"] = failure
    # Only an explicit --log-tail request exposes a bounded excerpt. Detailed
    # per-primitive evidence remains in fitting_report.json, not engine logs.
    if log_tail:
        stage = status.get("stage", "")
        log = case / ({"capture": "capture.log", "reconstruction": "workspace/colmap.log", "fitting": "fitting.log"}.get(stage, "pipeline.log"))
        if log.is_file():
            with log.open(encoding="utf-8", errors="replace") as stream:
                result["log_excerpt"] = {"path": str(log), "lines": [line.rstrip()[:1000] for line in deque(stream, maxlen=log_tail)]}
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", nargs="+", type=Path)
    parser.add_argument("--log-tail", type=int, default=0, help="Opt-in last N lines (maximum 30)")
    parser.add_argument("--brief", action="store_true", help="Only status, depth-map counts, and final CAD outcome")
    parser.add_argument("--inspect-sparse", action="store_true", help="Analyze the finished sparse model and database, without reading logs")
    parser.add_argument("--evidence", action="store_true", help="Include compact per-surface fitting evidence")
    args = parser.parse_args()
    if not 0 <= args.log_tail <= 30:
        parser.error("--log-tail must be between 0 and 30")
    for case in args.case:
        if not case.is_dir():
            parser.error(f"case directory does not exist: {case}")
        result = diagnose(case.resolve(), args.log_tail)
        if args.inspect_sparse:
            from run_performance_validation import database_statistics, find_colmap, find_sparse_model, parse_model_analyzer, PipelineError, hidden_process_options
            try:
                workspace = case.resolve() / "workspace"
                model = find_sparse_model(workspace)
                analysis = subprocess.run([str(find_colmap(None)), "model_analyzer", "--path", str(model)],
                                          capture_output=True, text=True, encoding="utf-8", errors="replace", **hidden_process_options())
                if analysis.returncode:
                    raise PipelineError(f"model_analyzer exited with {analysis.returncode}")
                result["sparse_health"] = {**database_statistics(workspace / "database.db"),
                                           **parse_model_analyzer(analysis.stdout + analysis.stderr)}
            except (PipelineError, OSError) as error:
                result["sparse_health"] = {"error": str(error)}
        if args.evidence:
            result["evidence_summary"] = read(case / "fitting_report.json").get("evidence_summary", [])
        if args.brief:
            fit = result.get("fitting", {})
            requested_details = {key: result[key] for key in ("log_excerpt", "sparse_health", "evidence_summary", "native_crash") if key in result}
            stage = result.get("colmap_stage", {})
            depth = result.get("depth_maps", {})
            brief = {"case": case.name, "status": result["pipeline"].get("status"),
                     "stage": stage.get("command") if stage.get("status") == "running" else result["pipeline"].get("stage")}
            if depth:
                brief["depth_maps"] = f"photo={depth['photometric']}, geometric={depth['geometric']}, expected_each={depth['expected_each']}"
            if fit:
                brief.update(cad_status=fit.get("status"), cad_passed=fit.get("complete_cad_fit"))
            if result.get("failure"):
                brief["failure"] = result["failure"]
            result = {**brief, **requested_details}
        print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
