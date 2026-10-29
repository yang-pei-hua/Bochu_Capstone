#!/usr/bin/env python3
"""Reconstruct editable CAD from PLY observations, without source models/labels."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
_ISOLATED_ENGINE = ROOT / "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/cad_reconstruction_validation.exe"
DEFAULT_ENGINE = _ISOLATED_ENGINE if _ISOLATED_ENGINE.is_file() else ROOT / "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def reconstruct(engine: Path, cloud: Path, directory: Path, timeout: float) -> dict:
    report = directory / "report.json"
    command = [str(engine), "reconstruct-cloud", str(cloud), str(report), str(directory / "model.step")]
    started = time.monotonic()
    timed_out = False
    with (directory / "reconstruction.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        while process.poll() is None:
            remaining = timeout - (time.monotonic() - started)
            if remaining <= 0:
                timed_out = True
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                break
            try:
                process.wait(timeout=min(30, remaining))
            except subprocess.TimeoutExpired:
                try:
                    stage = read_json(report).get("stage", "starting")
                except (OSError, ValueError):
                    stage = "starting"
                print(f"{directory.name}: {stage}, {time.monotonic()-started:.0f}s (PID {process.pid})", flush=True)
    try:
        result = read_json(report)
    except (OSError, ValueError):
        result = {"status": "failed", "error": "Worker exited without a readable report"}
    if timed_out:
        result.update({"status": "timeout", "cad_constructed": False, "error": f"Exceeded {timeout:g} second time limit"})
        write_json(report, result)
    exported = process.returncode == 0 and result.get("status") in {"cad_exported", "cad_exported_partial"} \
        and result.get("cad_constructed") is True \
        and result.get("project_round_trip_verified") is True \
        and result.get("step_round_trip_verified") is True
    artifacts = {}
    if exported:
        for key in ("project", "step", "features_file"):
            artifact = Path(result[key])
            if not artifact.is_file():
                raise ValueError(f"Missing exported artifact: {artifact}")
            artifacts[key] = {"path": str(artifact), "sha256": sha256(artifact)}
    return {"status": result.get("status", "failed"), "cad_constructed": bool(exported),
        "source_completeness_verified": False, "exit_code": process.returncode,
        "elapsed_seconds": time.monotonic() - started, "command": command, "report": str(report),
        "error": result.get("error"), "solids": result.get("solids", 0), "features": result.get("features", 0),
        "surface_coverage": result.get("surface_coverage"), "coverage_basis": result.get("coverage_basis"),
        "assigned_observation_fraction": result.get("assigned_observation_fraction"), "assembly": result.get("assembly"),
        "partial_reconstruction": result.get("partial_reconstruction", False), "artifacts": artifacts}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", action="append", required=True, metavar="NAME=FILE.ply")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--unit", choices=("mm", "m"), help="Declare scan units when no unit sidecar is available")
    parser.add_argument("--executable", type=Path, default=DEFAULT_ENGINE)
    parser.add_argument("--timeout", type=float, default=300, help="Time limit for each scan in seconds")
    args = parser.parse_args()
    try:
        output, engine = args.output.resolve(), args.executable.resolve()
        output.relative_to((ROOT / "outputs").resolve())
        if not engine.is_file() or not 0 < args.timeout < float("inf"):
            raise ValueError("Executable must exist and timeout must be finite and positive")
        cases = {}
        for argument in args.input:
            name, path = argument.split("=", 1)
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name) or name in cases:
                raise ValueError(f"Invalid or repeated case name: {name}")
            cloud = Path(path).resolve()
            if cloud.suffix.lower() != ".ply" or not cloud.is_file():
                raise ValueError(f"Expected an existing PLY: {cloud}")
            cases[name] = cloud
        if output.exists() and any(output.iterdir()):
            raise ValueError("Output directory must be new or empty")
        output.mkdir(parents=True, exist_ok=True)
        manifest = {"schema": "paramcad.point-cloud-only-cad", "version": 1, "status": "running",
            "source_model_read": False, "topology_correspondence_used": False, "reference_labels_used": False,
            "source_completeness_verified": False, "executable_sha256": sha256(engine), "cases": []}
        write_json(output / "summary.json", manifest)
        for name, source in cases.items():
            directory = output / name
            (directory / "input").mkdir(parents=True)
            row = {"name": name, "source_cloud": str(source), "source_cloud_sha256": sha256(source)}
            manifest["cases"].append(row)
            try:
                unit = args.unit
                if unit is None and source.with_suffix(".json").is_file():
                    unit = read_json(source.with_suffix(".json")).get("unit")
                if unit not in ("mm", "m"):
                    raise ValueError("Unknown metric unit: provide --unit mm or --unit m")
                # Copy observations and a unit-only sidecar. No other adjacent
                # files are copied, interpreted or supplied to the worker.
                cloud = directory / "input/cloud.ply"
                shutil.copy2(source, cloud)
                write_json(cloud.with_suffix(".json"), {"unit": unit})
                if sha256(cloud) != row["source_cloud_sha256"]:
                    raise ValueError("Observation copy checksum mismatch")
                row.update({"cloud": str(cloud), "unit": unit})
                write_json(output / "summary.json", manifest)
                print(f"{name}: reconstructing from observations only", flush=True)
                row.update(reconstruct(engine, cloud, directory, args.timeout))
            except (OSError, ValueError, KeyError) as error:
                row.update({"status": "failed", "cad_constructed": False, "error": str(error)})
            print(f"{name}: {row['status']} {row.get('error') or ''}", flush=True)
            write_json(output / "summary.json", manifest)
        exported = all(row["cad_constructed"] for row in manifest["cases"])
        partial = any(row.get("partial_reconstruction", False) for row in manifest["cases"])
        manifest["partial_reconstruction"] = partial
        manifest["status"] = ("cad_exported_partial" if partial else "cad_exported") if exported else "incomplete"
        write_json(output / "summary.json", manifest)
        lines = ["# Point-cloud-only CAD reconstruction", "", "Only PLY observations and declared units are reconstruction inputs.",
            "Export success does not prove recovery of every source part; no source topology is available.", "",
            "| Scan | Result | Exported solids | Features |", "| --- | --- | ---: | ---: |"]
        for row in manifest["cases"]:
            lines.append(f"| {row['name']} | {row['status']} | {row.get('solids', 0)} | {row.get('features', 0)} |")
        for row in manifest["cases"]:
            if row.get("error"):
                lines += ["", f"{row['name']}: {row['error']}"]
        (output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        return 0 if exported else 1
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
