#!/usr/bin/env python3
"""Sample a STEP assembly and verify every face through core and Studio analysis.

Reference face/solid membership is a separate evaluation oracle. Neither detector
receives STEP topology or labels. This checks surface recognition, not CAD export.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_ENGINE = ROOT / "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe"


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def add_noise(source: Path, target: Path, sigma: float, seed: int) -> None:
    randomizer = random.Random(seed)
    with source.open(encoding="ascii") as original, target.open("w", encoding="ascii") as noisy:
        for line in original:
            noisy.write(line)
            if line.strip() == "end_header":
                break
        for line in original:
            values = list(map(float, line.split()))
            for axis in range(3):
                values[axis] += randomizer.gauss(0, sigma)
            noisy.write(" ".join(format(value, ".17g") for value in values) + "\n")
    write_json(target.with_suffix(".json"), {"unit": "mm", "source": "noisy_step_samples", "sigma_mm": sigma})


def run_stage(engine: Path, output: Path, mode: str, arguments: list[str]) -> dict:
    started = time.monotonic()
    print(f"{mode}: running", flush=True)
    with (output / f"{mode}.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen([str(engine), mode, *arguments], stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        while process.poll() is None:
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print(f"{mode}: {time.monotonic() - started:.0f}s elapsed", flush=True)
    if process.returncode:
        raise RuntimeError(f"{mode} exited with {process.returncode}; see {output / (mode + '.log')}")
    # All supported modes place report.json in the second positional argument.
    report = json.loads(Path(arguments[1]).read_text(encoding="utf-8"))
    print(f"{mode}: {report['status']} ({time.monotonic() - started:.1f}s)", flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--executable", type=Path, default=DEFAULT_ENGINE)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--noise", type=float, default=0.0, help="Seeded position noise sigma in mm")
    parser.add_argument("--seed", type=int, default=4500)
    parser.add_argument("--skip-client", action="store_true")
    parser.add_argument("--replay", action="store_true", help="Reuse saved samples and archive previous reports")
    args = parser.parse_args()
    try:
        for name in ("spacing", "tolerance", "noise"):
            value = getattr(args, name)
            if not math.isfinite(value) or value < 0 or (name != "noise" and value == 0):
                raise ValueError(f"Invalid {name}")
        if not 0 <= args.seed <= 2**32 - 1:
            raise ValueError("Seed must be an unsigned 32-bit integer")
        model, engine, output = args.model.resolve(), args.executable.resolve(), args.output.resolve()
        if not model.is_file() or not engine.is_file():
            raise ValueError("STEP model and validation executable must exist")
        if model.suffix.lower() not in {".step", ".stp"}:
            raise ValueError("Expected a STEP/STP model")
        output.relative_to((ROOT / "outputs").resolve())
        if not args.replay and output.exists() and any(output.iterdir()):
            raise ValueError("Output directory must be new or empty")
        output.mkdir(parents=True, exist_ok=True)
        cloud, reference = output / "cloud.ply", output / "reference.json"
        if args.replay:
            previous = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            if previous["source_sha256"] != sha256(model) or previous["noise_sigma_mm"] != args.noise or previous["ransac_seed"] != args.seed:
                raise ValueError("Replay source, noise and seed must match the saved case")
            cloud = output / ("cloud-noisy.ply" if args.noise else "cloud.ply")
            if previous["cloud_sha256"] != sha256(cloud):
                raise ValueError("Saved replay cloud has changed")
            sampled = json.loads(reference.read_text(encoding="utf-8"))
            clean_hash = previous["clean_cloud_sha256"]
            archive = output / "history" / str(time.time_ns())
            archive.mkdir(parents=True)
            for name in ("summary.json", "README.md", "analyze.json", "analyze.log", "analyze.segmented.ply", "analyze.segmented.json",
                         "analyze-client.json", "analyze-client.log", "analyze-client.segmented.ply", "analyze-client.segmented.json"):
                source = output / name
                if source.is_file():
                    source.rename(archive / name)
        else:
            sampled = run_stage(engine, output, "sample", [str(model), str(reference), str(cloud), str(args.spacing)])
            clean_hash = sha256(cloud)
            if args.noise:
                noisy = output / "cloud-noisy.ply"
                add_noise(cloud, noisy, args.noise, args.seed)
                cloud = noisy
        os.environ["PCAD_RANSAC_SEED"] = str(args.seed)
        reports = {}
        for mode in (["analyze"] if args.skip_client else ["analyze", "analyze-client"]):
            reports[mode] = run_stage(engine, output, mode,
                [str(cloud), str(output / f"{mode}.json"), str(reference), str(args.tolerance)])
        passed = all(report.get("complete_surface_recognition", False) for report in reports.values())
        summary = {
            "schema": "paramcad.step-surface-validation", "version": 1,
            "status": "passed" if passed else "failed", "complete_cad_fit": False,
            "reference_labels_used_by_recognition": False,
            "source_step": str(model), "source_sha256": sha256(model),
            "cloud": str(cloud), "clean_cloud_sha256": clean_hash, "cloud_sha256": sha256(cloud),
            "unit": "mm", "noise_sigma_mm": args.noise, "ransac_seed": args.seed,
            "required_per_face_coverage": 0.98, "tolerance_mm": args.tolerance,
            "points": sampled["points"], "solid_instances": sampled["solid_count"], "faces": sampled["face_count"],
            "results": {mode: {key: report[key] for key in
                ("status", "recognized_faces", "total_faces", "recognized_solids", "total_solids", "point_coverage", "elapsed_seconds", "segmented_cloud")}
                for mode, report in reports.items()},
        }
        write_json(output / "summary.json", summary)
        rows = [f"| {mode} | {r['recognized_faces']}/{r['total_faces']} | {r['recognized_solids']}/{r['total_solids']} | {r['point_coverage']:.4%} |"
                for mode, r in reports.items()]
        (output / "README.md").write_text(
            "# STEP surface validation\n\n"
            f"Source: `{model}`. Cloud: `{cloud.name}`, {sampled['points']:,} points in mm.\n\n"
            "| Path | Recognized faces | Solids with all faces recognized | Point coverage |\n"
            "| --- | --- | --- | --- |\n" + "\n".join(rows) + "\n\n"
            "Each face requires at least 98% correctly typed support and displayed point labels. "
            "The reference topology is read only after detection. Solid counts summarize reference "
            "face completeness; they do not assert independent assembly-instance segmentation.\n\n"
            "This validates analytic surfaces and segmentation. It does not certify a closed, "
            "editable CAD model or a reconstructed STEP.\n", encoding="utf-8")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if passed else 1
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
