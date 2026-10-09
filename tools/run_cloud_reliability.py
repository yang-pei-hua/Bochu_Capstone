"""Replay independently sampled stress clouds or an explicitly sourced dataset."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time

from generate_compound_test_clouds import ENGINE, ROOT, box, cases, mass_properties, sample_surface
from generate_curved_test_clouds import definition, samples
from run_performance_validation import hidden_process_options


CONDITIONS = ("clean", "noise", "outliers", "missing-normals", "no-normals", "nonuniform", "incomplete")


def box_definition():
    return {"id": "box", "unit": "mm", "geometry": "box_union",
            "parameters": {"boxes": [box((-30, -20, -10), (60, 40, 20))]},
            "expected": {"bounding_box_mm": [60, 40, 20], "volume_mm3": 48000,
                         "solids": 1, "through_holes": 0, "hole_diameters_mm": [], "boxes": 1, "prisms": 0},
            "acceptance": {"dimension_tolerance_mm": .5, "hole_diameter_tolerance_mm": .5,
                           "symmetric_difference_volume_fraction": .05}}


def stress_cloud(kind, condition, seed=4500):
    """No fitted geometry or ground truth is used to repair the observations."""
    rng = random.Random(seed)
    if kind in ("l-bracket-gusset", "box-through-hole"):
        case = json.loads((ROOT/"data/performance-validation"/kind/"case.json").read_text(encoding="utf-8"))
        if kind == "l-bracket-gusset":
            _, boxes, rib, *_ = next(c for c in cases() if c[0] == kind)
            _, center = mass_properties(boxes, rib)
            points = [[*(p[a]-center[a] for a in range(3)), *p[3:]] for p in sample_surface(boxes, rib)]
        else:
            parameters = case["parameters"]
            x, y, z, r = (parameters[name] for name in ("size_x", "size_y", "size_z", "hole_diameter"))
            r /= 2
            points = [list(p) for p in sample_surface([box((-x/2, -y/2, -z/2), (x, y, z))])
                      if not (abs(p[5]) == 1 and p[0]**2+p[1]**2 < r*r)]
            for azimuth in range(128):
                angle = 2*math.pi*azimuth/128
                c, s = math.cos(angle), math.sin(angle)
                for ring in range(int(z)+1):
                    points.append([r*c, r*s, ring-z/2, -c, -s, 0])
        if condition == "incomplete":
            points = [p for p in points if p[3] != -1]
    elif kind == "box":
        case = box_definition()
        points = [list(p) for p in sample_surface(case["parameters"]["boxes"])]
        if condition == "incomplete":
            points = [p for p in points if p[3] != -1]  # one entire face is unobserved
    else:
        case = definition(kind, rotated=True)
        points = samples(case, incomplete=condition == "incomplete", seed=seed)
    if condition == "noise":
        for p in points:
            for axis in range(3):
                p[axis] += rng.gauss(0, .02)
    if condition in ("missing-normals", "no-normals"):
        for p in points:
            if condition == "no-normals" or rng.random() < .8:
                p[3:] = [0, 0, 0]
    if condition == "nonuniform":
        # Reduce one half's density, while retaining observations of every face.
        points = [p for p in points if p[0] >= 0 or rng.random() < .3]
    if condition == "outliers":
        diagonal = math.sqrt(sum(x*x for x in case["expected"]["bounding_box_mm"]))
        for _ in range(max(1, len(points)//100)):
            points.append([rng.uniform(-100, 100)*diagonal for _ in range(3)] + [0, 0, 0])
    expectation = "reject" if condition == "incomplete" else (
        "accurate-or-reject" if condition == "no-normals" else "accurate")
    return case, points, expectation


def write_cloud(path, points, omit_normals=False):
    fields = ("x", "y", "z") if omit_normals else ("x", "y", "z", "nx", "ny", "nz")
    header = f"ply\nformat ascii 1.0\nelement vertex {len(points)}\n"
    header += "".join(f"property float {field}\n" for field in fields) + "end_header\n"
    path.write_text(header + "".join(" ".join(format(x, ".9g") for x in p[:len(fields)]) + "\n" for p in points), encoding="ascii")


def evaluate(report, exit_code, step_exists, expectation):
    accurate = report.get("status") == "cad_accuracy_passed" and report.get("complete_cad_fit") is True and step_exists
    rejected = exit_code == 0 and report.get("status") == "cad_not_recognized" and not step_exists
    passed = exit_code == 0 and (accurate if expectation == "accurate" else
                                rejected if expectation == "reject" else accurate or rejected)
    return {"passed": passed, "accurate": accurate, "safely_rejected": rejected,
            "unexpected_export": step_exists and not accurate}


def replay(directory, cloud, case, expectation, provenance, engine=ENGINE, seed=4500, reference=None):
    directory.mkdir(parents=True, exist_ok=True)
    # Each invocation has fresh outputs: interrupted runs cannot inherit accepted CAD.
    stamp = time.time_ns()
    for name in ("fitting_report.json", "reconstructed.step", "fitting_report.json.native-crash.json", "model.step", "model_report.json"):
        previous = directory/name
        if previous.exists():
            previous.rename(previous.with_name(previous.stem + f".previous-{stamp}" + previous.suffix))
    if cloud != directory/"cloud.ply":
        shutil.copyfile(cloud, directory/"cloud.ply")
    (directory/"case.json").write_text(json.dumps(case, indent=2)+"\n", encoding="utf-8")
    (directory/"reconstruction.json").write_text(json.dumps({"unit": case["unit"]})+"\n", encoding="utf-8")
    stages = []
    if reference:
        shutil.copyfile(reference, directory/"model.step")
    else:
        stages.append(("generate", ["generate", directory/"case.json", directory/"model.step", directory/"model_report.json"]))
    stages.append(("fit", ["fit", directory/"cloud.ply", directory/"fitting_report.json", directory/"reconstructed.step", directory/"model.step", directory/"case.json"]))
    start = time.monotonic()
    exit_code = 0
    for stage, arguments in stages:
        with (directory/(stage+".log")).open("wb") as log:
            try:
                exit_code = subprocess.run(list(map(str, [engine, *arguments])), stdout=log, stderr=subprocess.STDOUT,
                                           timeout=120, env={**os.environ, "PCAD_RANSAC_SEED": str(seed)},
                                           **hidden_process_options()).returncode
            except subprocess.TimeoutExpired:
                exit_code = "timeout"
        if exit_code != 0:
            break
    report_path = directory/"fitting_report.json"
    try:
        report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {}
    except (ValueError, OSError):
        report = {}
    result = {"case": directory.name, "provenance": provenance, "expectation": expectation,
              "cloud_sha256": hashlib.sha256((directory/"cloud.ply").read_bytes()).hexdigest(),
              "seed": seed, "elapsed_seconds": time.monotonic()-start, "exit_code": exit_code,
              "status": report.get("status"), "preprocessing": report.get("preprocessing"),
              "accuracy": report.get("accuracy"), "error": report.get("error") or report.get("recognition"),
              "report": str(report_path), **evaluate(report, exit_code, (directory/"reconstructed.step").is_file(), expectation)}
    (directory/"reliability_result.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    return result


def manifest_cases(path):
    manifest = json.loads(path.read_text(encoding="utf-8"))
    result = []
    for item in manifest["cases"]:
        if item["provenance"] not in ("real-capture", "virtual-capture") or not item.get("source"):
            raise ValueError("Recorded clouds require explicit provenance and source")
        if item.get("expectation", "accurate") not in ("accurate", "reject", "accurate-or-reject"):
            raise ValueError("Invalid expectation")
        if not item["id"] or Path(item["id"]).name != item["id"] or item["id"] in (".", ".."):
            raise ValueError("Case id must be a directory name")
        resolved = {**item, **{key: (path.parent/item[key]).resolve() for key in ("cloud", "definition", "reference")}}
        for key in ("cloud", "definition", "reference"):
            if not resolved[key].is_file():
                raise FileNotFoundError(resolved[key])
        case = json.loads(resolved["definition"].read_text(encoding="utf-8"))
        if case["unit"] not in ("mm", "m"):
            raise ValueError("Recorded clouds require explicit metric units")
        result.append((resolved, case))
    if not result:
        raise ValueError("Recorded dataset contains no cases")
    if len({item[0]["id"] for item in result}) != len(result):
        raise ValueError("Duplicate case ids")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/cloud-reliability")
    parser.add_argument("--engine", type=Path, default=ENGINE)
    parser.add_argument("--seed", type=int, default=4500)
    parser.add_argument("--kinds", nargs="+", choices=("box", "sphere", "cylinder", "cone", "torus", "frustum", "l-bracket-gusset", "box-through-hole"), default=["box", "sphere", "cylinder", "torus", "frustum", "l-bracket-gusset", "box-through-hole"])
    parser.add_argument("--conditions", nargs="+", choices=CONDITIONS, default=list(CONDITIONS))
    parser.add_argument("--manifest", type=Path, help="Replay measured clouds with declared provenance and independent CAD ground truth")
    args = parser.parse_args(argv)
    if not 0 <= args.seed <= 0xFFFFFFFF:
        parser.error("Invalid seed")
    results = []
    engine_hash = hashlib.sha256(args.engine.read_bytes()).hexdigest()
    # Validate the entire recorded manifest before launching or overwriting artifacts.
    recorded = manifest_cases(args.manifest) if args.manifest else []
    for item, case in recorded:
        result = replay(args.output/item["id"], item["cloud"], case, item.get("expectation", "accurate"),
                        {"kind": item["provenance"], "source": item["source"], "input": str(item["cloud"])},
                        args.engine, args.seed, item["reference"])
        results.append(result)
        print(json.dumps({key: result[key] for key in ("case", "passed", "status", "elapsed_seconds")}), flush=True)
    if not args.manifest:
        for kind in args.kinds:
            for condition in args.conditions:
                case, points, expectation = stress_cloud(kind, condition, args.seed)
                directory = args.output/f"{kind}-{condition}"
                directory.mkdir(parents=True, exist_ok=True)
                write_cloud(directory/"cloud.ply", points, condition == "no-normals")
                result = replay(directory, directory/"cloud.ply", case, expectation,
                                {"kind": "analytic-synthetic", "condition": condition,
                                 "noise_sigma_mm": .02 if condition == "noise" else 0}, args.engine, args.seed)
                results.append(result)
                print(json.dumps({key: result[key] for key in ("case", "passed", "status", "elapsed_seconds")}), flush=True)
    summary = {"schema": "paramcad.cloud-reliability.v1", "engine": str(args.engine), "engine_sha256": engine_hash,
               "seed": args.seed, "passed": sum(r["passed"] for r in results), "total": len(results),
               "accurate": sum(r["accurate"] for r in results), "safely_rejected": sum(r["safely_rejected"] for r in results),
               "unexpected_exports": sum(r["unexpected_export"] for r in results), "results": results}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "results"}))
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    sys.exit(main())
