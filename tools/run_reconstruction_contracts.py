#!/usr/bin/env python3
"""Run point-cloud module contracts from reference artifacts or a shared PLY."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys
import time

from reconstruction_contract_fixtures import materialize
from run_point_cloud_cad import DEFAULT_ENGINE, ROOT, read_json, sha256, write_json

DEFAULT_MANIFEST = ROOT / "data/reconstruction-contracts/v1/manifest.json"


def component_name(value):
    return isinstance(value, str) and value and value not in (".", "..") and all(
        c.isascii() and (c.isalnum() or c in "_.-") for c in value)


def verify(case, metrics, artifact, truth):
    for metric, expected in case["metrics"].items():
        actual = metrics.get(metric)
        if isinstance(expected, dict):
            good = (type(actual) in (int, float) and math.isfinite(actual)
                    and expected.get("min", -math.inf) <= actual <= expected.get("max", math.inf))
        else:
            good = actual == expected and isinstance(actual, bool) == isinstance(expected, bool)
        if not good:
            return f"{metric}: expected {expected!r}, got {actual!r}"
    for field, checks in case.get("point_ids", {}).items():
        actual = set(artifact[field])
        if not set(checks.get("required", ())) <= actual or actual & set(checks.get("excluded", ())):
            return f"{field}: required or excluded point IDs disagree"
    for field, membership in case.get("partitions", {}).items():
        actual = {frozenset(group[membership]) for group in artifact[field]}
        expected = {frozenset(group) for group in truth[field]}
        if membership == "working_ids":
            working = {row[0] for row in artifact["work"]}
            expected = {group & working for group in expected if group & working}
        if membership == "source_ids" and case.get("allow_unassigned_source"):
            omitted = artifact["unassigned_source_ids"]
            omitted_set = set(omitted)
            universe = set().union(*expected)
            if len(omitted) != len(omitted_set) or not omitted_set <= universe:
                return "Unassigned source IDs contain duplicates or foreign points"
            expected = {group - omitted_set for group in expected if group - omitted_set}
        if actual != expected or len(actual) != len(artifact[field]):
            return f"{field}.{membership}: point membership differs from the reference partition"
    return None


class Run:
    def __init__(self, manifest, output, engine, timeout, cloud_override):
        self.manifest = manifest
        self.output = output
        self.engine = engine
        self.timeout = timeout
        self.cloud_override = cloud_override
        self.datasets = {}
        self.live = {}
        self.backfill = {}
        self.executions = []
        self.backfill_cases = {}
        grouped = {}
        for case in manifest["cases"]:
            if case.get("phase"):
                name = case["dataset"]
                if case["phase"] != "backfill" or case["module"] != 19 or name in self.backfill_cases:
                    raise ValueError(f"Invalid backfill case: {case['id']}")
                self.backfill_cases[name] = case
            else:
                grouped.setdefault((case["dataset"], case["module"]), []).append(case)
        self.stage_cases = {}
        for key, cases in grouped.items():
            gates = cases if len(cases) == 1 else [case for case in cases if case.get("upstream_gate") is True]
            if len(gates) != 1:
                raise ValueError(f"{key} needs exactly one upstream_gate when it has multiple cases")
            self.stage_cases[key] = gates[0]

    def dataset(self, name):
        if name not in self.datasets:
            if not component_name(name):
                raise ValueError(f"Invalid dataset ID: {name!r}")
            folder = self.output / "datasets" / name
            folder.mkdir(parents=True)
            spec = self.manifest["datasets"][name]
            self.datasets[name] = materialize(spec, folder, self.cloud_override)
        return self.datasets[name]

    def execute(self, case, mode, upstream):
        name, module = case["dataset"], case["module"]
        phase = case.get("phase")
        cloud, cloud_hash, _, truth = self.dataset(name)
        spec = self.manifest["datasets"][name]
        tag = case["id"] if mode == "fixture" else f"{module}-backfill" if phase else str(module)
        folder = self.output / mode / name / tag
        folder.mkdir(parents=True)
        output = folder / "artifact.json"
        report = folder / "report.json"
        request = folder / "request.json"
        write_json(request, {"module": module, "cloud": str(cloud), "unit": spec["unit"],
                             "options": spec["options"], "input": str(upstream) if upstream else None,
                             "artifact": str(output), "phase": phase})
        command = [str(self.engine), "contract-stage", str(request), str(report)]
        started = time.monotonic()
        row = {"dataset": name, "module": module, "mode": mode, "status": "failed",
               "cloud_sha256": cloud_hash, "report": str(report), "request": str(request)}
        if phase:
            row["phase"] = phase
        self.executions.append(row)
        try:
            if upstream:
                row["upstream_sha256"] = sha256(upstream)
            with (folder / "engine.log").open("wb") as log:
                process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                         timeout=self.timeout, check=False,
                                         creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0)
            if not report.is_file():
                raise ValueError(f"Stage returned no report (exit code {process.returncode})")
            result = read_json(report)
            if process.returncode or result.get("status") != "contract_stage_completed":
                raise ValueError(result.get("error") or f"Stage failed (exit code {process.returncode})")
            row["artifact"] = str(output)
            row["artifact_sha256"] = sha256(output)
            row["metrics"] = result["metrics"]
            issue = verify(case, row["metrics"], read_json(output), truth)
            if issue:
                raise ValueError(issue)
            row["status"] = "passed"
        except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
            row["error"] = str(error)
        row["elapsed_seconds"] = time.monotonic() - started
        return row

    def upstream(self, module):
        predecessor = self.manifest["stages"][str(module)]
        if predecessor is not None and (type(predecessor) is not int or predecessor >= module):
            raise ValueError(f"Invalid upstream stage for module {module}")
        return predecessor

    def from_cloud(self, case):
        name, module = case["dataset"], case["module"]
        key = name, module
        if key not in self.live:
            predecessor = self.upstream(module)
            upstream = None
            if predecessor is not None:
                dependency = self.from_cloud(self.stage_cases[name, predecessor])
                if dependency["status"] != "passed":
                    self.live[key] = self.blocked(case, dependency)
                    return self.live[key]
                upstream = Path(dependency["artifact"])
            bridge = self.manifest.get("bridges", {}).get(str(module))
            if bridge:
                if bridge != {"module": 19, "phase": "backfill"} or module != 21 or predecessor != 20:
                    raise ValueError(f"Unsupported bridge for module {module}")
                dependency = self.from_backfill(self.backfill_cases[name])
                if dependency["status"] != "passed":
                    self.live[key] = self.blocked(case, dependency)
                    return self.live[key]
                upstream = Path(dependency["artifact"])
            self.live[key] = self.execute(case, "cloud", upstream)
        return self.live[key]

    def from_backfill(self, case):
        name = case["dataset"]
        if name not in self.backfill:
            upstream = self.from_cloud(self.stage_cases[name, 20])
            if upstream["status"] == "passed":
                self.backfill[name] = self.execute(case, "cloud", Path(upstream["artifact"]))
            else:
                self.backfill[name] = self.blocked(case, upstream)
        return self.backfill[name]

    def cloud_case(self, case):
        if case.get("phase") == "backfill":
            return self.from_backfill(case)
        key = case["dataset"], case["module"]
        gate = self.stage_cases[key]
        row = self.from_cloud(gate)
        if gate is case or "artifact" not in row:
            return row
        _, _, _, truth = self.dataset(case["dataset"])
        issue = verify(case, row["metrics"], read_json(Path(row["artifact"])), truth)
        result = {k: v for k, v in row.items() if k != "error"}
        result["status"] = "failed" if issue else "passed"
        if issue:
            result["error"] = issue
        return result

    @staticmethod
    def blocked(case, dependency):
        return {"dataset": case["dataset"], "module": case["module"], "mode": "cloud",
                "status": "blocked_upstream",
                "blocked_by_module": dependency.get("blocked_by_module", dependency["module"]),
                "blocked_by_phase": dependency.get("blocked_by_phase", dependency.get("phase")),
                "error": dependency.get("error", "Upstream stage failed")}

    def from_fixture(self, case, override):
        if override is not None:
            return self.execute(case, "fixture", override)
        name = case["dataset"]
        fixture_name = case["fixture"]
        fixture = self.manifest["fixtures"][fixture_name]
        needs_backfill = case.get("phase") == "backfill"
        expected = 20 if needs_backfill else (self.upstream(case["module"]) or 0)
        if fixture["dataset"] != name or fixture["module"] != expected or not component_name(fixture_name):
            raise ValueError(f"Mismatched fixture {fixture_name!r}")
        _, _, references, _ = self.dataset(name)
        upstream = self.output / "datasets" / name / f"{fixture_name}.json"
        if not upstream.exists():
            key = fixture.get("reference", "20-boundary" if needs_backfill else expected)
            reference = references[key]
            if reference["module"] != expected:
                raise ValueError(f"Reference type disagrees with fixture {fixture_name!r}")
            write_json(upstream, reference)
        return self.execute(case, "fixture", upstream)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--executable", type=Path, default=DEFAULT_ENGINE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", action="append", help="Select case IDs. Repeat to select several.")
    parser.add_argument("--module", action="append", type=int, help="Select module numbers.")
    parser.add_argument("--mode", choices=("both", "fixture", "cloud"), default="both")
    parser.add_argument("--upstream", type=Path, help="Use a saved upstream JSON artifact with one fixture case.")
    parser.add_argument("--cloud", type=Path, help="Use an explicit PLY backing file for one selected dataset.")
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args(argv)

    manifest = read_json(args.manifest)
    if manifest.get("schema") != "paramcad.reconstruction-contracts" or manifest.get("version") != 1:
        parser.error("Unsupported manifest version")
    cases = manifest["cases"]
    if (any(not component_name(case["id"]) for case in cases)
            or len({case["id"] for case in cases}) != len(cases)):
        parser.error("Invalid or duplicate case IDs")
    selected = [case for case in cases if (not args.case or case["id"] in args.case)
                and (not args.module or case["module"] in args.module)]
    if (not selected or set(args.case or ()) - {case["id"] for case in selected}
            or set(args.module or ()) - {case["module"] for case in selected}):
        parser.error("Case selection is empty or includes unknown/excluded IDs")
    if args.upstream and (len(selected) != 1 or args.mode != "fixture" or not args.upstream.is_file()):
        parser.error("--upstream requires one fixture-mode case and an existing JSON file")
    if args.cloud and (len({case["dataset"] for case in selected}) != 1 or not args.cloud.is_file()
                       or (args.mode != "cloud" and args.upstream is None)):
        parser.error("--cloud requires one dataset and either cloud mode or an explicit --upstream artifact")
    engine = args.executable.resolve()
    if not engine.is_file() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("An existing executable and positive timeout are required")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    run = Run(manifest, output, engine, args.timeout, args.cloud.resolve() if args.cloud else None)
    modes = ("fixture", "cloud") if args.mode == "both" else (args.mode,)
    results = []
    for case in selected:
        for mode in modes:
            row = run.cloud_case(case) if mode == "cloud" else run.from_fixture(
                case, args.upstream.resolve() if args.upstream else None)
            results.append({"case": case["id"], **row})
            print(f"{case['id']} [{mode}]: {row['status']}", flush=True)
    summary = {"schema": "paramcad.reconstruction-contract-run", "version": 1,
               "manifest_sha256": sha256(args.manifest), "executable": str(engine),
               "executable_sha256": sha256(engine),
               "fixture_generator_sha256": sha256(ROOT / "tools/reconstruction_contract_fixtures.py"),
               "results": results, "executions": run.executions}
    write_json(output / "summary.json", summary)
    return int(any(row["status"] != "passed" for row in results))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError) as error:
        sys.exit(f"Contract run: {error}")
