#!/usr/bin/env python3
"""Measure boundary clutter and closure blockers; optionally run isolated assembly probes.

Requires numpy and matplotlib. Reference STEP geometry/labels are never read.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from run_point_cloud_cad import DEFAULT_ENGINE, sha256, write_json


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def observed_patch(patch):
    return {**patch, **patch.get("boundary_before_closure", {})}


def projected(loop, patch):
    coordinates = np.asarray(loop, dtype=float) - patch["origin_mm"]
    x = np.asarray(patch["x_direction"])
    y = np.cross(patch["normal"], x)
    return np.column_stack((coordinates @ x, coordinates @ y))


def signed_area(polygon):
    following = np.roll(polygon, -1, axis=0)
    return float(np.sum(polygon[:, 0]*following[:, 1] - following[:, 0]*polygon[:, 1])*.5)


def crosses_itself(polygon):
    """Proper crossings, excluding touching/collinear edges and adjacent edges."""
    def cross(a, b):
        return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]

    following = np.roll(polygon, -1, axis=0)
    for i in range(len(polygon)-2):
        end = len(polygon)-1 if i == 0 else len(polygon)
        a, b = polygon[i], following[i]
        c, d = polygon[i+2:end], following[i+2:end]
        ab, cd = b-a, d-c
        # Only strict opposite sides count; this is not a full polygon validity test.
        if np.any((cross(ab, c-a)*cross(ab, d-a) < -1e-16) &
                  (cross(cd, a-c)*cross(cd, b-c) < -1e-16)):
            return True
    return False


def metrics(directory: Path, input_points: int) -> dict:
    data = read(directory / "parts.json")
    patches = {p["id"]: observed_patch(p) for p in data["patches"]}
    parts = data["parts"]
    owner = {i: part for part in parts for i in part["patch_ids"]}
    segment_count = closed_segments = loops = sharp_turns = self_crossing = 0
    islands = set()
    lengths = []
    for patch in patches.values():
        positives = 0
        for loop in patch["boundary_loops_mm"]:
            loops += 1
            segment_count += len(loop)
            if owner[patch["id"]]["closed"]:
                closed_segments += len(loop)
            polygon = projected(loop, patch)
            positives += signed_area(polygon) > 0
            self_crossing += crosses_itself(polygon)
            edges = np.roll(polygon, -1, axis=0)-polygon
            size = np.linalg.norm(edges, axis=1)
            lengths.extend(size.tolist())
            units = edges / np.maximum(size[:, None], 1e-20)
            sharp_turns += int(np.sum(np.sum(units*np.roll(units, 1, axis=0), axis=1) < -np.cos(np.pi/6)))
        if positives > 1:
            islands.add(patch["id"])
    reasons = Counter()
    for part in parts:
        if part["closed"]:
            continue
        reason = part["reason"]
        if reason.startswith("Boundary graph has"):
            reason = "boundary_edge_mismatch"
        elif reason == "Unresolved curved/degenerate patches or disconnected boundary islands":
            reason = "curved_missing_boundary_or_disconnected_islands"
        elif reason == "Surface component exceeds bounded closure budget":
            reason = "closure_budget"
        reasons[reason] += 1
    open_parts = [p for p in parts if not p["closed"]]
    count_parts = lambda predicate: sum(any(predicate(patches[i]) for i in p["patch_ids"]) for p in open_parts)
    return {
        **data["summary"], "input_points": input_points,
        "boundary_recovery_totals": dict(sum((Counter(p.get("boundary_recovery", {})) for p in patches.values()), Counter())),
        "working_fraction": data["summary"]["working_points"]/input_points,
        # Historical name retained for saved reports. These are all raw patch
        # edges, before viewport deduplication and selection filtering.
        "display_segments": segment_count, "raw_boundary_segments": segment_count,
        "boundary_basis": "observed" if "unassigned_working_points" in data["summary"] else "legacy",
        "closure_boundary_segments": sum(len(loop) for p in data["patches"] for loop in p["boundary_loops_mm"]),
        "closed_candidate_segments": closed_segments,
        "open_candidate_segments": segment_count-closed_segments,
        "boundary_loops": loops,
        "planar_patches": sum(p["planar"] for p in patches.values()),
        "nonplanar_patches": sum(not p["planar"] for p in patches.values()),
        "planar_patches_without_loops": sum(p["planar"] and not p["boundary_loops_mm"] for p in patches.values()),
        "patches_with_multiple_loops": sum(len(p["boundary_loops_mm"]) > 1 for p in patches.values()),
        "patches_with_disconnected_outer_loops": len(islands),
        "open_parts_with_nonplanar_patches": count_parts(lambda p: not p["planar"]),
        "open_parts_with_missing_planar_loops": count_parts(lambda p: p["planar"] and not p["boundary_loops_mm"]),
        "open_parts_with_disconnected_outer_loops": count_parts(lambda p: p["id"] in islands),
        "nonmanifold_edges": sum(p["nonmanifold_edges"] for p in parts),
        "closed_observation_fraction": sum(len(p["point_ids"]) for p in parts if p["closed"])/input_points,
        "proper_self_crossing_loops": int(self_crossing), "turns_over_150_degrees": sharp_turns,
        "segment_length_quantiles_mm": np.quantile(lengths, [0, .1, .5, .9, 1]).tolist() if lengths else [],
        "spacing_quantiles_mm": np.quantile([p["spacing_mm"] for p in patches.values()], [.1, .5, .9]).tolist(),
        "failure_reasons": dict(reasons),
    }


def run_probe(engine: Path, cloud: Path, destination: Path, options: dict, timeout: float):
    destination.mkdir(parents=True, exist_ok=False)
    write_json(destination / "options.json", options)
    command = [str(engine), "probe-assembly", str(cloud), str(destination / "report.json"),
               str(destination / "options.json")]
    started = time.monotonic()
    with (destination / "probe.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=timeout,
                                creationflags=subprocess.CREATE_NO_WINDOW if __import__("os").name == "nt" else 0)
    report = read(destination / "report.json")
    if result.returncode or report.get("status") != "assembly_probe_completed":
        raise RuntimeError(f"Assembly probe failed: {destination}: {report.get('error')}")
    return {"command": command, "elapsed_seconds": time.monotonic()-started,
            "report": str(destination / "report.json"), "options": report["probe_options"]}


def plot_boundaries(cases, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.ticker import MaxNLocator

    figure = plt.figure(figsize=(16, 12), layout="constrained")
    for index, case in enumerate(cases):
        data = read(Path(case["baseline_directory"]) / "parts.json")
        patches = {p["id"]: observed_patch(p) for p in data["patches"]}
        axis = figure.add_subplot(2, 2, index+1)
        all_points = np.asarray([q for patch in patches.values() for loop in patch["boundary_loops_mm"] for q in loop])
        center = all_points.mean(axis=0)
        _, directions = np.linalg.eigh((all_points-center).T @ (all_points-center))
        directions = directions[:, ::-1]
        for column in range(3):
            if directions[np.argmax(np.abs(directions[:, column])), column] < 0:
                directions[:, column] *= -1
        def projection(points):
            local = (points-center) @ directions
            return np.column_stack((local[:, 0]+.3*local[:, 2], local[:, 1]+.45*local[:, 2]))
        for closed, color in [(False, "#d88426"), (True, "#279d59")]:
            segments = []
            for part in data["parts"]:
                if part["closed"] != closed:
                    continue
                for patch_id in part["patch_ids"]:
                    for loop in patches[patch_id]["boundary_loops_mm"]:
                        points = projection(np.asarray(loop))
                        segments.extend(np.stack((points, np.roll(points, -1, axis=0)), axis=1))
            if segments:
                axis.add_collection(LineCollection(segments, colors=color, linewidths=.6 if not closed else 1.3,
                                                    alpha=.75 if not closed else 1))
        axis.autoscale_view()
        axis.set_aspect("equal", adjustable="datalim")
        axis.xaxis.set_major_locator(MaxNLocator(4)); axis.yaxis.set_major_locator(MaxNLocator(4))
        value = case["baseline"]
        axis.set_title(f"{case['name']}: {value['display_segments']:,} segments; "
                       f"{value['closed_parts']}/{value['candidate_parts']} candidates closed")
        axis.set_xlabel("Projected U / mm"); axis.set_ylabel("Projected V / mm")
    figure.suptitle("Recovered patch contours (orange: open candidate; green: verified closed candidate)\n"
                    "Oblique projection along each workpiece's principal axes; not a GUI screenshot", fontsize=14)
    figure.savefig(destination / "boundary-overview.png", dpi=170)
    plt.close(figure)

    figure, axes = plt.subplots(len(cases), 2, figsize=(13, 4*len(cases)), layout="constrained", squeeze=False)
    for row, case in enumerate(cases):
        data = read(Path(case["baseline_directory"]) / "parts.json")
        candidates = [(len(loop), patch, loop) for patch in data["patches"]
                      for loop in patch["boundary_loops_mm"] if crosses_itself(projected(loop, patch))]
        if not candidates:
            continue
        def shape_score(entry):
            extent = np.ptp(projected(entry[2], entry[1]), axis=0)
            return min(extent)/max(max(extent), 1e-20)
        _, selected, loop = max(candidates, key=shape_score)
        variants = [("baseline", selected)]
        if "alpha_5" in case["probes"]:
            alternate = read(Path(case["probes"]["alpha_5"]["provenance"]["report"]).parent / "parts.json")
            variants.append(("alpha factor 5", next(p for p in alternate["patches"] if p["id"] == selected["id"])))
        for column, (name, patch) in enumerate(variants):
            axis = axes[row, column]
            if not patch["boundary_loops_mm"]:
                axis.text(.5, .5, "No recovered loop", ha="center", va="center", transform=axis.transAxes)
            for contour in patch["boundary_loops_mm"]:
                polygon = projected(contour, patch)
                points = np.vstack((polygon, polygon[0]))
                invalid = crosses_itself(polygon)
                axis.plot(points[:, 0], points[:, 1], ".-", linewidth=.9, markersize=3,
                          color="#c83c36" if invalid else "#387b9d")
            axis.set_aspect("equal", adjustable="datalim")
            axis.set_title(f"{case['name']} / patch {selected['id']} / {name}")
            axis.set_xlabel("Plane U / mm"); axis.set_ylabel("Plane V / mm")
            axis.grid(alpha=.2)
    figure.suptitle("Same patch before/after alpha-radius change; red contours contain proper self-crossings\n"
                    "Local fitted-plane coordinates; no reference model used", fontsize=13)
    figure.savefig(destination / "boundary-details.png", dpi=150)
    plt.close(figure)


def markdown(summary):
    lines = ["# 四工件点云：边界杂线与闭合诊断", "",
             "输入为工件 STEP 的解析采样点云；诊断和重建仅读取 PLY、单位及参数，不读取参考 STEP 或标签。",
             "基线由客户端同一个 CadReconstructionTask 重新执行；参数探针直接调用同一个 reconstructAssembly。",
             "探针会执行内部闭合与 B-Rep 检查，但不导出/回读 STEP，不构成完整性或精度验收。", "",
             "## 基线", "",
             "| 工件 | 工作点/原点 | 候选/闭合 | 原始轮廓线段（开放候选） | 开放边/非流形边 | 非平面片 | 平面片无轮廓 | 自交轮廓 |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for case in summary["cases"]:
        v = case["baseline"]
        lines.append(f"| {case['name']} | {v['working_points']}/{v['input_points']} | {v['candidate_parts']}/{v['closed_parts']} | "
                     f"{v['display_segments']}（{v['open_candidate_segments']}） | {v['open_edges']}/{v['nonmanifold_edges']} | "
                     f"{v['nonplanar_patches']} | {v['planar_patches_without_loops']} | {v['proper_self_crossing_loops']} |")
    lines += ["", "开放边是部件壳拓扑的未配对边；每条显示轮廓首尾相接，并不表示轮廓本身存在未连上的最后一段。",
              "历史字段 display_segments 表示全部原始面片边段；新客户端会去重并按选择过滤，实际默认显示数应查看 GUI 验收报告。",
              "自交统计仅检查二维投影内严格相交的非相邻线段，忽略接触与共线，不是完整多边形有效性检测。", "",
              "## 受控参数对比", "",
              "| 工件 | 变体 | 工作点 | 面片 | 候选/闭合 | 显示线段 | 开放边 | 非流形边 | 自交轮廓 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for case in summary["cases"]:
        for name, v in [("baseline", case["baseline"])] + list(case["probes"].items()):
            lines.append(f"| {case['name']} | {name} | {v['working_points']} | {v['finite_patches']} | "
                         f"{v['candidate_parts']}/{v['closed_parts']} | {v['display_segments']} | {v['open_edges']} | "
                         f"{v['nonmanifold_edges']} | {v['proper_self_crossing_loops']} |")
    lines += ["", "default_probe 检查直接调用和客户端分离结果一致；alpha_5 只将 alpha_factor 从 2.5 改为 5；",
              "full_points 只将 maximum_working_points 提高到输入点数，其余采用默认参数。",
              "这些对比用于定位原因；更多闭合候选不等于更准确，未将参数探针改为生产默认。", "",
              "## 每份案例的失败原因", ""]
    for case in summary["cases"]:
        v = case["baseline"]
        lines += [f"### {case['name']}", "", f"失败分类：`{json.dumps(v['failure_reasons'], ensure_ascii=False)}`。",
                  f"开放候选中，含非平面片 {v['open_parts_with_nonplanar_patches']} 个；"
                  f"含缺失平面轮廓 {v['open_parts_with_missing_planar_loops']} 个；"
                  f"含分离外轮廓 {v['open_parts_with_disconnected_outer_loops']} 个。这三项可以重叠。", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--probe-variants", nargs="+", choices=("default_probe", "alpha_5", "full_points"),
                        default=["default_probe", "alpha_5", "full_points"])
    parser.add_argument("--executable", type=Path, default=DEFAULT_ENGINE)
    parser.add_argument("--timeout", type=float, default=300)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    engine = args.executable.resolve()
    baseline = args.baseline.resolve()
    manifest = read(baseline / "summary.json")
    summary = {"schema": "paramcad.assembly-boundary-diagnostics", "version": 2, "status": "running",
               "source_model_read": False, "reference_labels_used": False,
               "baseline_manifest": str(baseline / "summary.json"),
               "probe_executable_sha256": sha256(engine) if args.probe else None,
               "baseline_executable_sha256": manifest["executable_sha256"], "cases": []}
    for row in manifest["cases"]:
        directory = baseline / row["name"]
        report = read(directory / "report.json")
        count = report["input_points"]
        cloud = directory / "input/cloud.ply"
        if sha256(cloud) != row["source_cloud_sha256"]:
            raise ValueError(f"Input checksum changed: {cloud}")
        case = {"name": row["name"], "baseline_directory": str(directory), "cloud": str(cloud),
                "cloud_sha256": row["source_cloud_sha256"], "baseline": metrics(directory, count), "probes": {}}
        summary["cases"].append(case)
        if args.probe:
            for name, options in [("default_probe", {}), ("alpha_5", {"alpha_factor": 5}),
                                  ("full_points", {"maximum_working_points": count})]:
                if name not in args.probe_variants:
                    continue
                print(f"{case['name']}: {name}", flush=True)
                destination = output / "probes" / case["name"] / name
                provenance = run_probe(engine, cloud, destination, options, args.timeout)
                case["probes"][name] = {**metrics(destination, count), "provenance": provenance}
                if name == "default_probe":
                    for field in ("candidate_parts", "closed_parts", "closure_boundary_segments", "open_edges", "finite_patches"):
                        if case["probes"][name][field] != case["baseline"][field]:
                            raise RuntimeError(f"Default probe differs from client baseline: {case['name']}: {field}")
                write_json(output / "summary.json", summary)
    plot_boundaries(summary["cases"], output)
    summary["status"] = "completed"
    write_json(output / "summary.json", summary)
    (output / "README.md").write_text(markdown(summary), encoding="utf-8")
    print(f"Saved diagnostics: {output}", flush=True)


if __name__ == "__main__":
    main()
