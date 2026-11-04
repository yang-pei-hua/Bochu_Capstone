#!/usr/bin/env python3
"""Export every inferred part and a 12-view contact sheet, plus numbered parent atlases.

Input is run_point_cloud_cad.py output (summary.json, original PLY and parts.json).
Requires numpy and Pillow; uses only observed samples, never reference CAD/labels.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import colorsys
import csv
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import re
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from run_point_cloud_cad import DEFAULT_ENGINE, sha256, write_json


SCALARS = {"char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
           "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
           "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
           "float": "f4", "float32": "f4", "double": "f8", "float64": "f8"}
PLY_NAMES = {"i1": "char", "u1": "uchar", "i2": "short", "u2": "ushort",
             "i4": "int", "u4": "uint", "f4": "float", "f8": "double"}
VIEWS = [("主轴 +X", (1, 0, 0)), ("主轴 −X", (-1, 0, 0)),
         ("主轴 +Y", (0, 1, 0)), ("主轴 −Y", (0, -1, 0)),
         ("主轴 +Z", (0, 0, 1)), ("主轴 −Z", (0, 0, -1)),
         ("斜视 +X +Y +Z", (1, 1, 1)), ("斜视 +X −Y +Z", (1, -1, 1)),
         ("斜视 −X +Y +Z", (-1, 1, 1)), ("斜视 −X −Y +Z", (-1, -1, 1)),
         ("仰视 +X +Y −Z", (1, 1, -1)), ("仰视 −X −Y −Z", (-1, -1, -1))]
BACKGROUND = (245, 248, 251)
INK = (31, 44, 61)
MUTED = (91, 108, 125)


def read_ply(path: Path) -> np.ndarray:
    """Read scalar vertex properties without converting source doubles to floats.

    Vertex must be the first nonempty element; subsequent mesh elements are ignored.
    List-valued vertex properties are deliberately rejected.
    """
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError(f"Not a PLY: {path}")
        encoding = None
        properties = []
        current = None
        count = None
        before_vertices = False
        for _ in range(10000):
            line = stream.readline()
            if not line:
                raise ValueError("Incomplete PLY header")
            fields = line.decode("ascii").split()
            if not fields or fields[0] in ("comment", "obj_info"):
                continue
            if fields[0] == "end_header":
                break
            if fields[0] == "format":
                encoding = fields[1]
            elif fields[0] == "element":
                current = fields[1]
                if current == "vertex":
                    if count is not None or before_vertices:
                        raise ValueError("Expected one vertex element before other data")
                    count = int(fields[2])
                elif count is None and int(fields[2]):
                    before_vertices = True
            elif fields[0] == "property" and current == "vertex":
                if fields[1] not in SCALARS:
                    raise ValueError("Only scalar vertex properties are supported")
                properties.append((fields[2], SCALARS[fields[1]]))
        else:
            raise ValueError("Excessive PLY header")
        if count is None or count < 0 or not {"x", "y", "z"}.issubset(n for n, _ in properties):
            raise ValueError("PLY needs vertex x/y/z properties")
        if len({n for n, _ in properties}) != len(properties):
            raise ValueError("Repeated PLY property")
        endian = ">" if encoding == "binary_big_endian" else "<"
        dtype = np.dtype([(n, endian+t) for n, t in properties])
        if encoding == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, max_rows=count, ndmin=2) if count else np.empty((0, len(properties)))
            if values.shape != (count, len(properties)):
                raise ValueError("PLY vertex count/property mismatch")
            result = np.empty(count, dtype=dtype)
            for i, (name, _) in enumerate(properties):
                result[name] = values[:, i]
        elif encoding in ("binary_little_endian", "binary_big_endian"):
            result = np.fromfile(stream, dtype=dtype, count=count)
            if len(result) != count:
                raise ValueError("Truncated binary PLY")
        else:
            raise ValueError("Unsupported PLY format")
    if not np.isfinite(positions(result)).all():
        raise ValueError("Nonfinite point coordinates")
    return result


def positions(cloud):
    return np.column_stack([cloud[a] for a in ("x", "y", "z")]).astype(np.float64, copy=False)


def normals(cloud):
    return np.column_stack([cloud[a] for a in ("nx", "ny", "nz")]) if {"nx", "ny", "nz"}.issubset(cloud.dtype.names) else None


def write_part(path: Path, source: np.ndarray, indices: np.ndarray, part_id: int):
    names = [n for n in source.dtype.names if n not in ("source_point_id", "part_id")]
    properties = [(n, source.dtype.fields[n][0].newbyteorder("<")) for n in names]
    dtype = np.dtype(properties + [("source_point_id", "<u4"), ("part_id", "<u4")])
    result = np.empty(len(indices), dtype=dtype)
    for name in names:
        result[name] = source[name][indices]
    result["source_point_id"] = indices+1
    result["part_id"] = part_id
    header = ["ply", "format binary_little_endian 1.0",
              "comment original observations; world coordinates unchanged",
              "comment source_point_id is the 1-based row in the source PLY",
              f"element vertex {len(result)}"]
    for name in dtype.names:
        scalar = dtype.fields[name][0]
        header.append(f"property {PLY_NAMES[scalar.kind+str(scalar.itemsize)]} {name}")
    header.append("end_header")
    with path.open("wb") as stream:
        stream.write(("\n".join(header)+"\n").encode("ascii"))
        result.tofile(stream)


def partition(source_count: int, report: dict):
    labels = np.zeros(source_count, dtype=np.uint32)
    ids_seen = set()
    groups = []
    for part in sorted(report["parts"], key=lambda p: p["id"]):
        part_id = part["id"]
        if not isinstance(part_id, int) or not 0 < part_id < 2**32 or part_id in ids_seen:
            raise ValueError("Invalid/repeated part ID")
        ids_seen.add(part_id)
        raw = np.asarray(part["point_ids"])
        if raw.ndim != 1 or not np.issubdtype(raw.dtype, np.number) or not np.isfinite(raw).all() or np.any(raw != np.floor(raw)):
            raise ValueError("Expected finite integral original point IDs")
        indices = np.sort(raw.astype(np.int64)-1)
        if len(indices) == 0:
            raise ValueError("Empty part candidate")
        if indices[0] < 0 or indices[-1] >= source_count or len(np.unique(indices)) != len(indices):
            raise ValueError("Out-of-range/repeated source point ID")
        if np.any(labels[indices]):
            raise ValueError("Point assigned to more than one candidate")
        labels[indices] = part_id
        groups.append((part, indices))
    residual = np.flatnonzero(labels == 0)
    expected = report["summary"]["unassigned_points"]
    if expected != len(residual):
        raise ValueError("Unassigned count disagrees with point membership")
    return labels, groups, residual


def principal_frame(points):
    center = points.mean(axis=0)
    centered = points-center
    if len(points) < 3:
        return center, np.eye(3)
    covariance = centered.T @ centered/max(1, len(points)-1)
    _, basis = np.linalg.eigh(covariance)
    basis = basis[:, ::-1].copy()
    for axis in (0, 1):
        if basis[np.argmax(np.abs(basis[:, axis])), axis] < 0:
            basis[:, axis] *= -1
    basis[:, 2] = np.cross(basis[:, 0], basis[:, 1])
    return center, basis


def camera(direction):
    toward = np.asarray(direction, dtype=float)
    toward /= np.linalg.norm(toward)
    up = np.array([0., 0., 1.]) if abs(toward[2]) < .95 else np.array([0., 1., 0.])
    right = np.cross(up, toward)
    right /= np.linalg.norm(right)
    up = np.cross(toward, right)
    return np.column_stack((right, up, toward))


def color(part_id):
    if part_id == 0:
        return (113, 126, 140)
    return tuple(round(c*255) for c in colorsys.hsv_to_rgb((part_id*.618033988749895) % 1, .63, .79))


def label_colors(labels):
    unique, inverse = np.unique(labels, return_inverse=True)
    return np.asarray([color(int(i)) for i in unique], dtype=np.uint8)[inverse]


@lru_cache(maxsize=24)
def font(size):
    for path in ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def text(draw, xy, value, size=22, fill=INK, **kwargs):
    draw.text(xy, value, font=font(size), fill=fill, **kwargs)


def sample_indices(count, limit):
    return np.linspace(0, count-1, min(count, limit), dtype=np.int64)


def render_points(points, direction, width, height, tint, point_normals=None, *, scale=None,
                  middle=None, radius=1, background=BACKGROUND):
    view = camera(direction)
    projected = points @ view
    if middle is None:
        middle = (projected[:, :2].min(axis=0)+projected[:, :2].max(axis=0))*.5
    extent = np.ptp(projected[:, :2], axis=0)
    if scale is None:
        scale = min((width-32)/max(extent[0], 1e-9), (height-32)/max(extent[1], 1e-9))
    xy = np.column_stack(((projected[:, 0]-middle[0])*scale+width*.5,
                          -(projected[:, 1]-middle[1])*scale+height*.5))
    pixels = np.rint(xy).astype(np.int32)
    tints = np.broadcast_to(np.asarray(tint, dtype=float), (len(points), 3))
    if point_normals is not None:
        lengths = np.linalg.norm(point_normals, axis=1)
        light = .52+.48*np.abs(point_normals @ view[:, 2]/np.maximum(lengths, 1e-20))
    else:
        depth = projected[:, 2]
        light = .7+.3*(depth-depth.min())/max(float(np.ptp(depth)), 1e-12)
    shades = np.clip(tints*light[:, None], 0, 255).astype(np.uint8)
    canvas = np.full((height*width, 3), background, dtype=np.uint8)
    zbuffer = np.full(height*width, -np.inf)
    offsets = [(dx, dy) for dy in range(-radius, radius+1) for dx in range(-radius, radius+1)
               if dx*dx+dy*dy <= radius*radius]
    for dx, dy in offsets:
        x, y = pixels[:, 0]+dx, pixels[:, 1]+dy
        valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        indices = np.flatnonzero(valid)
        flat = y[valid]*width+x[valid]
        depth = projected[valid, 2]
        np.maximum.at(zbuffer, flat, depth)
        visible = depth >= zbuffer[flat]
        canvas[flat[visible]] = shades[indices[visible]]
    return Image.fromarray(canvas.reshape(height, width, 3)), xy, scale, middle


def draw_axis(draw, corner, direction, scale=23):
    view = camera(direction)
    for axis, shade in zip(range(3), [(199, 69, 69), (44, 141, 85), (57, 108, 201)]):
        x, y = view[axis, :2]*scale
        end = (corner[0]+x, corner[1]-y)
        draw.line((corner, end), fill=shade, width=2)
        if x*x+y*y > 12:
            text(draw, end, "XYZ"[axis], size=14, fill=shade)


def render_contact_sheet(task):
    directory = Path(task["directory"])
    cloud = read_ply(directory/"cloud.ply")
    all_points = positions(cloud)
    center, basis = principal_frame(all_points)
    indices = sample_indices(len(cloud), task["preview_points"])
    points = (all_points[indices]-center) @ basis
    n = normals(cloud)
    if n is not None:
        n = n[indices] @ basis
    views = [points @ camera(direction) for _, direction in VIEWS]
    extents = np.max(np.asarray([np.ptp(p[:, :2], axis=0) for p in views]), axis=0)
    tile_w, tile_h, header = 440, 360, 126
    plot_w, plot_h = tile_w-30, tile_h-58
    scale = min((plot_w-30)/max(extents[0], 1e-9), (plot_h-30)/max(extents[1], 1e-9))
    sheet = Image.new("RGB", (tile_w*4, header+tile_h*3+56), "white")
    draw = ImageDraw.Draw(sheet)
    text(draw, (25, 14), task["name"], size=30)
    state = "未归属点（000）" if task["part_id"] == 0 else ("已闭合候选" if task["closed"] else "未闭合候选")
    text(draw, (25, 57), f"{len(cloud):,} 个原始点  ·  {state}  ·  单位 {task['unit']}", size=20)
    local = (all_points-center) @ basis
    dimensions = np.ptp(local, axis=0)
    text(draw, (25, 88), "主轴包围尺寸："+" × ".join(f"{v:.4g}" for v in dimensions)+f" {task['unit']}  ·  12 图使用相同正交比例", size=18, fill=MUTED)
    for index, ((label, direction), projected) in enumerate(zip(VIEWS, views)):
        left, top = (index % 4)*tile_w, header+(index//4)*tile_h
        image, _, _, _ = render_points(points, direction, plot_w, plot_h, color(task["part_id"]), n,
                                        scale=scale, radius=2 if len(points) < 1200 else 1)
        sheet.paste(image, (left+15, top+40))
        draw.rectangle((left+6, top+4, left+tile_w-6, top+tile_h-4), outline=(218, 226, 234))
        text(draw, (left+18, top+10), label, size=18)
        draw_axis(draw, (left+40, top+tile_h-45), direction)
    text(draw, (24, sheet.height-40), f"显示 {len(indices):,}/{len(cloud):,} 点；PLY 保留全部点及原始坐标。主轴旋转仅用于图片，未修改点云。", size=18, fill=MUTED)
    sheet.save(directory/"views.png", optimize=True)
    metadata = json.loads((directory/"cloud.json").read_text(encoding="utf-8"))
    metadata["preview"] = {"image": "views.png", "view_count": len(VIEWS), "displayed_points": len(indices),
                           "pca_origin_world": center.tolist(), "pca_axes_columns_world": basis.tolist(),
                           "pca_dimensions": dimensions.tolist(), "shared_orthographic_scale": True,
                           "views": [{"name": label, "camera_direction_in_pca_frame": list(direction)} for label, direction in VIEWS]}
    write_json(directory/"cloud.json", metadata)
    return {"name": task["name"], "image": str(directory/"views.png"), "points": len(cloud)}


def spread_labels(anchors, top, bottom, gap=39):
    """Allocate one label lane per anchor; leader ends stay tied to source points."""
    if not anchors:
        return []
    ordered = sorted(anchors, key=lambda x: (x[1], x[0]))
    y = np.asarray([np.clip(a[1], top, bottom) for a in ordered], dtype=float)
    gap = min(gap, (bottom-top)/max(1, len(ordered)-1))
    for i in range(1, len(y)):
        y[i] = max(y[i], y[i-1]+gap)
    if y[-1] > bottom:
        y -= y[-1]-bottom
    for i in range(len(y)-2, -1, -1):
        y[i] = min(y[i], y[i+1]-gap)
    if y[0] < top:
        y += top-y[0]
    return [(anchor, float(height)) for anchor, height in zip(ordered, y)]


def locator_atlas(name, source, labels, groups, directory, per_panel):
    xyz = positions(source)
    center, basis = principal_frame(xyz)
    local = (xyz-center) @ basis
    # Looking along the third principal axis reveals the full extent of thin
    # plates. A small tilt also exposes depth; front/back panels alternate.
    directions = [(0, -.42, 1), (0, .42, -1)]
    sample = sample_indices(len(xyz), 150000)
    rows = [p for p, _ in groups]
    batches = [rows[i:i+per_panel] for i in range(0, len(rows), per_panel)]
    requested_panels = len(batches)+1
    columns = 3 if requested_panels >= 7 else 2
    panel_count = math.ceil(requested_panels/columns)*columns
    panel_w, panel_h, header = 1280, 920, 125
    atlas = Image.new("RGB", (columns*panel_w, math.ceil(panel_count/columns)*panel_h+header+70), "white")
    draw = ImageDraw.Draw(atlas)
    text(draw, (30, 16), f"{name}  ·  候选点云编号定位图", size=36)
    text(draw, (30, 67), f"全部 {len(rows)} 个候选编号均在下方分组标注；编号与 {name}_NNN 文件夹一致。000 是未归属点，不作为零件标注。", size=23)
    lookup = {p["id"]: ix for p, ix in groups}
    anchors_world = {}
    anchor_indices = {}
    annotations = []
    for part, indices in groups:
        points = xyz[indices]
        middle = np.median(points, axis=0)
        anchor_indices[part["id"]] = int(indices[np.argmin(np.sum((points-middle)**2, axis=1))])
        anchors_world[part["id"]] = local[anchor_indices[part["id"]]]
    for panel in range(panel_count):
        left, top = panel % columns*panel_w, header+panel//columns*panel_h
        overview = panel == 0 or panel >= requested_panels
        current = [] if overview else batches[panel-1]
        direction = directions[(panel-1) % 2] if not overview else directions[0 if panel == 0 else 1]
        view = camera(direction)
        projected = local @ view
        middle = (projected[:, :2].min(axis=0)+projected[:, :2].max(axis=0))*.5
        plot_w, plot_h, px, py = panel_w-250, panel_h-125, 125, 64
        extent = np.ptp(projected[:, :2], axis=0)
        scale = min((plot_w-42)/max(extent[0], 1e-9), (plot_h-42)/max(extent[1], 1e-9))
        tints = np.full((len(sample), 3), (190, 201, 211), dtype=np.uint8)
        if overview:
            tints = label_colors(labels[sample])
        image, _, _, _ = render_points(local[sample], direction, plot_w, plot_h, tints,
                                        scale=scale, middle=middle, radius=1)
        atlas.paste(image, (left+px, top+py))
        if current:
            # Render highlighted observations independently, preserving even
            # tiny candidates that a whole-scene preview sample could miss.
            selected_indices = np.concatenate([lookup[p["id"]][sample_indices(len(lookup[p["id"]]), 12000)] for p in current])
            highlighted, _, _, _ = render_points(local[selected_indices], direction, plot_w, plot_h,
                label_colors(labels[selected_indices]), scale=scale, middle=middle, radius=2,
                background=(255, 255, 255))
            mask = np.any(np.asarray(highlighted) != 255, axis=2)
            image_values = np.asarray(image).copy()
            image_values[mask] = np.asarray(highlighted)[mask]
            atlas.paste(Image.fromarray(image_values), (left+px, top+py))
        title = ("整件总览 · 正向" if panel == 0 else "整件总览 · 背向") if overview else f"编号 {current[0]['id']:03d} — {current[-1]['id']:03d}  ·  "+("背向观察" if (panel-1) % 2 else "正向观察")
        text(draw, (left+24, top+16), title, size=25)
        sides = [[], []]
        projected_anchors = {}
        for part in current:
            position = anchors_world[part["id"]] @ view
            x = (position[0]-middle[0])*scale+plot_w*.5+px
            y = -(position[1]-middle[1])*scale+plot_h*.5+py
            projected_anchors[part["id"]] = (x, y)
            sides[0 if x < panel_w*.5 else 1].append((part["id"], y))
        for side, anchors in enumerate(sides):
            for (part_id, _), y in spread_labels(anchors, 92, panel_h-65):
                x = 66 if side == 0 else panel_w-66
                target_x, target_y = projected_anchors[part_id]
                shade = color(part_id)
                draw.line((left+x, top+y, left+target_x, top+target_y), fill=shade, width=2)
                draw.ellipse((left+target_x-4, top+target_y-4, left+target_x+4, top+target_y+4), fill=shade, outline="white", width=1)
                draw.rounded_rectangle((left+x-39, top+y-17, left+x+39, top+y+18), radius=6, fill="white", outline=shade, width=2)
                text(draw, (left+x, top+y), f"{part_id:03d}", size=24, anchor="mm", fill=shade)
                source_index = anchor_indices[part_id]
                annotations.append({"part_id": part_id, "panel": panel,
                                    "source_point_id": source_index+1,
                                    "anchor_world": xyz[source_index].tolist(),
                                    "anchor_pixel": [left+target_x, top+target_y],
                                    "label_pixel": [left+x, top+y],
                                    "camera_direction_in_parent_pca": list(direction)})
        draw.rectangle((left+5, top+4, left+panel_w-5, top+panel_h-6), outline=(211, 220, 230), width=2)
        text(draw, (left+24, top+panel_h-43), "引线端点是该候选的真实观测点；高亮显示本栏编号，灰色为其余点。", size=18, fill=MUTED)
    text(draw, (28, atlas.height-45), "候选编号是算法分组，不代表原装配实例；薄片/相邻小件请放大编号图，并查看独立 views.png。", size=21, fill=MUTED)
    path = directory/f"{name}_locator.png"
    atlas.save(path, optimize=True)
    return {"image": str(path), "labelled_ids": [p["id"] for p in rows],
            "panels": panel_count, "labels_per_panel": per_panel,
            "annotations": annotations,
            "pca_origin_world": center.tolist(), "pca_axes_columns_world": basis.tolist()}


def verify_export_package(output: Path):
    summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
    validation = {"schema": "paramcad.component-cloud-review-validation", "cases": []}
    for case in summary["cases"]:
        if sha256(Path(case["source_cloud"])) != case["source_sha256"]:
            raise ValueError("Source changed during export")
        source = read_ply(Path(case["source_cloud"]))
        report = json.loads(Path(case["parts_report"]).read_text(encoding="utf-8"))
        labels, groups, _ = partition(len(source), report)
        seen = np.zeros(len(source), dtype=bool)
        for item in case["exported_clouds"]:
            directory = output/item["directory"]
            cloud = read_ply(directory/"cloud.ply")
            indices = cloud["source_point_id"].astype(np.int64)-1
            if (len(cloud) != item["points"] or len(np.unique(indices)) != len(indices)
                    or np.any(indices < 0) or np.any(indices >= len(source)) or np.any(seen[indices])):
                raise ValueError(f"Invalid/duplicate exported observations: {directory}")
            seen[indices] = True
            if not np.all(labels[indices] == item["part_id"]) or not np.all(cloud["part_id"] == item["part_id"]):
                raise ValueError("Exported candidate membership differs from reconstruction")
            for name in source.dtype.names:
                if name in ("source_point_id", "part_id"):
                    continue
                if not np.array_equal(cloud[name], source[name][indices], equal_nan=True):
                    raise ValueError(f"Original attribute changed: {directory}: {name}")
            if sha256(directory/"cloud.ply") != item["cloud_sha256"]:
                raise ValueError("Cloud changed while rendering preview")
            metadata = json.loads((directory/"cloud.json").read_text(encoding="utf-8"))
            if metadata["preview"]["view_count"] != len(VIEWS) or metadata["name"] != item["name"]:
                raise ValueError("Preview metadata mismatch")
            with Image.open(directory/"views.png") as image:
                if image.size != (1760, 1262):
                    raise ValueError("Unexpected contact-sheet dimensions")
                image.verify()
        if not seen.all():
            raise ValueError("Some original observations were not exported")
        annotations = case["locator"]["annotations"]
        if len(annotations) != len(groups) or {a["part_id"] for a in annotations} != {p["id"] for p, _ in groups}:
            raise ValueError("Locator does not label every candidate exactly once")
        xyz = positions(source)
        for annotation in annotations:
            index = annotation["source_point_id"]-1
            if labels[index] != annotation["part_id"] or not np.array_equal(annotation["anchor_world"], xyz[index]):
                raise ValueError("Locator leader is not anchored to its own candidate")
        with Image.open(case["locator"]["image"]) as image:
            image.verify()
        validation["cases"].append({"name": case["name"], "points": len(source),
            "folders": len(case["exported_clouds"]), "numbered_candidates": len(annotations),
            "original_attributes_exact": True, "all_points_exported_once": True,
            "all_images_verified": True, "label_anchors_membership_verified": True})
        print(f"{case['name']}: full point/attribute/image/label verification passed", flush=True)
    validation["status"] = "passed"
    validation["total_clouds"] = sum(c["folders"] for c in validation["cases"])
    validation["total_source_points"] = sum(c["points"] for c in validation["cases"])
    write_json(output/"validation.json", validation)
    return validation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reconstruction", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=min(4, os.cpu_count() or 1))
    parser.add_argument("--preview-points", type=int, default=60000)
    parser.add_argument("--locator-labels", type=int, default=16)
    parser.add_argument("--engine", type=Path, default=DEFAULT_ENGINE,
                        help="Current engine; must match the reconstruction manifest hash")
    args = parser.parse_args()
    if not 1 <= args.workers <= 16 or args.preview_points < 100 or not 4 <= args.locator_labels <= 24:
        parser.error("workers must be 1..16; preview-points >=100; locator-labels 4..24")
    root = args.reconstruction.resolve()
    output = args.output.resolve()
    if output.exists():
        raise ValueError("Output must be a new directory")
    manifest_path = root/"summary.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "paramcad.point-cloud-only-cad":
        raise ValueError("Expected point-cloud-only reconstruction manifest")
    if sha256(args.engine.resolve()) != manifest["executable_sha256"]:
        raise ValueError("Reconstruction engine differs from current executable; rerun reconstruction first")
    output.mkdir(parents=True)
    started = time.monotonic()
    summary = {"schema": "paramcad.component-cloud-review", "version": 1, "status": "running",
               "reconstruction_manifest": str(manifest_path), "reconstruction_manifest_sha256": sha256(manifest_path),
               "engine_sha256": manifest["executable_sha256"], "preview_only_downsampling": True,
               "exported_coordinates": "unchanged_source_world_frame", "views_per_cloud": len(VIEWS), "cases": []}
    tasks = []
    index_rows = []
    for case in manifest["cases"]:
        name = case["name"]
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
            raise ValueError("Invalid case name")
        source_path = Path(case["cloud"])
        if not source_path.is_absolute():
            source_path = root/source_path
        if sha256(source_path) != case["source_cloud_sha256"]:
            raise ValueError(f"Changed source cloud: {name}")
        source = read_ply(source_path)
        report_path = root/name/"parts.json"
        report = json.loads(report_path.read_text(encoding="utf-8"))
        labels, groups, residual = partition(len(source), report)
        row = {"name": name, "source_cloud": str(source_path), "source_sha256": case["source_cloud_sha256"],
               "parts_report": str(report_path), "parts_report_sha256": sha256(report_path),
               "input_points": len(source), "candidate_count": len(groups), "unassigned_points": len(residual),
               "exported_clouds": [], "partition_complete_and_disjoint": True}
        summary["cases"].append(row)
        print(f"{name}: exporting {len(groups)} candidates + {len(residual):,} unassigned observations", flush=True)
        all_groups = groups+([({"id": 0, "closed": False, "status": "unassigned", "reason": "Observations not assigned to a candidate"}, residual)] if len(residual) else [])
        for part, indices in all_groups:
            basename = f"{name}_{part['id']:03d}"
            directory = output/basename
            directory.mkdir()
            write_part(directory/"cloud.ply", source, indices, part["id"])
            xyz = positions(source[indices])
            metadata = {"unit": case["unit"], "name": basename, "parent_case": name, "part_id": part["id"],
                        "kind": "unassigned" if part["id"] == 0 else "inferred_candidate",
                        "closed": part["closed"], "status": part["status"], "reason": part.get("reason", ""),
                        "point_count": len(indices), "cloud": "cloud.ply", "image": "views.png",
                        "source_cloud": str(source_path), "source_cloud_sha256": case["source_cloud_sha256"],
                        "source_id_attribute": "source_point_id", "source_id_base": 1,
                        "world_coordinates_unchanged": True, "lower_world": xyz.min(axis=0).tolist(),
                        "upper_world": xyz.max(axis=0).tolist(), "source_patch_ids": part.get("patch_ids", [])}
            write_json(directory/"cloud.json", metadata)
            item = {"name": basename, "part_id": part["id"], "closed": part["closed"],
                    "points": len(indices), "directory": str(directory.relative_to(output)),
                    "cloud_sha256": sha256(directory/"cloud.ply")}
            row["exported_clouds"].append(item)
            index_rows.append({"parent": name, "number": part["id"], "folder": basename,
                               "point_count": len(indices), "closed": part["closed"], "status": part["status"],
                               "cloud": f"{basename}/cloud.ply", "image": f"{basename}/views.png"})
            tasks.append({"directory": str(directory), "name": basename, "part_id": part["id"],
                          "closed": part["closed"], "unit": case["unit"], "preview_points": args.preview_points})
        row["locator"] = locator_atlas(name, source, labels, groups, output, args.locator_labels)
        write_json(output/"summary.json", summary)
    print(f"Rendering {len(tasks)} contact sheets with {args.workers} workers", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(render_contact_sheet, task) for task in tasks]
        for count, future in enumerate(as_completed(futures), 1):
            future.result()
            if count % 20 == 0 or count == len(futures):
                print(f"Contact sheets {count}/{len(futures)}", flush=True)
    with (output/"index.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(index_rows[0]))
        writer.writeheader();writer.writerows(index_rows)
    verify_export_package(output)
    summary["status"] = "completed"
    summary["exported_cloud_count"] = len(tasks)
    summary["elapsed_seconds"] = time.monotonic()-started
    write_json(output/"summary.json", summary)
    sample_cloud = next((row["cloud"] for row in index_rows if row["closed"]), index_rows[0]["cloud"])
    lines = ["# 工件候选点云人工检查包", "", "每个编号文件夹包含 cloud.ply、cloud.json、views.png（12 视角）。",
             "保留所有闭合/未闭合候选；000 单独保存未归属点。PLY 保留原坐标、法向及原始属性，source_point_id 对应源文件的 1-based 点行。",
             "图片采用该候选的 PCA 主轴坐标，相同正交比例；旋转和显示采样仅用于图片。导出的 PLY 不做降采样或重定位。",
             "每件的 *_locator.png 分栏标注全部候选编号；index.csv 可按名称、编号、状态和点数检索。", "",
             "| 工件 | 候选文件夹 | 未归属点 | 定位图 |", "| --- | ---: | ---: | --- |"]
    for case in summary["cases"]:
        lines.append(f"| {case['name']} | {case['candidate_count']} | {case['unassigned_points']} | {case['name']}_locator.png |")
    lines += ["", "输入是实际工件 STEP 的解析采样点云；编号代表当前算法候选，不是经核实的原装配实例。",
              "所有候选都可以单独用于后续算法测试；务必将 cloud.json 与 cloud.ply 放在一起，以保留毫米/米单位。",
              "validation.json 保存全量逐点/属性比对、点覆盖、图像完整性及编号引线所属集合检查结果。", "",
              "## 批量重建检查包", "", "在仓库根目录运行，输出目录使用一个新目录：", "", "```powershell",
              f'python tools/export_component_clouds.py --reconstruction "{root}" --output outputs/component-cloud-review-new --workers 4',
              "```", "", "## 测试一个孤立点云", "", "```powershell",
              f'python tools/run_point_cloud_cad.py --input isolated="{output / sample_cloud}" --output outputs/isolated-cloud-test-new --timeout 300',
              "```"]
    (output/"README.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(f"Completed: {len(tasks)} clouds/contact sheets; {len(summary['cases'])} locator atlases; {output}", flush=True)


if __name__ == "__main__":
    main()
