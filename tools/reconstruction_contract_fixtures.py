"""Analytic v1 observations and reference artifacts for the contract runner."""
from __future__ import annotations

from itertools import combinations
import math
from pathlib import Path

from run_point_cloud_cad import sha256


def write_cloud(path: Path, points):
    header = ["ply", "format ascii 1.0", f"element vertex {len(points)}"]
    header += [f"property float {field}" for field in ("x", "y", "z", "nx", "ny", "nz")]
    lines = [" ".join(format(x, ".12g") for x in (*position, *normal)) for position, normal in points]
    path.write_bytes(("\n".join([*header, "end_header", *lines]) + "\n").encode("ascii"))


def working_references(points):
    rows = [[str(i), normal, True, None] for i, (_, normal) in enumerate(points, 1)]
    return {0: {"observation_ids": [row[0] for row in rows]},
            **{stage: {"samples": rows} for stage in (1, 2, 3, 4)}}


def planar_region(axis, sign, coordinate, indices):
    normal = [0, 0, 0]
    normal[axis] = sign
    return {"type": 0, "axis": normal, "offset": -sign * coordinate}, indices


def cylindrical_region(center, radius, start, end, indices):
    return {"type": 1, "center": [*center, 0], "axis": [0, 0, 1], "radius": radius,
            "axial_min": start, "axial_max": end, "angular_coverage": 1}, indices


def analytic_analysis(points, regions):
    evidence, patches = [], []
    for i, (geometry, indices) in enumerate(regions, 1):
        support = [str(index + 1) for index in indices]
        positions = list(zip(*(points[index][0] for index in indices)))
        normals = list(zip(*(points[index][1] for index in indices)))
        evidence.append({**geometry, "id": str(i), "support_ids": support, "confidence": 1.0})
        patches.append({"surface_id": i, "support_ids": support, "lower": [min(v) for v in positions],
                        "upper": [max(v) for v in positions], "centroid": [sum(v) / len(v) for v in positions],
                        "normal": [sum(v) / len(v) for v in normals], "oriented": True})
    references = working_references(points)
    references[6] = {**references[4], "evidence": evidence}
    references[7] = {**references[6], "surface_patches": patches, "surface_relations": []}
    relations = []
    for a, b in combinations(range(len(regions)), 2):
        first, second = evidence[a], evidence[b]
        if first["type"] == second["type"] == 0:
            alignment = abs(sum(x * y for x, y in zip(first["axis"], second["axis"])))
            relations.append([a + 1, b + 1, 1 if alignment > .5 else 2, 0, 1, True])
    references[8] = {**references[7], "surface_relations": relations}
    return references


def known_host(origin, sizes):
    return {"primitives": [{"id": "1", "name": "reference host box", "suppressed": False,
                            "params": {"kind": 3, "join": False,
                                       "values": [*origin, 1, 0, 0, 0, 0, 1, *sizes]}}],
            "prisms": []}


def box_fixtures(boxes, tolerance):
    points, patches, parts, connections, work = [], [], [], [], []
    for spec in boxes:
        center, dimensions = spec["center"], spec["size"]
        lower = [c - d / 2 for c, d in zip(center, dimensions)]
        upper = [c + d / 2 for c, d in zip(center, dimensions)]
        patch_start = len(patches)
        first_id = len(points) + 1
        for axis in range(3):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            for side in (0, 1):
                if spec.get("missing_face") == 2 * axis + side:
                    continue
                sign = 1 if side else -1
                normal = [0, 0, 0]
                normal[axis] = sign
                x_axis = [0, 0, 0]
                x_axis[u] = 1
                origin = center.copy()
                origin[axis] = (upper if side else lower)[axis]
                rows = []
                for i in range(16):
                    for j in range(16):
                        p = origin.copy()
                        p[u] = lower[u] + (i + .5) * dimensions[u] / 16
                        p[v] = lower[v] + (j + .5) * dimensions[v] / 16
                        points.append((p, normal))
                        rows.append(str(len(points)))
                        work.append([rows[-1], normal, True, max(dimensions[u], dimensions[v]) / 16])
                corners = []
                for i, j in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    p = origin.copy()
                    p[u], p[v] = (upper if i else lower)[u], (upper if j else lower)[v]
                    corners.append(p)
                if sign < 0:
                    corners.reverse()
                lo, hi = lower.copy(), upper.copy()
                lo[axis] = hi[axis] = origin[axis]
                patches.append({"origin": origin, "normal": normal, "x": x_axis,
                                "lower": lo, "upper": hi, "spacing": max(dimensions[u], dimensions[v]) / 16,
                                "planar": True, "oriented": True, "working_ids": rows, "source_ids": [],
                                "loops": [corners]})
        patch_ids = list(range(patch_start + 1, len(patches) + 1))
        for a, b in combinations(patch_ids, 2):
            normal_a, normal_b = patches[a - 1]["normal"], patches[b - 1]["normal"]
            if sum(x * y for x, y in zip(normal_a, normal_b)) == 0:
                connections.append([a, b, True, 8, .4, True, True])
        parts.append({"patch_ids": patch_ids, "oriented": True, "lower": lower, "upper": upper,
                      "working_ids": [str(i) for i in range(first_id, len(points) + 1)], "source_ids": []})

    base = {"work": work, "tolerance": tolerance, "unassigned_working_ids": []}
    no_loops = [{**patch, "loops": []} for patch in patches]
    assigned_patches = [{**patch, "source_ids": patch["working_ids"]} for patch in patches]
    assigned_parts = [{**part, "source_ids": part["working_ids"]} for part in parts]
    artifacts = {
        0: {"observation_ids": [str(i) for i in range(1, len(points) + 1)]},
        18: {**base, "patches": no_loops},
        19: {**base, "patches": no_loops, "parts": parts, "connections": connections},
        "20-boundary": {**base, "patches": patches, "parts": parts, "connections": connections},
        20: {**base, "patches": assigned_patches, "parts": assigned_parts, "connections": connections,
             "unassigned_source_ids": []},
    }
    artifacts.update(working_references(points))
    truth = {"patches": [patch["working_ids"] for patch in patches],
             "parts": [part["working_ids"] for part in parts]}
    return points, artifacts, truth


def u_fixtures(tolerance):
    outline = [(-6, -5), (6, -5), (6, 5), (2, 5), (2, -2), (-2, -2), (-2, 5), (-6, 5)]
    points, first, last = [], [], []
    depth = 6
    for z, cap, sign in ((0, first, -1), (depth, last, 1)):
        for i in range(-12, 13):
            for j in range(-10, 11):
                if -4 < i < 4 and j > -4:
                    continue
                points.append(((i / 2, j / 2, z), (0, 0, sign)))
                cap.append(str(len(points)))
    edges = list(zip(outline, outline[1:] + outline[:1]))
    for (ax, ay), (bx, by) in edges:
        count = 2 * (abs(bx - ax) + abs(by - ay))
        normal = (0 if by == ay else (1 if by > ay else -1),
                  0 if bx == ax else (-1 if bx > ax else 1), 0)
        for i in range(count):
            t = (i + .5) / count
            for j in range(12):
                points.append(((ax + (bx - ax) * t, ay + (by - ay) * t, (j + .5) / 2), normal))
    return projection_fixtures(points, first, last, outline, depth, tolerance)


def projection_fixtures(points, first, last, outline, depth, tolerance, evidence=(), cuts=()):
    edges = list(zip(outline, outline[1:] + outline[:1]))
    rows = [[str(i), normal, True] for i, (_, normal) in enumerate(points, 1)]
    axis = {"axis": [0, 0, 1], "origin": [0, 0, depth / 2], "source": "reference"}
    context = {"fit_points": rows, "scoring_points": rows, "tolerance": tolerance,
               "evidence": list(evidence), "center": axis["origin"]}
    axes = {**context, "axes": [axis]}
    interval = {**axes, "selected_axis": axis, "first": {"depth": -depth / 2, "ids": first},
                "last": {"depth": depth / 2, "ids": last}}
    # The projection frame for +Z uses -Y as x and +X as y.
    sketch = [{"id": "1", "name": "reference profile", "suppressed": False,
               "params": {"kind": 0, "plane": {"kind": 3, "values": [0, 0, 0, 0, 0, 1, 0, -1, 0]},
                          "entities": [{"id": str(i), "kind": 1, "values": [-ay, ax, -by, bx]}
                                       for i, ((ax, ay), (bx, by)) in enumerate(edges, 1)]}}]
    profiles = {**interval, "profiles": [sketch, sketch]}
    operation = {"sketch": sketch, "operation": 0, "depth": depth, "reverse": False, "support_ids": first}
    proposals = {**profiles, "proposals": [{"operation": operation, "allow_join": False}], "secondary_operations": list(cuts)}
    area = abs(sum(ax * by - bx * ay for (ax, ay), (bx, by) in edges)) / 2
    fit = {"volume_mm3": area * depth, "scored_points": len(points), "median_mm": 0, "p95_mm": 0,
           "inlier_fraction": 1, "unsupported_fraction": 0, "normal_error_deg": 0,
           "normal_comparisons": len(points), "score": .023, "residual_ids": []}
    if cuts:
        # Fixed deliberately insufficient base score for exercising the search gate.
        fit.update(p95_mm=1.0, inlier_fraction=.8, unsupported_fraction=.1, score=1.5)
    programs = {**proposals, "programs": [{"operations": [operation], "fit": fit}]}
    return points, {0: context, 22: axes, 23: interval, 24: profiles, 25: proposals, 26: programs}, {}


def blind_hole_fixtures(tolerance):
    depth, floor, radius = 8, 4, 1.5
    points, first, last, wall = [], [], [], []
    for i in range(25):
        for j in range(25):
            x, y, z = -4 + 8 * i / 24, -3 + 6 * i / 24, depth * j / 24
            points.extend([((x, -3, z), (0, -1, 0)), ((x, 3, z), (0, 1, 0)),
                           ((-4, y, z), (-1, 0, 0)), ((4, y, z), (1, 0, 0))])
            v = -3 + 6 * j / 24
            in_hole = math.hypot(x, v) < radius
            points.append(((x, v, 0), (0, 0, -1)))
            first.append(str(len(points)))
            points.append(((x, v, floor if in_hole else depth), (0, 0, 1)))
            if not in_hole:
                last.append(str(len(points)))
    for i in range(80):
        angle = 2 * math.pi * i / 80
        x, y = (0 if abs(value) < 1e-12 else value for value in (math.cos(angle), math.sin(angle)))
        for j in range(25):
            points.append(((radius * x, radius * y, floor + (depth - floor) * j / 24), (-x, -y, 0)))
            wall.append(str(len(points)))
    evidence = [{"type": "cylinder", "axis_point": [0, 0, 0], "axis": [0, 0, 1], "radius": radius,
                 "axial_min": floor, "axial_max": depth, "angular_coverage": 1, "support_ids": wall}]
    cut = {"sketch": [{"id": "1", "name": "reference blind hole", "suppressed": False,
                       "params": {"kind": 0, "plane": {"kind": 3,
                                  "values": [0, 0, depth + tolerance, 0, 0, 1, 0, -1, 0]},
                                  "entities": [{"id": "1", "kind": 3, "values": [0, 0, radius]}]}}],
           "operation": 2, "depth": depth - floor + tolerance, "reverse": True, "support_ids": wall}
    outline = [(-4, -3), (4, -3), (4, 3), (-4, 3)]
    raw, references, truth = projection_fixtures(points, first, last, outline, depth, tolerance, evidence, [cut])
    regions = []
    for axis_index, extent in enumerate(((-4, 4), (-3, 3), (0, depth))):
        for side, coordinate in enumerate(extent):
            sign = 1 if side else -1
            support = [i for i, (p, n) in enumerate(points)
                       if n[axis_index] == sign and p[axis_index] == coordinate]
            regions.append(planar_region(axis_index, sign, coordinate, support))
    regions.append(cylindrical_region((0, 0), radius, floor, depth, [int(i) - 1 for i in wall]))
    analysis = analytic_analysis(points, regions)
    references[0].update(analysis.pop(0))
    references.update(analysis)
    references[12] = {**references[7], "candidate": known_host([-4, -3, 0], [8, 6, depth])}
    base = references[25]["proposals"][0]["operation"]
    references[27] = {"selected_operations": [base, cut]}
    features = projection_plan_features(base, cut)
    plan = {"plan_features": features, "expected_revision": "0"}
    body = {"kind": "box_with_blind_hole", "origin": [-4, -3, 0], "size": [8, 6, depth],
            "hole_floor_center": [0, 0, floor], "hole_radius": radius, "tool_height": depth - floor + 1}
    model = {**plan, "features": features, "reference_body": body}
    references.update({28: plan, 29: plan, 30: model, 31: model, 32: {"reference_body": body}})
    return raw, references, truth


def projection_plan_features(base, cut):
    features = []
    for operation in (base, cut):
        sketch_id = len(features) + 1
        features.append({**operation["sketch"][0], "id": str(sketch_id), "reconstructed": True})
        params = {"kind": 1, "sketch": str(sketch_id), "values": [operation["depth"]],
                  "reverse": operation["reverse"], "operation": operation["operation"]}
        if sketch_id > 1:
            params["target_body"] = str(sketch_id - 1)
        features.append({"id": str(sketch_id + 1), "name": "reference extrusion",
                         "suppressed": False, "reconstructed": True, "params": params})
    return features


def surface_box_fixtures(spec):
    points, geometry, _ = box_fixtures(spec["boxes"], spec["options"]["tolerance"])
    count = len(points)
    patches = geometry[18]["patches"]
    raw = []
    for i, (position, normal) in enumerate(points):
        face_row, face_col = divmod(i % 256, 16)
        hide_normal = 4 <= face_row < 12 and 4 <= face_col < 12 and i % 3 == 0
        raw.append((position, (0, 0, 0) if hide_normal else normal))
    if spec.get("outlier", True):
        raw.append(((50, 50, 50), (0, 0, 1)))
    if spec["unit"] == "m":
        raw = [([x / 1000 for x in position], normal) for position, normal in raw]
    raw_rows = [[str(i), normal if any(normal) else None, True, None]
                for i, (_, normal) in enumerate(raw, 1)]
    # Keep a known, evenly spaced subset for direct inputs. It is defined
    # analytically so its membership does not depend on the voxel implementation.
    retained = []
    for face in range(count // 256):
        interior = [row * 16 + col for row in range(3, 13) for col in range(3, 13)]
        interior.sort(key=lambda index: (index * 71) % 257)
        # Give the three axes distinct support counts. Replenishing the top
        # face then keeps the same fitted frame while adding observations.
        removed = set(interior[:(70, 65, 55, 50, 35, 20)[face % 6]])
        retained.extend(face * 256 + i for i in range(256) if i not in removed)
    before_normals = [raw_rows[i] for i in retained]
    fitted = [[str(i + 1), points[i][1], True, 0] for i in retained]
    working = {row[0] for row in fitted}
    def plane(index, patch, support):
        normal = patch["normal"]
        return {"id": str(index), "type": 0, "axis": normal,
                "offset": -sum(a * b for a, b in zip(normal, patch["origin"])),
                "support_ids": support, "confidence": 1.0}

    surfaces, indexed = [], []
    for index, patch in enumerate(patches, 1):
        support = [pid for pid in patch["working_ids"] if pid in working]
        surfaces.append(plane(index, patch, support))
        indexed.append({"surface_id": index, "support_ids": support,
                        "lower": patch["lower"], "upper": patch["upper"], "centroid": patch["origin"],
                        "normal": patch["normal"], "oriented": True})
    relations = []
    for a, b in combinations(range(len(patches)), 2):
        alignment = abs(sum(x * y for x, y in zip(patches[a]["normal"], patches[b]["normal"])))
        relations.append([a + 1, b + 1, 1 if alignment else 2, 0, 1, True])
    stages = {
        0: {"observation_ids": [row[0] for row in raw_rows]},
        "0-active": {"observation_ids": [row[0] for row in raw_rows[:count]]},
        1: {"samples": raw_rows},
        2: {"samples": before_normals + raw_rows[count:]},
        3: {"samples": before_normals},
        4: {"samples": fitted, "template_ids": indexed[-1]["support_ids"]},
        6: {"samples": fitted, "evidence": surfaces},
        7: {"samples": fitted, "evidence": surfaces, "surface_patches": indexed, "surface_relations": []},
        8: {"samples": fitted, "evidence": surfaces, "surface_patches": indexed, "surface_relations": relations},
    }
    stages[9] = stages[8]
    cap = patches[-1]
    rows = [[pid, cap["normal"], True, 0] for pid in cap["working_ids"]]
    stages[10] = {**stages[9], "recovery": {
        "region_ids": indexed[-1]["support_ids"], "added_ids": [row[0] for row in rows if row[0] not in working],
        "samples": rows, "evidence": [plane(1, cap, cap["working_ids"])]}}
    return raw, stages, {}


def circle_direction(index, count, half_step=True):
    angle = 2 * math.pi * (index + (.5 if half_step else 0)) / count
    return tuple(0.0 if abs(value) < 1e-12 else value for value in (math.cos(angle), math.sin(angle)))


def cylinder_fixtures(missing_cap):
    radius, height = 3, 8
    points, regions = [], []
    for axial in range(16):
        for i in range(72):
            x, y = circle_direction(i, 72)
            points.append(((radius * x, radius * y, (axial + .5) * height / 16), (x, y, 0)))
    regions.append(cylindrical_region((0, 0), radius, .25, height - .25, list(range(len(points)))))
    for z, sign in [(0, -1), (height, 1)]:
        if missing_cap and sign == 1:
            continue
        start = len(points)
        for ring in range(12):
            r = (ring + .5) * radius / 12
            for i in range(72):
                x, y = circle_direction(i, 72)
                points.append(((r * x, r * y, z), (0, 0, sign)))
        regions.append(planar_region(2, sign, z, list(range(start, len(points)))))
    return points, analytic_analysis(points, regions), {}


def mixed_plate_fixtures():
    dimensions = (120, 80, 5)
    points, regions = [], []
    for axis in range(3):
        u, v = (axis + 1) % 3, (axis + 2) % 3
        for side in (0, 1):
            start = len(points)
            sign, coordinate = 2 * side - 1, side * dimensions[axis]
            normal = [0, 0, 0]
            normal[axis] = sign
            for i in range(dimensions[u]):
                for j in range(dimensions[v]):
                    p = [0, 0, 0]
                    p[axis], p[u], p[v] = coordinate, i + .5, j + .5
                    if math.hypot(p[0] - 85, p[1] - 45) < 7:
                        continue
                    if axis == 2 and side and math.hypot(p[0] - 30, p[1] - 30) < 8:
                        continue
                    points.append((p, normal))
            regions.append(planar_region(axis, sign, coordinate, list(range(start, len(points)))))
    for cx, cy, radius, base, height, sign in [(30, 30, 8, 5, 22, 1), (85, 45, 7, 0, 5, -1)]:
        start = len(points)
        for z in range(2 * height):
            for i in range(96):
                x, y = circle_direction(i, 96)
                points.append(((cx + radius * x, cy + radius * y, base + (z + .5) / 2), (sign * x, sign * y, 0)))
        regions.append(cylindrical_region((cx, cy), radius, base + .25, base + height - .25,
                                          list(range(start, len(points)))))
    start = len(points)
    for x in range(-8, 8):
        for y in range(-8, 8):
            if math.hypot(x + .5, y + .5) < 8:
                points.append(((30 + x + .5, 30 + y + .5, 27), (0, 0, 1)))
    regions.append(planar_region(2, 1, 27, list(range(start, len(points)))))
    references = analytic_analysis(points, regions)
    references[12] = {**references[7], "candidate": known_host([0, 0, 0], dimensions)}
    return points, references, {}


def materialize(spec, folder: Path, cloud_override: Path | None):
    if spec["recipe"] == "boxes_v1":
        points, artifacts, truth = box_fixtures(spec["boxes"], spec["options"]["tolerance"])
    elif spec["recipe"] == "u_channel_v1":
        points, artifacts, truth = u_fixtures(spec["options"]["tolerance"])
    elif spec["recipe"] == "blind_hole_v1":
        points, artifacts, truth = blind_hole_fixtures(spec["options"]["tolerance"])
    elif spec["recipe"] == "surface_box_v1":
        points, artifacts, truth = surface_box_fixtures(spec)
    elif spec["recipe"] == "cylinder_v1":
        points, artifacts, truth = cylinder_fixtures(spec.get("missing_cap", False))
    elif spec["recipe"] == "mixed_plate_v1":
        points, artifacts, truth = mixed_plate_fixtures()
    else:
        raise ValueError(f"Unknown dataset recipe: {spec['recipe']}")
    cloud = cloud_override or folder / "cloud.ply"
    if cloud_override is None:
        write_cloud(cloud, points)
    digest = sha256(cloud)
    if spec.get("sha256") != digest and cloud_override is None:
        raise ValueError(f"Dataset hash changed for {folder.name}: {digest}")
    metadata = {"schema": "paramcad.contract-stage", "version": 1,
                "source_sha256": digest, "source_unit": spec["unit"]}
    references = {key: {**data, **metadata, "module": int(str(key).split("-", 1)[0])}
                  for key, data in artifacts.items()}
    return cloud, digest, references, truth
