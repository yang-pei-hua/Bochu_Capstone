"""Generate a spacious, closed mixed-feature board without CAD/recognizer dependencies.

Requires numpy and matplotlib. Coordinates and outward normals are analytic;
union/intersection probes remove all internal contact faces. No hidden caps are
invented beneath bosses. Ground-truth labels are separate from fitting inputs.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COLORS = np.array([[170, 180, 195], [65, 160, 220], [235, 155, 50],
                   [65, 185, 140], [180, 110, 220], [230, 100, 110],
                   [100, 155, 235], [220, 185, 65], [110, 195, 190]], dtype=np.uint8)
NAMES = ['base', 'round_through_hole', 'square_through_hole', 'cylinder_boss',
         'cube', 'l_bracket_gusset', 'rectangular_boss', 'frustum_boss', 'two_step_boss']


def definition(seed=4500):
    source = ROOT / 'data/performance-validation/l-bracket-gusset/case.json'
    bracket = json.loads(source.read_text(encoding='utf-8'))['parameters']
    case = {'id': 'mixed-template-board', 'unit': 'mm', 'base_size': [600, 360, 8],
            'features': [
                {'id': 1, 'kind': 'round_through_hole', 'center_xy': [78, 112], 'radius': 22},
                {'id': 2, 'kind': 'square_through_hole', 'origin_xy': [203, 55], 'size_xy': [44, 44]},
                {'id': 3, 'kind': 'cylinder_boss', 'center_xy': [355, 145], 'radius': 24, 'height': 92},
                {'id': 4, 'kind': 'cube', 'origin_xy': [485, 78], 'size': [32, 32, 32]},
                {'id': 5, 'kind': 'l_bracket_gusset', 'origin_xy': [75, 220], 'parameters': bracket,
                 'source': 'data/performance-validation/l-bracket-gusset/case.json'},
                {'id': 6, 'kind': 'rectangular_boss', 'origin_xy': [220, 260], 'size': [85, 38, 18]},
                {'id': 7, 'kind': 'frustum_boss', 'center_xy': [410, 285],
                 'base_radius': 28, 'top_radius': 14, 'height': 43},
                {'id': 8, 'kind': 'two_step_boss', 'origin_xy': [495, 225],
                 'size_xy': [70, 60], 'lower_height': 17, 'upper_width': 35, 'upper_height': 38},
            ]}
    rng = np.random.default_rng(seed)
    for f in case['features']:
        key = 'center_xy' if 'center_xy' in f else 'origin_xy'
        f[key] = np.round(np.array(f[key])+rng.uniform(-10, 10, 2), 3).tolist()
        if 'origin_xy' in f:
            sign = -1 if f['id'] in (4, 6, 8) else 1
            f['yaw_degrees'] = round(sign*float(rng.uniform(15, 36)), 3)
    case['geometry'] = 'mixed_template_board'
    boxes, rounds, ribs, cuts, _ = solids(case)
    volume = sum(float(np.prod(size)) for _, _, size in boxes)
    volume += sum(math.pi*h*(r0*r0+r0*r1+r1*r1)/3 for _, _, r0, r1, h in rounds)
    volume += sum(depth*run*rise/2 for _, _, depth, run, rise in ribs)
    for f in cuts:
        area = math.pi*f['radius']**2 if 'radius' in f else math.prod(f['size_xy'])
        volume -= area*case['base_size'][2]
    case['expected'] = {'volume_mm3': volume, 'solids': 1, 'bounding_box_mm': [600, 360, 100],
                        'through_holes': 1, 'hole_diameters_mm': [44],
                        'boxes': 7, 'prisms': 2, 'curved_solids': 2}
    case['acceptance'] = {'dimension_tolerance_mm': 0.5, 'hole_diameter_tolerance_mm': 0.25,
                          'symmetric_difference_volume_fraction': 0.02}
    return case


def frames(case):
    result = {0: (np.zeros(3), np.eye(3))}
    for f in case['features']:
        a = math.radians(f.get('yaw_degrees', 0))
        c, s = math.cos(a), math.sin(a)
        result[f['id']] = (np.array([*f.get('origin_xy', f.get('center_xy')), 0]),
                           np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))
    return result


def footprint(f):
    if 'center_xy' in f:
        center = np.array(f['center_xy'])
        radius = f.get('radius', f.get('base_radius'))
        return center-radius, center+radius
    if f['kind'] == 'l_bracket_gusset':
        size = [f['parameters']['width'], f['parameters']['base_depth']]
    else:
        size = f.get('size_xy', f.get('size'))[:2]
    a = math.radians(f.get('yaw_degrees', 0)); c, s = math.cos(a), math.sin(a)
    corners = np.array([[0, 0], [size[0], 0], size, [0, size[1]]])
    corners = corners @ np.array([[c, s], [-s, c]]) + f['origin_xy']
    return corners.min(axis=0), corners.max(axis=0)


def solids(case):
    """Components carry semantic feature IDs; the L-bracket follows its source parameters."""
    boxes = [(0, [0, 0, 0], case['base_size'])]
    rounds, ribs, cuts = [], [], []
    z = case['base_size'][2]
    for f in case['features']:
        i, kind = f['id'], f['kind']
        if 'through_hole' in kind:
            cuts.append(f)
        elif kind in ('cube', 'rectangular_boss'):
            boxes.append((i, [*f['origin_xy'], z], f['size']))
        elif kind in ('cylinder_boss', 'frustum_boss'):
            rounds.append((i, [*f['center_xy'], z], f.get('radius', f.get('base_radius')),
                           f.get('radius', f.get('top_radius')), f['height']))
        elif kind == 'two_step_boss':
            x, y = f['origin_xy']; w, d = f['size_xy']; h = f['lower_height']
            boxes.extend([(i, [x, y, z], [w, d, h]),
                          (i, [x, y, z+h], [f['upper_width'], d, f['upper_height']])])
        elif kind == 'l_bracket_gusset':
            x, y = f['origin_xy']; p = f['parameters']
            w, d, t = p['width'], p['base_depth'], p['base_thickness']
            u, h = p['upright_thickness'], p['upright_height']
            boxes.extend([(i, [x, y, z], [w, d, t]), (i, [x, y, z+t], [w, u, h])])
            ribs.append((i, [x+(w-p['rib_thickness'])/2, y+u, z+t],
                         p['rib_thickness'], p['rib_run'], p['rib_rise']))
    return boxes, rounds, ribs, cuts, frames(case)


def contains(points, parts):
    boxes, rounds, ribs, cuts, transforms = parts
    def local(feature):
        pivot, rotation = transforms[feature]
        return (points-pivot) @ rotation + pivot
    inside = np.zeros(len(points), dtype=bool)
    for feature, origin, size in boxes:
        q = local(feature) - origin
        inside |= np.all((q >= 0) & (q <= size), axis=1)
    for feature, origin, r0, r1, h in rounds:
        q = local(feature) - origin
        inside |= (q[:, 2] >= 0) & (q[:, 2] <= h) & (
            q[:, 0]**2 + q[:, 1]**2 <= (r0 + (r1-r0)*q[:, 2]/h)**2)
    for feature, origin, depth, run, rise in ribs:
        q = local(feature) - origin
        inside |= ((q[:, 0] >= 0) & (q[:, 0] <= depth) & (q[:, 1] >= 0) &
                   (q[:, 2] >= 0) & (q[:, 1]/run + q[:, 2]/rise <= 1))
    for f in cuts:
        if f['kind'] == 'round_through_hole':
            q = points[:, :2] - f['center_xy']
            void = np.sum(q*q, axis=1) <= f['radius']**2
        else:
            q = local(f['id'])[:, :2] - f['origin_xy']
            void = np.all((q >= 0) & (q <= f['size_xy']), axis=1)
        inside &= ~void
    return inside


def midpoints(length, spacing):
    n = max(1, math.ceil(length / spacing))
    return (np.arange(n) + 0.5)*length/n


def sample(case, spacing):
    parts = solids(case)
    blocks, labels, patches, patch_names = [], [], [], []

    def emit(points, normals, feature, name):
        points = np.asarray(points, dtype=float)
        normals = np.broadcast_to(normals, points.shape).copy()
        pivot, rotation = parts[4][feature]
        points = (points-pivot) @ rotation.T + pivot
        normals = normals @ rotation.T
        keep = contains(points - normals*1e-4, parts) & ~contains(points + normals*1e-4, parts)
        patch = len(patch_names)
        patch_names.append({'id': patch, 'feature_id': feature, 'name': name})
        if keep.any():
            blocks.append(np.column_stack((points[keep], normals[keep])))
            labels.append(np.full(keep.sum(), feature, dtype=np.uint8))
            patches.append(np.full(keep.sum(), patch, dtype=np.uint16))

    def rectangle(origin, u, v, normal, feature, name):
        u, v = np.asarray(u), np.asarray(v)
        a = midpoints(np.linalg.norm(u), spacing)/np.linalg.norm(u)
        b = midpoints(np.linalg.norm(v), spacing)/np.linalg.norm(v)
        aa, bb = np.meshgrid(a, b, indexing='ij')
        emit(np.asarray(origin) + aa.ravel()[:, None]*u + bb.ravel()[:, None]*v,
             normal, feature, name)

    for feature, origin, size in parts[0]:
        for a in range(3):
            u, v = [k for k in range(3) if k != a]
            du, dv = np.zeros(3), np.zeros(3)
            du[u], dv[v] = size[u], size[v]
            for side in (0, 1):
                o = np.array(origin, dtype=float); o[a] += side*size[a]
                normal = np.zeros(3); normal[a] = 2*side-1
                rectangle(o, du, dv, normal, feature, f'box_axis{a}_side{side}')

    def rotational(feature, origin, r0, r1, h, hole=False):
        na = max(16, math.ceil(2*math.pi*max(r0, r1)/spacing))
        angle = (np.arange(na)+0.5)*2*math.pi/na
        zz, aa = np.meshgrid(midpoints(h, spacing), angle, indexing='ij')
        zz, aa = zz.ravel(), aa.ravel()
        r = r0 + (r1-r0)*zz/h
        p = np.column_stack((r*np.cos(aa), r*np.sin(aa), zz)) + origin
        n = np.column_stack((np.cos(aa), np.sin(aa), np.full(len(aa), (r0-r1)/h)))
        n /= np.linalg.norm(n, axis=1)[:, None]
        emit(p, -n if hole else n, feature, 'inner_cylinder_wall' if hole else 'curved_side')
        if not hole:
            for radius, height, sign in ((r0, 0, -1), (r1, h, 1)):
                coords = midpoints(2*radius, spacing)-radius
                xx, yy = np.meshgrid(coords, coords, indexing='ij')
                keep = xx*xx+yy*yy < radius*radius
                p = np.column_stack((xx[keep], yy[keep], np.full(keep.sum(), height)))+origin
                emit(p, [0, 0, sign], feature, f'cap_{sign}')

    for f, o, r0, r1, h in parts[1]:
        rotational(f, o, r0, r1, h)
    for feature, origin, depth, run, rise in parts[2]:
        o = np.asarray(origin)
        yy, zz = np.meshgrid(midpoints(run, spacing), midpoints(rise, spacing), indexing='ij')
        keep = yy/run + zz/rise < 1
        for side in (0, 1):
            emit(o + np.column_stack((np.full(keep.sum(), side*depth), yy[keep], zz[keep])),
                 [2*side-1, 0, 0], feature, f'gusset_triangle_{side}')
        vertices = [o, o+[0, run, 0], o+[0, 0, rise]]
        for j in range(3):
            a, b = vertices[j], vertices[(j+1) % 3]
            tangent = b-a
            n = np.array([0, tangent[2], -tangent[1]])
            n /= np.linalg.norm(n)
            rectangle(a, [depth, 0, 0], tangent, n, feature, f'gusset_edge_{j}')
    for f in parts[3]:
        h = case['base_size'][2]
        if f['kind'] == 'round_through_hole':
            rotational(f['id'], [*f['center_xy'], 0], f['radius'], f['radius'], h, True)
        else:
            x, y = f['origin_xy']; w, d = f['size_xy']
            for o, u, n in [([x, y, 0], [w, 0, 0], [0, 1, 0]),
                            ([x, y+d, 0], [w, 0, 0], [0, -1, 0]),
                            ([x, y, 0], [0, d, 0], [1, 0, 0]),
                            ([x+w, y, 0], [0, d, 0], [-1, 0, 0])]:
                rectangle(o, u, [0, 0, h], n, f['id'], 'square_hole_wall')
    return np.concatenate(blocks), np.concatenate(labels), np.concatenate(patches), patch_names


def write_ply(path, points, labels):
    dtype = np.dtype([(k, '<f4') for k in ('x', 'y', 'z', 'nx', 'ny', 'nz')] +
                     [(k, 'u1') for k in ('red', 'green', 'blue')])
    data = np.empty(len(points), dtype=dtype)
    for i, k in enumerate(('x', 'y', 'z', 'nx', 'ny', 'nz')):
        data[k] = points[:, i]
    for i, k in enumerate(('red', 'green', 'blue')):
        data[k] = COLORS[labels, i]
    header = ('ply\nformat binary_little_endian 1.0\ncomment unit mm\n'
              f'element vertex {len(data)}\n' +
              ''.join(f'property float {k}\n' for k in dtype.names[:6]) +
              ''.join(f'property uchar {k}\n' for k in dtype.names[6:]) + 'end_header\n')
    with path.open('wb') as out:
        out.write(header.encode('ascii')); data.tofile(out)
    # Read back on-disk bytes rather than only checking the pre-write arrays.
    with path.open('rb') as inp:
        while inp.readline().strip() != b'end_header':
            pass
        actual = np.fromfile(inp, dtype=dtype)
    assert len(actual) == len(points)
    for i, k in enumerate(dtype.names[:6]):
        assert np.array_equal(actual[k], points[:, i].astype('<f4'))


def validate(case, cloud, labels):
    assert np.isfinite(cloud).all()
    assert np.allclose(np.linalg.norm(cloud[:, 3:], axis=1), 1)
    assert set(labels.tolist()) == set(range(9))
    assert len(np.unique(cloud[:, :3], axis=0)) == len(cloud)
    # Every sample must have material on its inward side and void outward.
    parts = solids(case)
    assert contains(cloud[:, :3]-cloud[:, 3:]*1e-4, parts).all()
    assert not contains(cloud[:, :3]+cloud[:, 3:]*1e-4, parts).any()
    footprints = []
    for f in case['features']:
        lo, hi = footprint(f)
        assert np.all(lo > 0) and np.all(hi < case['base_size'][:2])
        footprints.append((lo, hi))
    gaps = []
    for i, (lo, hi) in enumerate(footprints):
        for lo2, hi2 in footprints[i+1:]:
            sep = np.maximum(np.maximum(lo2-hi, lo-hi2), 0)
            assert np.any(sep > 0), 'Overlapping feature footprints'
            gaps.append(float(np.linalg.norm(sep)))
    # Hole centers must remain void throughout the plate thickness.
    for f in parts[3]:
        if 'center_xy' in f:
            xy = f['center_xy']
        else:
            pivot, rotation = parts[4][f['id']]
            center = np.array([*(np.array(f['origin_xy'])+np.array(f['size_xy'])/2), 0])
            xy = ((center-pivot) @ rotation.T+pivot)[:2]
        assert not contains(np.array([[*xy, z] for z in (0.1, 4, 7.9)]), parts).any()
    return {'finite': True, 'unit_outward_normals': True, 'no_duplicate_positions': True,
            'all_samples_on_exterior_boundary': True, 'feature_footprints_disjoint': True,
            'minimum_feature_footprint_gap_mm': min(gaps), 'through_hole_centers_empty': True}


def preview(directory, cloud, labels, case):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, Polygon
    fig = plt.figure(figsize=(15, 7), facecolor='white')
    ax = fig.add_subplot(121, projection='3d')
    rng = np.random.default_rng(42)
    # Keep all feature surfaces dense; reduce the visually dominant base only.
    selected = np.concatenate([rng.choice(np.flatnonzero(labels == k),
                              min(16000, np.count_nonzero(labels == k)), replace=False) for k in range(9)])
    p = cloud[selected, :3]
    ax.scatter(*p.T, c=COLORS[labels[selected]]/255, s=0.45, linewidths=0, rasterized=True)
    ax.set(xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)', title='Exterior surface point cloud')
    ax.set_box_aspect((600, 360, 160)); ax.view_init(elev=38, azim=55)
    top = fig.add_subplot(122)
    top.add_patch(Rectangle((0, 0), 600, 360, facecolor='#e5e9ef', edgecolor='#7d8796'))
    for f in case['features']:
        i = f['id']; lo, hi = footprint(f)
        color = 'white' if i in (1, 2) else COLORS[i]/255
        if 'center_xy' in f:
            patch = Circle(f['center_xy'], f.get('radius', f.get('base_radius')),
                           facecolor=color, edgecolor='#344054')
        else:
            if i == 5:
                w, h = f['parameters']['width'], f['parameters']['base_depth']
            else:
                w, h = f.get('size_xy', f.get('size'))[:2]
            pivot, rotation = frames(case)[i]
            corners = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]])
            corners = corners @ rotation.T + pivot
            patch = Polygon(corners[:, :2], facecolor=color, edgecolor='#344054')
        top.add_patch(patch)
        top.text((lo[0]+hi[0])/2, lo[1]-14,
                 f'{i}: '+NAMES[i].replace('_', ' '), ha='center', va='top', fontsize=7)
    top.set(xlim=(-15, 615), ylim=(-20, 380), xlabel='X (mm)', ylabel='Y (mm)',
            title='Top layout: all additions on Z = 8 mm')
    top.set_aspect('equal'); top.grid(alpha=0.2)
    fig.suptitle('Mixed-template board | 600 x 360 x 8 mm | exact source', fontsize=14)
    fig.tight_layout(); fig.savefig(directory/'preview.png', dpi=180); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/mixed-template-board')
    parser.add_argument('--spacing', type=float, default=1.0)
    parser.add_argument('--noise', type=float, default=0.02)
    parser.add_argument('--seed', type=int, default=4500)
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing <= 0:
        parser.error('--spacing must be finite and positive')
    if not math.isfinite(args.noise) or args.noise < 0:
        parser.error('--noise must be finite and nonnegative')
    case = definition(args.seed)
    cloud, labels, patches, patch_names = sample(case, args.spacing)
    checks = validate(case, cloud, labels)
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(len(cloud))
    cloud, labels, patches = cloud[order], labels[order], patches[order]
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'point_count': len(cloud), 'spacing_mm': args.spacing, 'seed': args.seed,
              'exact_bounds_mm': [cloud[:, :3].min(axis=0).tolist(), cloud[:, :3].max(axis=0).tolist()],
              'feature_point_counts': {name: int(np.count_nonzero(labels == i)) for i, name in enumerate(NAMES)},
              'validation': checks, 'files': []}
    for variant, sigma in [('exact', 0.0), ('noisy', args.noise)]:
        target = args.output/variant; target.mkdir(exist_ok=True)
        values = cloud.copy()
        if sigma:
            values[:, :3] += rng.normal(0, sigma, (len(cloud), 3))
        write_ply(target/'cloud.ply', values, labels)
        (target/'reconstruction.json').write_text(json.dumps({
            'unit': 'mm', 'source': 'analytic_mixed_template_board', 'point_count': len(values),
            'position_noise_sigma_mm': sigma, 'normals': 'analytic_outward_no_noise',
            'coordinate_system': 'right_handed_z_up'}, indent=2)+'\n', encoding='utf-8')
        np.savez_compressed(target/'ground_truth.npz', feature_id=labels, surface_patch_id=patches)
        report['files'].append(str(target/'cloud.ply'))
    (args.output/'case.json').write_text(json.dumps(case, indent=2)+'\n', encoding='utf-8')
    (args.output/'surface_patches.json').write_text(json.dumps(patch_names, indent=2)+'\n', encoding='utf-8')
    (args.output/'generation_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    readme = ROOT/'data/performance-validation/mixed-template-board/README.md'
    (args.output/'README.md').write_text(readme.read_text(encoding='utf-8'), encoding='utf-8')
    preview(args.output, cloud, labels, case)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
