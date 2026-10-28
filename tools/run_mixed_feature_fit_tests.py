"""Validate shared-analysis mixed reconstruction against independent source CAD.

Checks every source box's pose/dimensions, curved bosses, square cut, source CAD
boolean/sampling accuracy, and one-solid STEP round-trip. No truth labels enter
the recognizer. Runtime outputs are kept under outputs/.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np

from generate_mixed_template_board import ROOT, solids
from generate_compound_test_clouds import ENGINE
from run_performance_validation import hidden_process_options


def parameter_errors(case, report):
    boxes, rounds, _, _, transforms = solids(case)
    actual = report.get('boxes', [])
    remaining = list(range(len(actual)))
    errors = []
    for feature, origin, size in boxes:
        # Cover solvers may extend an upper box into already present material;
        # maximal overlapping boxes encode the same exact union as stacked ones.
        origin, size = list(origin), list(size)
        for owner, low, dimensions in boxes:
            if owner != feature or low[2] >= origin[2]: continue
            if (all(low[k] <= origin[k] and low[k]+dimensions[k] >= origin[k]+size[k] for k in (0,1))
                    and abs(low[2]+dimensions[2]-origin[2]) < 1e-8):
                size[2] += origin[2]-low[2];origin[2] = low[2]
        pivot, rotation = transforms[feature]
        center = (np.array(origin)+np.array(size)/2-pivot) @ rotation.T+pivot
        best = None
        for i in remaining:
            b = actual[i]; dims = np.array(b['dimensions_mm'])
            x, z = np.array(b['x_direction']), np.array(b['z_direction'])
            y = np.cross(z, x)
            c = np.array(b['origin_mm'])+(x*dims[0]+y*dims[1]+z*dims[2])/2
            position = float(np.max(np.abs(c-center)))
            dimension = float(np.max(np.abs(np.sort(dims)-np.sort(size))))
            score = max(position, dimension)
            if best is None or score < best[0]: best = (score, i, position, dimension)
        if best is None: return {'passed': False, 'reason': 'Missing source box'}
        remaining.remove(best[1])
        errors.append({'feature_id': feature, 'position_mm': best[2], 'dimension_mm': best[3]})
    curved_errors = []
    for feature, origin, r0, r1, height in rounds:
        matches = [c for c in report.get('curved_solids', []) if c['kind'] == ('cylinder' if r0 == r1 else 'cone')]
        if len(matches) != 1: return {'passed': False, 'reason': 'Missing or ambiguous curved boss'}
        c = matches[0]
        radial = abs(c['radius_mm']-r0) if r0 == r1 else max(abs(c['base_radius_mm']-r0), abs(c['top_radius_mm']-r1))
        curved_errors.append({'feature_id': feature, 'radius_mm': radial, 'height_mm': abs(c['height_mm']-height),
                              'position_mm': float(np.max(np.abs(np.array(c['base_center_mm'])-origin)))})
    cuts = [p for p in report.get('prisms', []) if p['subtract']]
    square_error = math.inf
    if len(cuts) == 1:
        p = cuts[0]; vertices = np.array(p['polygon_mm'])
        edges = np.linalg.norm(np.roll(vertices, -1, axis=0)-vertices, axis=1)
        square_error = max(float(np.max(np.abs(edges-44))), abs(p['depth_mm']-8))
    maximum = max([square_error] + [max(e['position_mm'],e['dimension_mm']) for e in errors] +
                  [max(e['radius_mm'],e['height_mm'],e['position_mm']) for e in curved_errors])
    return {'passed': not remaining and maximum <= 0.5, 'maximum_error_mm': maximum,
            'boxes': errors, 'curved_bosses': curved_errors, 'square_hole_dimension_error_mm': square_error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/shared-surface-mixed-validation')
    parser.add_argument('--seed', type=int, default=4500)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    generated = args.output/'input'
    subprocess.run([sys.executable, str(ROOT/'tools/generate_mixed_template_board.py'),
                    '--output', str(generated), '--seed', str(args.seed)], check=True,
                   stdout=subprocess.DEVNULL, **hidden_process_options())
    definition = generated/'case.json'
    case = json.loads(definition.read_text(encoding='utf-8'))
    source = args.output/'source.step'
    with (args.output/'source.log').open('wb') as log:
        subprocess.run([str(ENGINE), 'generate', str(definition), str(source), str(args.output/'source_report.json')],
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120, **hidden_process_options())
    results = []
    for variant in ('exact', 'noisy'):
        target = args.output/variant; target.mkdir(exist_ok=True)
        with (target/'fit.log').open('wb') as log:
            process = subprocess.run([str(ENGINE), 'fit', str(generated/variant/'cloud.ply'),
                str(target/'fitting_report.json'), str(target/'reconstructed.step'), str(source), str(definition)],
                stdout=log, stderr=subprocess.STDOUT, timeout=180, **hidden_process_options())
        report = json.loads((target/'fitting_report.json').read_text(encoding='utf-8'))
        parameters = parameter_errors(case, report)
        (target/'parameter_validation.json').write_text(json.dumps(parameters, indent=2)+'\n', encoding='utf-8')
        counts = report.get('compound_recognition', {})
        passed = (process.returncode == 0 and report.get('complete_cad_fit') is True and parameters['passed'] and
                  all(counts.get(k) == case['expected'][k] for k in ('boxes','prisms','curved_solids')) and
                  report.get('cad', {}).get('solids') == 1)
        result = {'variant': variant, 'passed': passed, 'status': report['status'],
                  'parameter_check': parameters, 'accuracy': report.get('accuracy'),
                  'reason': report.get('recognition', {}).get('reason', report.get('error'))}
        results.append(result)
        print(json.dumps({'variant': variant, 'passed': passed, 'status': report['status'],
                          'maximum_parameter_error_mm': parameters.get('maximum_error_mm'),
                          'symmetric_difference': report.get('accuracy', {}).get('symmetric_difference_volume_fraction'),
                          'reason': result['reason']}), flush=True)
    (args.output/'summary.json').write_text(json.dumps({'passed': sum(r['passed'] for r in results),
        'total': len(results), 'results': results}, indent=2)+'\n', encoding='utf-8')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
