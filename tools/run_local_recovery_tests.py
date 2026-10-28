"""Mixed-board acceptance across noise, density, feature size and missing caps.

Ground-truth labels only generate inputs and validate outputs; the fitter receives
an unlabeled PLY and an independent STEP reference for post-fit accuracy checks.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import subprocess
import numpy as np
from generate_mixed_template_board import ROOT, definition, sample, write_ply
from generate_compound_test_clouds import ENGINE
from run_mixed_feature_fit_tests import parameter_errors
from run_performance_validation import hidden_process_options


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/local-recovery-validation')
    parser.add_argument('--variants', nargs='+', default=['exact','noise_004','noise_008','density_025','density_010','small_050','small_025','missing_cap'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for name in args.variants:
        case = copy.deepcopy(definition())
        if name.startswith('small_'):
            scale = {'small_050': 0.5, 'small_025': 0.25}[name]
            feature = next(f for f in case['features'] if f['id'] == 4)
            old = float(np.prod(feature['size']))
            feature['size'] = [v*scale for v in feature['size']]
            case['expected']['volume_mm3'] += float(np.prod(feature['size']))-old
        cloud, labels, _, _ = sample(case, 1.0)
        rng = np.random.default_rng(4500)
        sigma = {'noise_004': 0.04, 'noise_008': 0.08}.get(name, 0)
        if sigma:
            cloud[:, :3] += rng.normal(0, sigma, (len(cloud), 3))
        if name.startswith('density_'):
            fraction = {'density_025': 0.25, 'density_010': 0.1}[name]
            keep = (labels != 4) | (rng.random(len(cloud)) < fraction)
            cloud, labels = cloud[keep], labels[keep]
        if name == 'missing_cap':
            keep = ~((labels == 3) & (cloud[:, 5] > 0.9))
            cloud, labels = cloud[keep], labels[keep]
        target = args.output/name
        target.mkdir(exist_ok=True)
        case_path = target/'case.json'
        case_path.write_text(json.dumps(case, indent=2)+'\n', encoding='utf-8')
        write_ply(target/'cloud.ply', cloud, labels)
        (target/'reconstruction.json').write_text(json.dumps({'unit': 'mm', 'point_count': len(cloud)}), encoding='utf-8')
        with (target/'source.log').open('wb') as log:
            subprocess.run([str(ENGINE), 'generate', str(case_path), str(target/'source.step'), str(target/'source_report.json')], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120, **hidden_process_options())
        with (target/'fit.log').open('wb') as log:
            run = subprocess.run([str(ENGINE), 'fit', str(target/'cloud.ply'), str(target/'fitting_report.json'), str(target/'reconstructed.step'), str(target/'source.step'), str(case_path)], stdout=log, stderr=subprocess.STDOUT, timeout=240, **hidden_process_options())
        report = json.loads((target/'fitting_report.json').read_text(encoding='utf-8'))
        if name == 'missing_cap':
            diagnostics = report.get('local_analysis_failures', [])
            passed = (report.get('complete_cad_fit') is not True and any(f['reason'] == 'missing_end_face' and not f['repaired'] for f in diagnostics) and not (target/'reconstructed.step').exists())
            parameters = None
        else:
            parameters = parameter_errors(case, report)
            passed = run.returncode == 0 and report.get('complete_cad_fit') is True and parameters['passed'] and report.get('cad', {}).get('solids') == 1
        result = {'variant': name, 'passed': bool(passed), 'status': report.get('status'), 'parameters': parameters,
                  'local_failures': len(report.get('local_analysis_failures', [])), 'accuracy': report.get('accuracy')}
        results.append(result)
        print(json.dumps({'variant': name, 'passed': bool(passed), 'status': report.get('status'), 'maximum_parameter_error_mm': (parameters or {}).get('maximum_error_mm'), 'local_failures': result['local_failures']}), flush=True)
    (args.output/'summary.json').write_text(json.dumps({'passed': sum(r['passed'] for r in results), 'total': len(results), 'results': results}, indent=2)+'\n', encoding='utf-8')
    return 0 if all(r['passed'] for r in results) else 1

if __name__ == '__main__':
    raise SystemExit(main())
