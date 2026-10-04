"""Run analytic compound clouds through the production fitter and STEP validation.

Print bounded JSON diagnostics; process output stays in per-case logs.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from generate_compound_test_clouds import ENGINE, ROOT, generate


def run_suite(directory, noise=0.02, executable=ENGINE):
    directory=Path(directory)
    generated=generate(directory,noise)
    results=[]
    for sample in generated:
        case=directory/sample['case']
        expected=json.loads((case/'case.json').read_text())['expected']
        result={'case':sample['case'],'noise_mm':noise,'expected_boxes':expected['boxes'],'expected_prisms':expected['prisms']}
        started=time.monotonic()
        for stage,arguments in (
            ('generate',['generate',case/'case.json',case/'model.step',case/'model_report.json']),
            ('fit',['fit',case/'cloud.ply',case/'fitting_report.json',case/'reconstructed.step',case/'model.step',case/'case.json']),
        ):
            with (case/(stage+'.log')).open('wb') as log:
                try:
                    process=subprocess.run([str(executable),*map(str,arguments)],stdout=log,stderr=subprocess.STDOUT,timeout=120)
                    result[stage+'_exit_code']=process.returncode
                except subprocess.TimeoutExpired:
                    result[stage+'_exit_code']='timeout'
            if result[stage+'_exit_code']!=0:break
        report_path=case/'fitting_report.json'
        report=json.loads(report_path.read_text()) if report_path.is_file() else {}
        compound=report.get('compound_recognition',{})
        result.update({'status':report.get('status','not_run'),'stage':report.get('stage'),
            'boxes':compound.get('boxes'),'prisms':compound.get('prisms'),
            'minimum_box_count_proven':compound.get('minimum_box_count_proven'),
            'occupancy_error':compound.get('occupancy_error'),'fallback_reason':compound.get('fallback_reason'),
            'surface_coverage':compound.get('surface_coverage'),
            'accuracy':report.get('accuracy'),'error':report.get('error'),
            'elapsed_seconds':round(time.monotonic()-started,3)})
        result['passed']=(result.get('fit_exit_code')==0 and report.get('complete_cad_fit') is True
            and compound.get('recognized') is True and compound.get('boxes')==expected['boxes']
            and compound.get('prisms')==expected['prisms'] and compound.get('minimum_box_count_proven') is True)
        results.append(result)
        diagnostic={key:result[key] for key in ('case','passed','boxes','prisms','surface_coverage')}
        diagnostic['symmetric_difference_fraction']=result.get('accuracy',{}).get('symmetric_difference_volume_fraction') if result.get('accuracy') else None
        if not result['passed']:diagnostic['failure']=result
        print(json.dumps(diagnostic,ensure_ascii=False),flush=True)
    summary={'schema':'paramcad.compound-fit-suite','noise_mm':noise,'cases':results,
             'passed':sum(r['passed'] for r in results),'total':len(results)}
    (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/compound-fit-tests')
    parser.add_argument('--noise',type=float,default=0.02)
    args=parser.parse_args()
    summary=run_suite(args.output,args.noise)
    print(json.dumps({k:summary[k] for k in ('passed','total')}))
    return 0 if summary['passed']==summary['total'] else 1

if __name__=='__main__':raise SystemExit(main())
