"""Validate curved solids and reject incomplete surfaces; reads only JSON reports."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from generate_curved_test_clouds import ROOT,KINDS,definition,write_case
from generate_compound_test_clouds import ENGINE
from run_performance_validation import hidden_process_options


def run_case(directory, kind, noise=0, rotated=False, seed=4500, incomplete=False):
    directory=Path(directory); case=definition(kind,rotated)
    # A failed or unrecognized replay must not expose a previous accepted STEP
    # or diagnostic as this run's output. Keep previous artifacts reviewable.
    stamp=time.time_ns()
    for name in ("model.step","model_report.json","fitting_report.json","reconstructed.step",
                 "fitting_report.json.native-crash.json"):
        previous=directory/name
        if previous.is_file():
            previous.rename(previous.with_name(previous.stem+f".previous-{stamp}"+previous.suffix))
    count=write_case(directory,case,noise,incomplete)
    exit_code=0
    for stage,arguments in (("generate",["generate",directory/"case.json",directory/"model.step",directory/"model_report.json"]),
                            ("fit",["fit",directory/"cloud.ply",directory/"fitting_report.json",directory/"reconstructed.step",directory/"model.step",directory/"case.json"])):
        with (directory/(stage+".log")).open("wb") as log:
            try:
                process=subprocess.run(list(map(str,[ENGINE,*arguments])),stdout=log,stderr=subprocess.STDOUT,timeout=120,
                                       env={**os.environ,"PCAD_RANSAC_SEED":str(seed)},**hidden_process_options())
                exit_code=process.returncode
            except subprocess.TimeoutExpired:
                exit_code="timeout"
        if exit_code:
            break
    path=directory/"fitting_report.json"
    report=json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    curved=report.get("curved_solids",[])
    manifest={"schema":"paramcad.direct-cloud-validation","status":"completed" if exit_code==0 else "failed",
              "inputs":{"model":{"snapshot":"model.step"},"expected":{"snapshot":"case.json"}},
              "artifacts":{"point_cloud":"cloud.ply"},"executables":{"fit":str(ENGINE)},"fitting":report}
    (directory/"validation_report.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    (directory/"pipeline_status.json").write_text(json.dumps({"status":manifest["status"],"stage":"reporting",
        "cad_status":report.get("status"),"cad_passed":report.get("complete_cad_fit",False)})+"\n",encoding="utf-8")
    result={"case":case["id"],"noise_mm":noise,"seed":seed,"incomplete":incomplete,"input_points":count,
            "exit_code":exit_code,"status":report.get("status"),"stage":report.get("stage"),"curved_solids":curved,
            "accuracy":report.get("accuracy"),"report":str(path)}
    result["passed"]=(exit_code==0 and (report.get("status")=="cad_not_recognized" if incomplete else
        report.get("complete_cad_fit") is True and len(curved)==1 and curved[0]["kind"]==case["geometry"]))
    if not result["passed"]:
        result["error"]=report.get("error") or report.get("recognition")
        crash=path.with_name(path.name+".native-crash.json")
        if crash.is_file():result["native_crash"]=json.loads(crash.read_text())
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/curved-fit-tests")
    parser.add_argument("--noise",type=float,default=0)
    parser.add_argument("--rotated",action="store_true")
    parser.add_argument("--seed",type=int,default=4500)
    parser.add_argument("--seed-sweep",type=int,default=1,help="Number of consecutive random seeds")
    parser.add_argument("--incomplete",action="store_true")
    parser.add_argument("--cases",nargs="+",choices=KINDS,default=list(KINDS))
    args=parser.parse_args()
    if not 1<=args.seed_sweep<=30 or not 0<=args.seed<=0xFFFFFFFF-args.seed_sweep+1 or args.noise<0:parser.error("Invalid seed, sweep or noise")
    results=[]
    for seed in range(args.seed,args.seed+args.seed_sweep):
        for kind in args.cases:
            result=run_case(args.output/f"{kind}-seed-{seed}",kind,args.noise,args.rotated,seed,args.incomplete)
            results.append(result)
            diagnostic={key:result[key] for key in ("case","noise_mm","seed","incomplete","passed","status")}
            accuracy=result.get("accuracy") or {}
            diagnostic["primitive_errors_mm"]=accuracy.get("primitive_dimension_errors_mm")
            diagnostic["symmetric_difference_fraction"]=accuracy.get("symmetric_difference_volume_fraction")
            if not result["passed"]: diagnostic["failure"]={key:result.get(key) for key in ("error","stage","exit_code","native_crash")}
            print(json.dumps(diagnostic,separators=(",",":")),flush=True)
    summary={"schema":"paramcad.curved-solid-validation","passed":sum(r["passed"] for r in results),"total":len(results),"results":results}
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"passed":summary["passed"],"total":summary["total"],"report":str(args.output/"summary.json")}))
    return 0 if summary["passed"]==summary["total"] else 2


if __name__=="__main__":sys.exit(main())
