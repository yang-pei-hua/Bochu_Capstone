"""Dimensioned closed curved primitives, sampled independently of the fitter."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[1]
KINDS = ("sphere", "cylinder", "cone", "torus", "frustum")


def definition(kind, rotated=False):
    axis = [0.36, -0.48, 0.8] if rotated else [0, 0, 1]
    p = {"center": [0, 0, 0], "axis": axis}
    geometry = "cone" if kind == "frustum" else kind
    if kind == "sphere":
        p["radius"] = 15
        bounds, volume = [30]*3, 4*math.pi*15**3/3
    elif kind == "torus":
        p.update(major_radius=25, minor_radius=6)
        bounds = [2*(25*math.sqrt(1-a*a)+6) for a in axis]
        volume = 2*math.pi**2*25*6**2
    else:
        p["height"] = 30
        if kind == "cylinder":
            p["radius"] = 12
            r0 = r1 = 12
            volume = math.pi*12**2*30
        else:
            r0, r1 = 12, (5 if kind == "frustum" else 0)
            p.update(base_radius=r0, top_radius=r1)
            volume = math.pi*30*(r0*r0+r0*r1+r1*r1)/3
        bounds = [max(-a*15+r0*math.sqrt(1-a*a), a*15+r1*math.sqrt(1-a*a)) -
                  min(-a*15-r0*math.sqrt(1-a*a), a*15-r1*math.sqrt(1-a*a)) for a in axis]
    expected = {"bounding_box_mm": bounds, "volume_mm3": volume, "solids": 1,
                "through_holes": 0, "hole_diameters_mm": [], "curved_kind": geometry}
    if kind == "sphere": expected["curved_dimensions_mm"] = {"radius_mm": 15}
    elif kind == "torus": expected["curved_dimensions_mm"] = {"major_radius_mm": 25, "minor_radius_mm": 6}
    elif kind == "cylinder": expected["curved_dimensions_mm"] = {"radius_mm": 12, "height_mm": 30}
    else: expected["curved_dimensions_mm"] = {"small_radius_mm": min(r0,r1), "large_radius_mm": max(r0,r1), "height_mm": 30}
    return {"id": kind + ("-rotated" if rotated else ""), "unit": "mm", "geometry": geometry,
            "parameters": p, "expected": expected,
            "acceptance": {"dimension_tolerance_mm": 0.2, "primitive_dimension_tolerance_mm": 0.1,
                           "hole_diameter_tolerance_mm": 0.2, "symmetric_difference_volume_fraction": 0.03}}


def samples(case, noise=0, incomplete=False, seed=4500):
    kind, p = case["geometry"], case["parameters"]
    axis = p["axis"]
    # Orthonormal transverse axes, independent of the project's C++ helper.
    ref = [0,0,1] if abs(axis[2]) < .9 else [1,0,0]
    cross = lambda a,b: [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    u = cross(axis,ref)
    norm = math.sqrt(sum(x*x for x in u)); u = [x/norm for x in u]
    v = cross(axis,u)
    rng = random.Random(seed)
    result = []
    def emit(point, normal):
        world = [sum(point[j]*basis[j][i] for j in range(3)) for i in range(3)]
        outward = [sum(normal[j]*basis[j][i] for j in range(3)) for i in range(3)]
        result.append([x+rng.gauss(0,noise) for x in world] +
                      [x+rng.gauss(0,noise*.01) for x in outward])
    basis = [u,v,axis]
    if kind == "sphere":
        for latitude in range(1,40):
            theta = math.pi*latitude/40
            for azimuth in range(128):
                phi = 2*math.pi*azimuth/128
                n = [math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)]
                if incomplete and n[0]<0: continue
                emit([p["radius"]*x for x in n],n)
        for sign in (-1,1): emit([0,0,sign*p["radius"]],[0,0,sign])
    elif kind == "torus":
        for major in range(128 if not incomplete else 64):
            phi = 2*math.pi*major/128
            for minor in range(48):
                theta = 2*math.pi*minor/48
                c,s,C,S = math.cos(phi),math.sin(phi),math.cos(theta),math.sin(theta)
                rho = p["major_radius"]+p["minor_radius"]*C
                emit([rho*c,rho*s,p["minor_radius"]*S],[C*c,C*s,S])
    else:
        h = p["height"]
        r0 = p["radius"] if kind == "cylinder" else p["base_radius"]
        r1 = p["radius"] if kind == "cylinder" else p["top_radius"]
        center_z = h/2 if kind == "cylinder" else h*(r0*r0+2*r0*r1+3*r1*r1)/(4*(r0*r0+r0*r1+r1*r1))
        slope = (r1-r0)/h
        for ring in range(65):
            t = h*ring/64; radius = r0+slope*t
            for azimuth in range(96):
                phi = 2*math.pi*azimuth/96; c,s = math.cos(phi),math.sin(phi)
                divisor = math.sqrt(1+slope*slope)
                emit([radius*c,radius*s,t-center_z],[c/divisor,s/divisor,-slope/divisor])
        if not incomplete:
            for t,radius,sign in ((0,r0,-1),(h,r1,1)):
                if not radius: continue
                for ring in range(1,11):
                    for azimuth in range(96):
                        phi=2*math.pi*azimuth/96; r=radius*(ring-.5)/10
                        emit([r*math.cos(phi),r*math.sin(phi),t-center_z],[0,0,sign])
    return result


def write_case(directory, case, noise=0, incomplete=False):
    directory = Path(directory); directory.mkdir(parents=True,exist_ok=True)
    (directory/"case.json").write_text(json.dumps(case,indent=2)+"\n",encoding="utf-8")
    points = samples(case,noise,incomplete)
    header = "ply\nformat ascii 1.0\nelement vertex " + str(len(points)) + "\n"
    header += "".join(f"property float {name}\n" for name in ("x","y","z","nx","ny","nz")) + "end_header\n"
    (directory/"cloud.ply").write_text(header+"".join(" ".join(map(str,p))+"\n" for p in points),encoding="ascii")
    (directory/"reconstruction.json").write_text('{"unit":"mm"}\n',encoding="utf-8")
    return len(points)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/curved-clouds")
    parser.add_argument("--noise",type=float,default=0)
    parser.add_argument("--rotated",action="store_true")
    parser.add_argument("--standard-definitions",action="store_true",help="Write four dimensioned source-case definitions")
    args=parser.parse_args()
    for kind in KINDS:
        if args.standard_definitions:
            if kind=="frustum":continue
            case=definition(kind); directory=ROOT/"data/performance-validation"/kind
            directory.mkdir(exist_ok=True)
            (directory/"case.json").write_text(json.dumps(case,indent=2)+"\n",encoding="utf-8")
            descriptions={"sphere":"球：半径 15 mm，直径 30 mm。",
                "cylinder":"独立圆柱：半径 12 mm，直径 24 mm，高度 30 mm，两个完整平面端面。",
                "cone":"完整圆锥：底面半径 12 mm，底面直径 24 mm，高度 30 mm，顶端半径 0。",
                "torus":"完整圆环体：大半径 25 mm，管半径 6 mm；外径 62 mm，内径 38 mm，轴向厚度 12 mm。"}
            placements={"sphere":"球心与质心位于原点。", "cylinder":"质心位于原点，轴沿 Z 方向，端面 z = ±15 mm。",
                "cone":"质心位于原点，轴沿 Z 方向；底面 z = -7.5 mm，顶点 z = 22.5 mm。",
                "torus":"圆环中心与质心位于原点，轴沿 Z 方向。"}
            (directory/"README.md").write_text(f"# {kind}\n\n{descriptions[kind]}\n\n"
                "单位为毫米。" + placements[kind] + "\n\n"
                "`case.json` 记录尺寸、实体数量和验收阈值，`model.step` 为独立构造的标准实体，"
                "`model_report.json` 记录有效性、解析体积和 STEP 回读检查。\n\n"
                "使用 `python tools/generate_standard_models.py --cases " + kind + "` 重新生成。"
                "运行 `python tools/run_curved_fit_tests.py --cases " + kind + "` 验证重建；"
                "生成的点云、重建 STEP、诊断和日志位于 `outputs/curved-fit-tests/`。\n",encoding="utf-8")
            print(json.dumps({"case":kind,"parameters":case["parameters"]}))
        else:
            case=definition(kind,args.rotated)
            print(json.dumps({"case":case["id"],"points":write_case(args.output/case["id"],case,args.noise)}))


if __name__=="__main__":main()
