"""Deterministic analytic surface clouds for box-count and gusset regression.

No image capture or COLMAP. Input surfaces are sampled independently of the
fitter; shared/internal faces are removed by the exact source union predicate.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import random
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ENGINE = ROOT / 'apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe'


def rotation(yaw=0, pitch=0):
    a,b=math.radians(yaw),math.radians(pitch)
    c,s,C,S=math.cos(a),math.sin(a),math.cos(b),math.sin(b)
    return [[c*C,-s,c*S],[s*C,c,s*S],[-S,0,C]]


def transform(p, matrix):
    return tuple(sum(row[i]*p[i] for i in range(3)) for row in matrix)


def box(origin, size):
    return {'origin':list(origin), 'size':list(size)}


def cases():
    base=box((0,0,0),(80,60,8)); upright=box((0,0,8),(80,8,60))
    rib={'vertices':[[36,8,8],[36,48,8],[36,8,58]],'depth':8}
    return [
        ('single-box',[box((0,0,0),(60,40,20))],None,1,1,rotation()),
        ('coplanar-joined-boxes',[box((0,0,0),(30,40,20)),box((30,0,0),(30,40,20))],None,1,1,rotation()),
        ('l-bracket',[base,upright],None,2,1,rotation()),
        ('l-bracket-gusset',[base,upright],rib,2,1,rotation()),
        ('t-bracket',[box((0,0,0),(80,30,8)),box((32,0,8),(16,30,50))],None,2,1,rotation()),
        ('three-step',[box((0,0,0),(90,30,10)),box((0,0,10),(60,30,10)),box((0,0,20),(30,30,10))],None,3,1,rotation()),
        ('four-step',[box((0,0,0),(80,30,8)),box((0,0,8),(60,30,8)),box((0,0,16),(40,30,8)),box((0,0,24),(20,30,8))],None,4,1,rotation()),
        ('u-bracket',[box((0,0,0),(80,30,8)),box((0,0,8),(8,30,50)),box((72,0,8),(8,30,50))],None,3,1,rotation()),
        ('overlapping-l',[box((0,0,0),(80,60,8)),box((0,0,0),(80,8,68))],None,2,1,rotation()),
        ('overlapping-rectangle',[box((0,0,0),(40,40,20)),box((20,0,0),(40,40,20))],None,1,1,rotation()),
        ('separated-boxes',[box((0,0,0),(20,30,10)),box((35,0,0),(25,30,10))],None,2,2,rotation()),
        ('rotated-l',[base,upright],None,2,1,rotation(31,23)),
        ('rotated-gusset',[base,upright],rib,2,1,rotation(31,23)),
        ('shuffled-gusset',[base,upright],rib,2,1,rotation(17,11)),
    ]


def contains(p, boxes, rib=None):
    if any(all(b['origin'][i]-1e-8 <= p[i] <= b['origin'][i]+b['size'][i]+1e-8 for i in range(3)) for b in boxes):
        return True
    if rib:
        a,b,c=rib['vertices']; run=b[1]-a[1];rise=c[2]-a[2]
        return a[0]-1e-8<=p[0]<=a[0]+rib['depth']+1e-8 and p[1]>=a[1]-1e-8 and p[2]>=a[2]-1e-8 and (p[1]-a[1])/run+(p[2]-a[2])/rise<=1+1e-8
    return False


def mass_properties(boxes,rib):
    cuts=[sorted({b['origin'][a]+v*b['size'][a] for b in boxes for v in (0,1)}) for a in range(3)]
    volumes=[]
    for indices in itertools.product(*(range(len(c)-1) for c in cuts)):
        lo=[cuts[a][indices[a]] for a in range(3)];hi=[cuts[a][indices[a]+1] for a in range(3)]
        center=[(lo[a]+hi[a])/2 for a in range(3)]
        if contains(center,boxes):
            volumes.append((math.prod(hi[a]-lo[a] for a in range(3)),center))
    if rib:
        a,b,c=rib['vertices']
        volumes.append((rib['depth']*(b[1]-a[1])*(c[2]-a[2])/2,[a[0]+rib['depth']/2,(a[1]+b[1]+c[1])/3,(a[2]+b[2]+c[2])/3]))
    volume=sum(v for v,_ in volumes)
    return volume, [sum(v*p[a] for v,p in volumes)/volume for a in range(3)]


def sample_surface(boxes,rib=None,spacing=1.0):
    points=[]
    def emit(p,n):
        probe=[p[i]+n[i]*1e-4 for i in range(3)]
        if not contains(probe,boxes,rib):points.append((*p,*n))
    for b in boxes:
        for a in range(3):
            u,v=[i for i in range(3) if i!=a]
            nu,nv=[math.ceil(b['size'][i]/spacing) for i in (u,v)]
            for sign in (-1,1):
                for i in range(nu):
                    for j in range(nv):
                        p=list(b['origin']);p[a]+=(sign+1)/2*b['size'][a]
                        p[u]+=(i+0.5)*b['size'][u]/nu;p[v]+=(j+0.5)*b['size'][v]/nv
                        n=[0,0,0];n[a]=sign;emit(p,n)
    if rib:
        a,b,c=rib['vertices']; run=b[1]-a[1];rise=c[2]-a[2];depth=rib['depth']
        # Two triangular end faces, sampled on an interior barycentric lattice.
        count=math.ceil(max(run,rise)/spacing)
        for sign in (-1,1):
            for i in range(count):
                for j in range(count-i):
                    u,v=(i+1/3)/count,(j+1/3)/count
                    p=[a[0]+(sign+1)/2*depth,a[1]+run*u,a[2]+rise*v]
                    emit(p,[sign,0,0])
        for start,end in ((a,b),(b,c),(c,a)):
            dy,dz=end[1]-start[1],end[2]-start[2]
            length=math.hypot(dy,dz);n=[0,dz/length,-dy/length]
            nu,nv=math.ceil(depth/spacing),math.ceil(length/spacing)
            for i in range(nu):
                for j in range(nv):
                    t=(j+0.5)/nv
                    emit([a[0]+depth*(i+0.5)/nu,start[1]+dy*t,start[2]+dz*t],n)
    return points


def generate(directory,noise=0.02,seed=4500):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    diagnostics=[]
    for name,boxes,rib,expected_boxes,solids,matrix in cases():
        case=directory/name;case.mkdir(exist_ok=True)
        volume,center=mass_properties(boxes,rib)
        vertices=[tuple(b['origin'][a]+sign[a]*b['size'][a] for a in range(3)) for b in boxes for sign in itertools.product((0,1),repeat=3)]
        if rib:vertices += [tuple(v[a]+(side*rib['depth'] if a==0 else 0) for a in range(3)) for v in rib['vertices'] for side in (0,1)]
        transformed=[transform(v,matrix) for v in vertices]
        bbox=[max(v[a] for v in transformed)-min(v[a] for v in transformed) for a in range(3)]
        definition={'id':name,'unit':'mm','geometry':'box_union','parameters':{'boxes':boxes,'rotation':matrix},
                    'expected':{'bounding_box_mm':bbox,'volume_mm3':volume,'solids':solids,'through_holes':0,'hole_diameters_mm':[],
                                'boxes':expected_boxes,'prisms':int(rib is not None)},
                    'acceptance':{'dimension_tolerance_mm':0.5,'hole_diameter_tolerance_mm':0.5,'symmetric_difference_volume_fraction':0.05}}
        if rib:definition['parameters']['rib']=rib
        (case/'case.json').write_text(json.dumps(definition,indent=2)+'\n')
        rng=random.Random(seed)
        points=[]
        for sample in sample_surface(boxes,rib):
            p=transform([sample[a]-center[a] for a in range(3)],matrix);n=transform(sample[3:],matrix)
            points.append(tuple(p[a]+rng.gauss(0,noise) for a in range(3))+tuple(n[a]+rng.gauss(0,0.0002 if noise else 0) for a in range(3)))
        cloud=case/'cloud.ply'
        if name.startswith('shuffled'):rng.shuffle(points)
        cloud.write_text('ply\nformat ascii 1.0\nelement vertex '+str(len(points))+'\n'+''.join('property float '+f+'\n' for f in ('x','y','z','nx','ny','nz'))+'end_header\n'+''.join(' '.join(f'{v:.9f}' for v in p)+'\n' for p in points))
        (case/'reconstruction.json').write_text('{"unit":"mm"}\n')
        diagnostics.append({'case':name,'points':len(points),'expected_boxes':expected_boxes,'expected_prisms':int(rib is not None)})
    return diagnostics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/compound-fit-tests')
    parser.add_argument('--noise',type=float,default=0.02)
    args=parser.parse_args()
    print(json.dumps(generate(args.output,args.noise),indent=2))

if __name__=='__main__':main()
