from pathlib import Path
import json
import argparse
import yaml
import numpy as np
work=Path(__file__).resolve().parent
meshRoot=work.parent/'models/rmuc2027_terrain/meshes'
stlType=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
parts=[]
for path in sorted(meshRoot.glob('collision_*.stl')):
    with path.open('rb') as stream:
        stream.seek(80)
        count=int.from_bytes(stream.read(4),'little')
        parts.append(np.fromfile(stream,dtype=stlType,count=count)['v'].astype(float))
t=np.concatenate(parts); lo=t.min(1); hi=t.max(1)

def clearance(point, limit=1.0):
    p=np.asarray(point,float)
    mask=np.all(lo<=p+limit,axis=1)&np.all(hi>=p-limit,axis=1)
    q=t[mask]
    if not len(q): return float(limit)
    a,b,c=q[:,0],q[:,1],q[:,2];ab=b-a;ac=c-a;ap=p-a
    d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac);d11=np.einsum('ij,ij->i',ac,ac)
    d20=np.einsum('ij,ij->i',ap,ab);d21=np.einsum('ij,ij->i',ap,ac);den=d00*d11-d01*d01
    v=np.divide(d11*d20-d01*d21,den,out=np.zeros_like(den),where=den>1e-20)
    w=np.divide(d00*d21-d01*d20,den,out=np.zeros_like(den),where=den>1e-20)
    proj=a+v[:,None]*ab+w[:,None]*ac
    ds=np.where((den>1e-20)&(v>=0)&(w>=0)&(v+w<=1),np.sum((proj-p)**2,axis=1),np.inf)
    for x,y in ((a,b),(b,c),(c,a)):
        e=y-x;ee=np.sum(e*e,axis=1)
        u=np.clip(np.divide(np.sum((p-x)*e,axis=1),ee,out=np.zeros_like(ee),where=ee>0),0,1)
        ds=np.minimum(ds,np.sum((x+u[:,None]*e-p)**2,axis=1))
    return min(float(np.sqrt(ds.min())),limit)

def segment(a,b):
    a,b=np.array(a,float),np.array(b,float)
    n=max(2,int(np.ceil(np.linalg.norm(b-a)/.1))+1)
    samples=np.linspace(a,b,n)
    values=[clearance(p) for p in samples]
    tri=t[np.all(lo<=np.maximum(a,b),axis=1)&np.all(hi>=np.minimum(a,b),axis=1)]
    edge1=tri[:,1]-tri[:,0];edge2=tri[:,2]-tri[:,0];direction=b-a
    h=np.cross(direction,edge2);det=np.sum(edge1*h,axis=1)
    valid=np.abs(det)>1e-10;inv=np.divide(1,det,out=np.zeros_like(det),where=valid)
    delta=a-tri[:,0];u=inv*np.sum(delta*h,axis=1);cross=np.cross(delta,edge1)
    v=inv*np.sum(cross*direction,axis=1);fraction=inv*np.sum(edge2*cross,axis=1)
    hits=valid&(u>=0)&(v>=0)&(u+v<=1)&(fraction>=0)&(fraction<=1)
    return {'samples':n,'segment_surface_intersections':int(hits.sum()),'minimum_surface_distance_m_capped_at_1m':min(values),'minimum_at':samples[int(np.argmin(values))].tolist(),'sampled_radius_025_clear':min(values)>.25}

scenarioFile=work.parents[2]/'config/arena/rmuc2027_scenarios.yaml'
config=yaml.safe_load(scenarioFile.read_text())
cases={case['id']:[case['reference_hold_pose_arena_xyz_yaw'][:3],*case['candidate_goals_arena_xyz_m']]
       for case in config['cases'] if case['kind']!='fault'}
# 独立的几何对照折线，仅用于确认旁侧空间；不向 EGO 下发。
cases['S04_reference_bypass']=[[-11.6,-2,1.0],[-12.7,-1,1.0],[-12.7,1,1.0],[-11.6,2,1.0]]

def surfaceHeight(x,y):
    q=t[(lo[:,0]<=x)&(hi[:,0]>=x)&(lo[:,1]<=y)&(hi[:,1]>=y)]
    a,b,c=q[:,0],q[:,1],q[:,2]
    u=b[:,:2]-a[:,:2];v=c[:,:2]-a[:,:2];d=np.array([x,y])-a[:,:2]
    den=u[:,0]*v[:,1]-u[:,1]*v[:,0]
    s=np.divide(d[:,0]*v[:,1]-d[:,1]*v[:,0],den,out=np.zeros_like(den),where=np.abs(den)>1e-15)
    r=np.divide(u[:,0]*d[:,1]-u[:,1]*d[:,0],den,out=np.zeros_like(den),where=np.abs(den)>1e-15)
    good=(np.abs(den)>1e-15)&(s>=0)&(r>=0)&(s+r<=1)
    h=a[:,2]+s*(b[:,2]-a[:,2])+r*(c[:,2]-a[:,2])
    return None if not np.any(good) else float(np.max(h[good]))

report={'scope' :'offline collision-surface samples only; no simulation, dynamics, visibility or reachability proof','probe_radius_m':.25,'sample_step_m':.1,'distance_cap_m':1,'triangle_count':len(t),'cases':{}}
for name,points in cases.items():
    report['cases'][name]={'points':points,'surface_height_at_points_m':[surfaceHeight(p[0],p[1]) for p in points],'endpoints_distance_m_capped_at_1m':[clearance(p) for p in points],'segments':[segment(a,b) for a,b in zip(points,points[1:])]}
parser=argparse.ArgumentParser(description='Offline candidate collision-surface inspection; no simulation')
parser.add_argument('--out',required=True)
args=parser.parse_args()
out=Path(args.out).resolve()
if out.exists(): raise FileExistsError(out)
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n')
for name,c in report['cases'].items():print(name, [round(s['minimum_surface_distance_m_capped_at_1m'],3) for s in c['segments']])
