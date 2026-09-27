from __future__ import annotations
import argparse,json,pickle
from pathlib import Path
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.ndimage import median_filter
from route_map import enu,project,lookup
from prepare_features import series
from physics_model import fresh_baseline
from observer_v2 import observer_v2
from tree_runtime import export_arrays
ROOT=Path(__file__).resolve().parents[1]
ORIGIN=[55.805,37.425,170.]
SOURCES=['30618_073f08d1','30618_0652866c']

def integrate(t,v):return np.r_[0,np.cumsum(.5*(v[1:]+v[:-1])*np.diff(t))]

def main():
 p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--features',type=Path,required=True);a=p.parse_args()
 split=json.loads((ROOT/'config/split_v1.json').read_text());maps=[]
 for b in SOURCES:
  assert split[b]['part']=='train'
  raw=np.load(a.cache/(b+'.npz'));z=np.load(a.features/(b+'.npz'));v=fresh_baseline(z['X']);s=integrate(z['t'],v)
  h,bt,f=series(raw,'master_fix');q=(h-z['q'][0])/1e9
  valid=(q>=z['t'][0])&(q<=z['t'][-1]);q=q[valid];pts=enu(f[valid,:3],ORIGIN);sp=np.interp(q,z['t'],s)
  bins=(sp/1.).astype(int);ub=np.unique(bins)
  ss=[];pp=[]
  for k in ub:
   m=bins==k;ss.append(float(np.median(sp[m])));pp.append(np.median(pts[m],axis=0))
  ss=np.array(ss);pp=np.array(pp);pp=median_filter(pp,size=(5,1),mode='nearest')
  grid=np.arange(ss[0],ss[-1],.5)
  xyz=np.column_stack([UnivariateSpline(ss,pp[:,j],s=len(ss)*(.25 if j<2 else 1.)**2)(grid) for j in range(3)])
  arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1))];poly=np.column_stack([arc,xyz]);maps.append(poly)
  np.savetxt(ROOT/'config'/('experimental_route_'+b+'.csv'),poly,delimiter=',',header='s,x,y,z',comments='')
  print('TRAIN MAP',b,'length',arc[-1],flush=True)
 (ROOT/'config/experimental_map_meta.json').write_text(json.dumps({'status':'EXPERIMENTAL_GNSS_DERIVED_TRAIN_ONLY_PERMISSION_REQUIRED','origin_lla':ORIGIN,'sources':SOURCES,'path_parameter':'horizontal arclength','initial_gnss_window_s':3.,'frame':'fixed ENU at map origin, translated by first initialization fix','branch_prior':'penalize less than 100 m of remaining map; initial velocity direction used only when available'},indent=2))
 models=pickle.load((ROOT/'versions/candidates_wave2.pkl').open('rb'));tree=export_arrays(models['drive_hgb']);coef=models['drive_parametric'].coef_
 rows=[]
 for b,r in split.items():
  if r['part']!='validation':continue
  raw=np.load(a.cache/(b+'.npz'));z=np.load(a.features/(b+'.npz'));q=z['q'];qb=z['qb'];t=z['t']
  n='master_fix' if len(raw['master_fix_ht']) else 'rover_fix';h,bt,x=series(raw,n)
  good=np.all(np.isfinite(x[:,:3]),axis=1)&(bt<=qb[0]+3_000_000_000)&(h>=q[0])
  if not good.any():rows.append({'bag':b,'status':'NO_INITIAL_FIX'});continue
  init_idx=np.flatnonzero(good)[0];p0=enu(x[init_idx,:3],ORIGIN);t0=(h[init_idx]-q[0])/1e9
  vn='master_vel' if len(raw['master_vel_ht']) else 'rover_vel';vh,vbt,vx=series(raw,vn)
  vg=(vbt<=qb[0]+3_000_000_000)&(vh>=q[0]);vv=np.median(vx[vg,:2],axis=0) if vg.any() else np.zeros(2)
  heading=vv/np.linalg.norm(vv) if np.linalg.norm(vv)>1 else None
  choices=[]
  for j,poly in enumerate(maps):
   sp,dist,tan=project(poly,p0);cost=dist+(10*(1-float(tan@heading)) if heading is not None else 0)+(max(0,100-(poly[-1,0]-sp))*.2)
   choices.append((cost,j,sp))
  cost,j,s0=min(choices);poly=maps[j];anchor=lookup(poly,np.array([s0]))[0]
  idx=np.minimum(np.searchsorted(h,q),len(h)-1);prev=np.maximum(idx-1,0);idx=np.where(abs(h[prev]-q)<abs(h[idx]-q),prev,idx)
  valid=(abs(h[idx]-q)<=50_000_000)&(t>=t0)&np.all(np.isfinite(x[idx,:3]),axis=1)
  gt=enu(x[idx,:3],ORIGIN)-p0
  for name,v in [('A3_fresh_extrap',fresh_baseline(z['X'])),('B4_learned_drive_recovery',observer_v2(t,z['X'],tree,coef,True)[0])]:
   s=integrate(t,v);progress=s-np.interp(t0,t,s);pp=lookup(poly,s0+progress)-anchor;err=pp-gt;e=np.linalg.norm(err[valid],axis=1)
   rows.append({'bag':b,'candidate':name,'status':'CONDITIONAL_NOT_OFFICIAL','n':int(valid.sum()),'map_source':SOURCES[j],
    'rmse_3d':float(np.sqrt(np.mean(e**2))),'rmse_xy':float(np.sqrt(np.mean(np.sum(err[valid,:2]**2,axis=1)))),
    'endpoint_3d_m':float(e[-1]),'distance_m':float(progress[-1]),'clipped_fraction':float(np.mean((s0+progress<0)|(s0+progress>poly[-1,0]))),
    'initial_projection_distance_m':project(poly,p0)[1]})
 (ROOT/'results/experimental_map_validation.json').write_text(json.dumps(rows,indent=2))
 for r in rows:print(r,flush=True)
if __name__=='__main__':main()
