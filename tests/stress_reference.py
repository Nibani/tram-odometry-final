"""Synthetic corruptions of TRAIN inputs, original GNSS kept as reference.
Not evidence about frequency/distribution of failures in hidden tests."""
from pathlib import Path
import numpy as np,sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from prepare_features import build
from tree_runtime import load_arrays
from observer_v3 import observer_v3
from physics_model import fresh_baseline
from oracle_v1 import metrics
cache=Path(sys.argv[1]);bag='30618_0e41eac3';ar=np.load(cache/(bag+'.npz'));raw={k:ar[k] for k in ar.files};clean=build(raw)
t=clean['t'];y=clean['y'];start=None
for s in np.arange(30.,t[-1]-10,1.):
 m=(t>=s)&(t<s+3.)&np.isfinite(y)
 if m.sum()>45 and np.mean(y[m])>5. and np.std(y[m])<.06:start=float(s);break
assert start is not None
coef=np.array(json.loads((ROOT/'config/drive_parametric_v1.json').read_text())['coef']);tree=load_arrays(ROOT/'config/drive_tree_v1.npz');rows=[]
for mode in ['clean','front_spike_3mps_1s','both_step_3mps_1s','both_smooth_ramp_0.5mps2_3s','both_missing_1s','rear_missing_whole_bag']:
 d={k:v.copy() for k,v in raw.items()}
 for n in ('front','rear'):
  tt=(d[n+'_ht']-clean['q'][0])*1e-9;mask=(tt>=start)&(tt<start+1.)
  if (mode=='front_spike_3mps_1s' and n=='front') or mode=='both_step_3mps_1s':d[n+'_data'][mask,0]+=3.*3.6
  if mode=='both_smooth_ramp_0.5mps2_3s':
   mask=(tt>=start)&(tt<start+3.);d[n+'_data'][mask,0]+=.5*(tt[mask]-start)*3.6
  if mode=='both_missing_1s' or (mode=='rear_missing_whole_bag' and n=='rear'):
   keep=~mask if mode=='both_missing_1s' else np.zeros(len(tt),bool)
   for suf in ('ht','bt','data'):d[n+'_'+suf]=d[n+'_'+suf][keep]
 z=build(d);a=fresh_baseline(z['X']);b,flags,_=observer_v3(z['t'],z['X'],tree,coef,True)
 region=(t>=start)&(t<start+6.)
 row={'scenario':mode,'injection_start_s':start,'evaluation_window_s':6.,'baseline':metrics(t[region],y[region],a[region]),'B6':metrics(t[region],y[region],b[region]),'finite_nonnegative':bool(np.all(np.isfinite(b)) and np.all(b>=0.))}
 assert row['finite_nonnegative'];rows.append(row);print(mode,'A3',row['baseline']['rmse'],'B6',row['B6']['rmse'],flush=True)
(ROOT/'results/synthetic_stress.json').write_text(json.dumps({'status':'SYNTHETIC_TRAIN_INPUT_CORRUPTIONS','bag':bag,'results':rows},indent=2))
