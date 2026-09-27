from __future__ import annotations
import argparse,json,pickle,time
from pathlib import Path
import numpy as np
from tree_runtime import export_arrays,tree_predict
from observer_v1 import observer
from physics_model import fresh_baseline
from oracle_v1 import metrics,aggregate
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('--features',type=Path,required=True);a=p.parse_args()
 split=json.loads((ROOT/'config/split_v1.json').read_text());m=pickle.load((ROOT/'versions/candidates_wave2.pkl').open('rb'))
 tree=export_arrays(m['drive_hgb']);coef=m['drive_parametric'].coef_.copy()
 np.savez(ROOT/'config/drive_tree_v1.npz',**{f'arr{i}':a for i,a in enumerate(tree)})
 # Verify exported runtime versus fitted model on fixed TRAIN inputs.
 z=np.load(a.features/'30618_0e41eac3.npz');X=z['X'][::23];v=fresh_baseline(X)
 inp=np.column_stack([X[:,18],v,X[:,19],X[:,20],X[:,21]])
 maxdiff=float(np.max(abs(tree_predict(inp,tree)-m['drive_hgb'].predict(inp))))
 print('TREE_RUNTIME_MAX_ERROR',maxdiff,flush=True);assert maxdiff<1e-10
 rows=[];times=[]
 for b,r in split.items():
  if r['part']=='holdout':continue
  z=np.load(a.features/(b+'.npz'));X=z['X'];t=z['t']
  for name,learned in [('B1_parametric_observer',False),('B2_learned_drive_observer',True)]:
   st=time.perf_counter();y,flags,acc=observer(t,X,tree,coef,learned);times.append(time.perf_counter()-st)
   rows.append({'bag':b,'part':r['part'],'vehicle':b[:5],'candidate':name,'flagged_fraction':float(np.mean(flags>0)),**metrics(t,z['y'],y)})
 (ROOT/'results/wave3_per_bag.json').write_text(json.dumps(rows,indent=2))
 summary=[]
 for part in ['train','validation']:
  for n in ['B1_parametric_observer','B2_learned_drive_observer']:
   for veh in ['all','30618','30639']:
    s={'part':part,'candidate':n,'vehicle':veh,**aggregate([q for q in rows if q['part']==part and q['candidate']==n and (veh=='all' or q['vehicle']==veh)])};summary.append(s);print(s,flush=True)
 (ROOT/'results/wave3_summary.json').write_text(json.dumps(summary,indent=2))
 print('total batch seconds excl compilation',sum(times[1:]),flush=True)
if __name__=='__main__':main()
