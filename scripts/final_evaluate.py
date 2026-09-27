"""Frozen A3/B5/B6 evaluation, no fitting. holdout is previously exposed audit data."""
from __future__ import annotations
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
from tree_runtime import load_arrays
from observer_v3 import observer_v3
from physics_model import fresh_baseline
from oracle_v1 import metrics,aggregate
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--features',type=Path,required=True)
    p.add_argument('--part',choices=['train','validation','holdout'],required=True);p.add_argument('--save-predictions',action='store_true');a=p.parse_args()
    split=json.loads((ROOT/'config/split_v1.json').read_text());tree=load_arrays(ROOT/'config/drive_tree_v1.npz')
    coef=np.array(json.loads((ROOT/'config/drive_parametric_v1.json').read_text())['coef'])
    rows=[]
    for b,r in split.items():
        if r['part']!=a.part:continue
        z=np.load(a.features/(b+'.npz'));X=z['X'];t=z['t'];preds={'A3_fresh_extrap':fresh_baseline(X)}
        for name,learned in [('B5_parametric_uncertainty',False),('B6_learned_drive_uncertainty',True)]:
            preds[name]=observer_v3(t,X,tree,coef,learned)[0]
        for n,pred in preds.items():
            rows.append({'bag':b,'part':a.part,'vehicle':b[:5],'candidate':n,**metrics(t,z['y'],pred)})
        if a.save_predictions:
            dest=ROOT/'results/predictions';dest.mkdir(exist_ok=True)
            np.savez_compressed(dest/(b+'.npz'),t=t,reference=z['y'],**preds)
    summary=[]
    for cand in ['A3_fresh_extrap','B5_parametric_uncertainty','B6_learned_drive_uncertainty']:
        for vehicle in ['all','30618','30639']:
            q={'part':a.part,'candidate':cand,'vehicle':vehicle,**aggregate([r for r in rows if r['candidate']==cand and (vehicle=='all' or r['vehicle']==vehicle)])}
            summary.append(q)
            if vehicle=='all':print(json.dumps(q),flush=True)
    (ROOT/f'results/final_{a.part}_per_bag.json').write_text(json.dumps(rows,indent=2))
    (ROOT/f'results/final_{a.part}_summary.json').write_text(json.dumps(summary,indent=2))
if __name__=='__main__':main()
