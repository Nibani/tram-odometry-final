"""Reproduce the selected drive fit from TRAIN wheel signals only.
Writes a NEW output directory; does not overwrite the frozen candidate."""
from __future__ import annotations
import os
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,json
from pathlib import Path
import numpy as np
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.linear_model import HuberRegressor
from physics_model import fresh_baseline,drive_basis
from tree_runtime import export_arrays,load_arrays,tree_predict
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('--features',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 a.out.mkdir(parents=True,exist_ok=True);ax=[];ay=[]
 for b,r in json.loads((ROOT/'config/split_v1.json').read_text()).items():
  if r['part']!='train':continue
  z=np.load(a.features/(b+'.npz'));X=z['X'];v=fresh_baseline(X)
  safe=(X[:,5]<.3)&(X[:,6]<.3)&(X[:,4]<.15)&(abs(X[:,9]-X[:,10])<.25)&(v>.3)&(abs(X[:,9])<1.8)&(abs(X[:,10])<1.8)&(z['t']>5)
  idx=np.flatnonzero(safe)[::5];ax.append(np.column_stack([X[idx,18],v[idx],X[idx,19],X[idx,20],X[idx,21]]));ay.append((.5*(X[:,9]+X[:,10]))[idx])
 X=np.concatenate(ax);y=np.concatenate(ay)
 m=HistGradientBoostingRegressor(loss='squared_error',max_iter=120,max_leaf_nodes=15,learning_rate=.065,l2_regularization=10,min_samples_leaf=150,early_stopping=False,random_state=20260925).fit(X,y)
 tree=export_arrays(m);np.savez(a.out/'drive_tree_retrained.npz',**{f'arr{i}':v for i,v in enumerate(tree)})
 h=HuberRegressor(alpha=.05,fit_intercept=False,max_iter=250).fit(drive_basis(X[::3,3],X[::3,1]),y[::3])
 (a.out/'drive_parametric_retrained.json').write_text(json.dumps({'coef':h.coef_.tolist()},indent=2))
 err=float(np.max(abs(tree_predict(X[::101],tree)-tree_predict(X[::101],load_arrays(ROOT/'config/drive_tree_v1.npz')))))
 report={'training_examples':len(X),'features':5,'GNSS_targets_used':False,'maximum_prediction_difference_from_frozen':err}
 (a.out/'training_report.json').write_text(json.dumps(report,indent=2));print(report)
if __name__=='__main__':main()
