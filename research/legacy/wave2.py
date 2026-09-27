from __future__ import annotations
import os
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,json,pickle,time
from pathlib import Path
import numpy as np
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.linear_model import HuberRegressor
from physics_model import drive_basis,fresh_baseline
from experiments_v1 import predict_candidates
from oracle_v1 import metrics,aggregate
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('--features',type=Path,required=True);a=p.parse_args()
 split=json.loads((ROOT/'config/split_v1.json').read_text());xx=[];yy=[];ax=[];ay=[]
 for b,r in split.items():
  if r['part']!='train':continue
  z=np.load(a.features/(b+'.npz'));X=z['X'];y=z['y'];v=fresh_baseline(X)
  safe=(X[:,5]<.3)&(X[:,6]<.3)&(X[:,4]<.15)&(abs(X[:,9]-X[:,10])<.25)&(v>.3)&(abs(X[:,9])<1.8)&(abs(X[:,10])<1.8)&(z['t']>5)
  aa=.5*(X[:,9]+X[:,10]);idx=np.flatnonzero(safe)[::5]
  ax.append(np.column_stack([X[idx,18],v[idx],X[idx,19],X[idx,20],X[idx,21]]));ay.append(aa[idx])
  good=np.isfinite(y)&(y<25)&(z['t']>5)&((X[:,5]<.35)|(X[:,6]<.35))
  good&=(~np.isfinite(z['y_rover'])) | (abs(y-z['y_rover'])<.4)
  idx=np.flatnonzero(good)[::4];xx.append(X[idx]);yy.append(y[idx]-v[idx])
 X=np.concatenate(xx);res=np.concatenate(yy);AX=np.concatenate(ax);AY=np.concatenate(ay)
 print('TRAIN RESIDUAL',X.shape,'ACCEL',AX.shape,flush=True)
 models={}
 for name,loss in [('C3_fresh_absolute','absolute_error'),('C4_fresh_squared','squared_error')]:
  m=HistGradientBoostingRegressor(loss=loss,max_iter=140,max_leaf_nodes=15,learning_rate=.065,l2_regularization=10,min_samples_leaf=100,early_stopping=False,random_state=20260925)
  m.fit(X,res);models[name]=m;print('FITTED',name,flush=True)
 # No GNSS targets for acceleration model: reliable wheel derivatives from TRAIN only.
 m=HistGradientBoostingRegressor(loss='squared_error',max_iter=120,max_leaf_nodes=15,learning_rate=.065,l2_regularization=10,min_samples_leaf=150,early_stopping=False,random_state=20260925)
 m.fit(AX,AY);models['drive_hgb']=m
 # Compact parametric physically structured alternative.
 h=HuberRegressor(alpha=.05,fit_intercept=False,max_iter=250)
 h.fit(drive_basis(AX[::3,3],AX[::3,1]),AY[::3]);models['drive_parametric']=h
 print('DRIVE PARAM',h.coef_,flush=True)
 (ROOT/'config/drive_parametric_v1.json').write_text(json.dumps({'coef':h.coef_.tolist(),'actuator_tau_s':1.0,'label':'specific acceleration calibrated from TRAIN wheel signals, no GNSS targets'},indent=2))
 with (ROOT/'versions/candidates_wave2.pkl').open('wb') as f:pickle.dump(models,f)
 rows=[]
 for b,r in split.items():
  if r['part']=='holdout':continue
  z=np.load(a.features/(b+'.npz'));X=z['X'];base=fresh_baseline(X)
  cand={'A3_fresh_extrap':base,'C3_fresh_absolute':np.maximum(0,base+np.clip(models['C3_fresh_absolute'].predict(X),-3,3)),
        'C4_fresh_squared':np.maximum(0,base+np.clip(models['C4_fresh_squared'].predict(X),-3,3))}
  for n,y in cand.items():rows.append({'bag':b,'part':r['part'],'vehicle':b[:5],'candidate':n,**metrics(z['t'],z['y'],y)})
 (ROOT/'results/wave2_per_bag.json').write_text(json.dumps(rows,indent=2))
 summary=[]
 for part in ['train','validation']:
  for n in ['A3_fresh_extrap','C3_fresh_absolute','C4_fresh_squared']:
   for veh in ['all','30618','30639']:
    s={'part':part,'candidate':n,'vehicle':veh,**aggregate([q for q in rows if q['part']==part and q['candidate']==n and (veh=='all' or q['vehicle']==veh)])};summary.append(s);print(s,flush=True)
 (ROOT/'results/wave2_summary.json').write_text(json.dumps(summary,indent=2))
if __name__=='__main__':main()
