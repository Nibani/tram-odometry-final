from __future__ import annotations
import os
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,json,pickle,time
from pathlib import Path
import numpy as np
from sklearn.linear_model import HuberRegressor
from sklearn.ensemble import HistGradientBoostingRegressor
from oracle_v1 import metrics,aggregate
ROOT=Path(__file__).resolve().parents[1]

def linear_design(X):
    gate=np.tanh(np.maximum(X[:,17],0)/.3)
    return np.column_stack([X[:,17],gate,.5*(X[:,9]+X[:,10])*gate])

def predict_candidates(X,models):
    v=X[:,17].astype(float)
    out={'A0_mean_hold':X[:,2].astype(float),'A1_causal_extrap':v}
    if 'linear' in models:out['A2_robust_calibrated']=np.maximum(0,models['linear'].predict(linear_design(X)))
    for n in ['C1_residual_absolute','C2_residual_squared']:
        if n in models:out[n]=np.maximum(0,v+np.clip(models[n].predict(X),-1.5,1.5))
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--features',required=True,type=Path);p.add_argument('--train',action='store_true');a=p.parse_args()
    split=json.loads((ROOT/'config/split_v1.json').read_text()); models={}
    if a.train:
        xx=[];yy=[];start=time.perf_counter()
        for b,rec in split.items():
            if rec['part']!='train' or not rec['reference_available']:continue
            z=np.load(a.features/(b+'.npz'));X=z['X'];y=z['y'];t=z['t']
            good=np.isfinite(y)&(y<25)&(t>5)&(X[:,5]<.4)&(X[:,6]<.4)
            good &= (~np.isfinite(z['y_rover'])) | (abs(y-z['y_rover'])<.4)
            idx=np.flatnonzero(good)[::4];xx.append(X[idx]);yy.append(y[idx])
        X=np.concatenate(xx);y=np.concatenate(yy);v=X[:,17]
        print('TRAIN SHAPE',X.shape,'elapsed',time.perf_counter()-start,flush=True)
        lin=HuberRegressor(epsilon=1.35,alpha=1e-4,fit_intercept=False,max_iter=200)
        lin.fit(linear_design(X[::2]),y[::2]);models['linear']=lin
        print('LINEAR',lin.coef_,flush=True)
        for n,loss in [('C1_residual_absolute','absolute_error'),('C2_residual_squared','squared_error')]:
            t0=time.perf_counter()
            m=HistGradientBoostingRegressor(loss=loss,max_iter=140,max_leaf_nodes=15,learning_rate=.065,l2_regularization=10,min_samples_leaf=150,early_stopping=False,random_state=20260925)
            m.fit(X,y-v);models[n]=m
            print('FITTED',n,'seconds',time.perf_counter()-t0,flush=True)
        with (ROOT/'versions/candidates_wave1.pkl').open('wb') as f:pickle.dump(models,f)
        (ROOT/'config/linear_v1.json').write_text(json.dumps({'features':['mean_extrap','tanh_gate','slope_gate'],'coef':lin.coef_.tolist()},indent=2))
    else:
        with (ROOT/'versions/candidates_wave1.pkl').open('rb') as f:models=pickle.load(f)
    rows=[]
    for b,rec in split.items():
        if rec['part']=='holdout':continue
        z=np.load(a.features/(b+'.npz'));X=z['X'];y=z['y'];t=z['t'];pred=predict_candidates(X,models)
        for name,prediction in pred.items():
            rows.append({'bag':b,'part':rec['part'],'vehicle':b[:5],'candidate':name,'reference':str(z['reference']),**metrics(t,y,prediction)})
    (ROOT/'results/wave1_per_bag.json').write_text(json.dumps(rows,indent=2))
    summary=[]
    for part in ['train','validation']:
        for name in ['A0_mean_hold','A1_causal_extrap','A2_robust_calibrated','C1_residual_absolute','C2_residual_squared']:
            for veh in ['all','30618','30639']:
                rr=[r for r in rows if r['part']==part and r['candidate']==name and (veh=='all' or r['vehicle']==veh)]
                summary.append({'part':part,'candidate':name,'vehicle':veh,**aggregate(rr)})
    (ROOT/'results/wave1_summary.json').write_text(json.dumps(summary,indent=2))
    for r in summary:
        print(r['part'],r['vehicle'],r['candidate'],'RMSE',round(r.get('rmse',0),5),'MAE',round(r.get('mae',0),5),'bias',round(r.get('bias',0),5),'|int err|',round(r.get('mean_abs_covered_integral_error_m',0),2),flush=True)
if __name__=='__main__':main()
