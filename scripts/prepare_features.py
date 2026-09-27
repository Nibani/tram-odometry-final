"""Feature extractor v3: strict causal lags and empty-channel support."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
try:
    from numba import njit
except ImportError:
    def njit(*args,**kwargs):
        return lambda fn:fn

FEATURE_NAMES=['front','rear','mean','difference','abs_difference','front_age','rear_age',
 'front_slope_short','rear_slope_short','front_slope_03','rear_slope_03',
 'front_slope_1','rear_slope_1','front_slope_3','rear_slope_3',
 'front_extrap','rear_extrap','mean_extrap','cmd','cmd_ema_03','cmd_ema_1','cmd_ema_3',
 'front_prev_1','rear_prev_1','front_prev_3','rear_prev_3']

@njit(cache=True)
def ema(t,x,tau):
    y=x.copy()
    for i in range(1,len(t)):
        a=1-np.exp(-max(0.0,t[i]-t[i-1])/tau)
        y[i]=y[i-1]+a*(x[i]-y[i-1])
    return y

def monotonic_mask(h):
    if len(h)==0:return np.zeros(0,dtype=bool)
    return np.r_[True,h[1:]>np.maximum.accumulate(h)[:-1]]

def series(z,n):
    h=z[n+'_ht'];b=z[n+'_bt'];x=z[n+'_data'];m=monotonic_mask(h)
    return h[m],b[m],x[m]

def nearest_ref(z,n,q):
    h,b,x=series(z,n)
    if not len(h): return np.full((len(q),3),np.nan),np.full(len(q),np.inf)
    i=np.minimum(np.searchsorted(h,q),len(h)-1);p=np.maximum(i-1,0)
    i=np.where(np.abs(h[p]-q)<np.abs(h[i]-q),p,i)
    dist=np.abs(h[i]-q)/1e9;y=x[i,:3].copy();y[dist>.05]=np.nan
    return y,dist

def wheel_features(z,n,q,qb):
    h,b,x=series(z,n)
    if not len(h):
        zero=np.zeros(len(q));age=np.full(len(q),10.)
        return zero,age,zero,[zero.copy() for _ in range(3)],[zero.copy() for _ in range(3)],zero
    v=x[:,0]/3.6
    # Last message already arrived AND measured no later than the output's stamp.
    i=np.minimum(np.searchsorted(b,qb,side='right'),np.searchsorted(h,q,side='right'))-1
    present=i>=0;i=np.maximum(i,0)
    val=np.where(present,v[i],0);age=np.where(present,(q-h[i])/1e9,10.)
    prev=np.maximum(i-1,0);dt=(h[i]-h[prev])/1e9
    ds=np.divide(v[i]-v[prev],dt,out=np.zeros(len(q)),where=dt>1e-6)
    slopes=[];lags=[]
    for sec in (.3,1,3):
        j=np.maximum(np.searchsorted(h,h[i]-int(sec*1e9),side='right')-1,0)
        delta=(h[i]-h[j])/1e9
        slopes.append(np.clip(np.divide(v[i]-v[j],delta,out=np.zeros(len(q)),where=delta>1e-6),-4,4))
        lags.append(np.where(present,v[j],0.))
    ds=np.clip(ds,-4,4)
    extrap=np.maximum(0,val+np.minimum(age,.3)*slopes[0])
    return val,age,ds,slopes,lags,extrap

def build(z):
    q,qb,c=series(z,'cmd')
    if not len(q): raise ValueError('No valid controller timestamps for the benchmark grid')
    u=c[:,0]
    f,af,df,sf,lf,ef=wheel_features(z,'front',q,qb)
    r,ar,dr,sr,lr,er=wheel_features(z,'rear',q,qb)
    t=(q-q[0])/1e9
    X=np.column_stack([f,r,(f+r)/2,f-r,abs(f-r),np.minimum(af,10),np.minimum(ar,10),df,dr,
      sf[0],sr[0],sf[1],sr[1],sf[2],sr[2],ef,er,(ef+er)/2,u,
      ema(t,u,.3),ema(t,u,1.),ema(t,u,3.),lf[1],lr[1],lf[2],lr[2]])
    m,md=nearest_ref(z,'master_vel',q);r,rd=nearest_ref(z,'rover_vel',q)
    master=np.linalg.norm(m[:,:2],axis=1);rover=np.linalg.norm(r[:,:2],axis=1)
    ref=master if len(z['master_vel_ht']) else rover
    source='master' if len(z['master_vel_ht']) else 'rover_only'
    return dict(X=X.astype(np.float32),t=t,q=q,qb=qb,y=ref,y_master=master,y_rover=rover,
      y3_master=np.linalg.norm(m,axis=1),y3_rover=np.linalg.norm(r,axis=1),reference=np.array(source),feature_names=np.array(FEATURE_NAMES))

def main():
    p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--split',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);split=json.loads(a.split.read_text())
    for i,b in enumerate(split):
        with np.load(a.cache/(b+'.npz')) as z: d=build(z)
        np.savez_compressed(a.out/(b+'.npz'),**d)
        if (i+1)%20==0:print('features',i+1,flush=True)
if __name__=='__main__':main()
