from __future__ import annotations
import numpy as np
from numba import njit
from tree_runtime import tree_predict_one

@njit(cache=True)
def parametric_drive(u,v,coef):
    p=max(u,0.)/15;n=min(u,0.)/15
    x=np.array([p,n,p*p,n*n,p*v/15,n*v/15,np.tanh(v/.3),v/15,(v/15)**2])
    return np.dot(x,coef)

@njit(cache=True)
def observer(t,X,tree,coef,use_learned=True,gate=.35):
    """Causal two-channel innovation gate with adaptive specific-force bridge.
    This is NOT a complete IMM/UKF implementation. State: v, disturbance,
    two validity states, recovery durations; s is integrated outside.
    """
    out=np.zeros(len(t));flags=np.zeros(len(t),dtype=np.int32);acc=np.zeros(len(t))
    d=0.;v=0.;bad=np.zeros(2,dtype=np.bool_);recover=np.zeros(2);elapsed=0.;initialized=False
    for k in range(len(t)):
        dt=.05 if k==0 else max(0.,min(t[k]-t[k-1],.5))
        zz=np.array([X[k,15],X[k,16]],dtype=np.float64)
        ages=np.array([X[k,5],X[k,6]],dtype=np.float64)
        slopes=np.array([X[k,9],X[k,10]],dtype=np.float64)
        fresh=ages<.35
        if not initialized and np.any(fresh):
            v=np.mean(zz[fresh]);initialized=True
        xin=np.array([X[k,18],v,X[k,19],X[k,20],X[k,21]],dtype=np.float64)
        am=tree_predict_one(xin,tree) if use_learned else parametric_drive(X[k,20],v,coef)
        am=min(1.6,max(-2.2,am))
        a=min(1.6,max(-2.2,am+d));pred=max(0.,v+a*dt)
        threshold=gate+.15*min(elapsed,5.)
        valid=np.zeros(2,dtype=np.bool_)
        for j in range(2):
            plausible=slopes[j]<1.65 and slopes[j]>-2.35
            residual=abs(zz[j]-pred)
            if fresh[j] and plausible and residual<threshold:
                recover[j]+=dt
                if not bad[j] or recover[j]>=.15:
                    bad[j]=False;valid[j]=True
            else:
                bad[j]=True;recover[j]=0.
        if np.any(valid):
            meas=np.mean(zz[valid]);v=meas
            # Both low wheel speed AND near-zero model prediction: avoids false stop in a full slide.
            if v<.012 and pred<.08:v=0.
            obs_a=np.mean(slopes[valid])
            if abs(obs_a-am)<1. and np.all(valid):
                d+=(1-np.exp(-dt/3.))*((obs_a-am)-d);d=min(.6,max(-.6,d))
            elapsed=0.
        else:
            v=pred;elapsed+=dt
        out[k]=v;acc[k]=a;flags[k]=int(bad[0])+2*int(bad[1])
    return out,flags,acc
