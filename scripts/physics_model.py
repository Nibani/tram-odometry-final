"""Compact normalized longitudinal drive model. No claimed mass/torque identifiability."""
from __future__ import annotations
import numpy as np

def drive_basis(u,v):
    u=np.asarray(u);v=np.asarray(v);p=np.maximum(u,0)/15;n=np.minimum(u,0)/15
    # Specific-force basis, not independently measured N or N*m.
    return np.column_stack([p,n,p*p,n*n,p*v/15,n*v/15,
        np.tanh(v/.3),v/15,(v/15)**2])

def fresh_baseline(X):
    f=X[:,15].astype(float);r=X[:,16].astype(float)
    vf=X[:,5]<.35;vr=X[:,6]<.35
    y=np.where(vf&vr,.5*(f+r),np.where(vf,f,np.where(vr,r,np.where(X[:,5]<=X[:,6],f,r))))
    return np.maximum(y,0)
