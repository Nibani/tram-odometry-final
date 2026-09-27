"""Experimental TRAIN-only route geometry. Not authorized for submission until
organizer confirms that GNSS-derived offline maps are permitted.
"""
from __future__ import annotations
import numpy as np

def ecef(lla):
    a=6378137.;e2=6.6943799901413165e-3
    lat=np.deg2rad(lla[...,0]);lon=np.deg2rad(lla[...,1]);h=lla[...,2]
    n=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.stack([(n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-e2)+h)*np.sin(lat)],axis=-1)

def enu_rotation(origin):
    la,lo=np.deg2rad(origin[:2]);sl,cl=np.sin(lo),np.cos(lo);sp,cp=np.sin(la),np.cos(la)
    return np.array([[-sl,cl,0],[-sp*cl,-sp*sl,cp],[cp*cl,cp*sl,sp]])

def enu(lla,origin):return (ecef(lla)-ecef(np.asarray(origin)))@enu_rotation(origin).T

def project(poly,point):
    start=poly[:-1,1:3];vec=poly[1:,1:3]-start;v2=np.sum(vec*vec,axis=1)
    w=np.clip(np.sum((point[:2]-start)*vec,axis=1)/np.maximum(v2,1e-12),0,1)
    dist=np.sum((start+w[:,None]*vec-point[:2])**2,axis=1);i=int(np.argmin(dist))
    s=poly[i,0]+w[i]*(poly[i+1,0]-poly[i,0]);tangent=vec[i]/max(np.linalg.norm(vec[i]),1e-9)
    return float(s),float(np.sqrt(dist[i])),tangent

def lookup(poly,s):return np.column_stack([np.interp(s,poly[:,0],poly[:,j]) for j in [1,2,3]])
