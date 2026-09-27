"""Reference geometry. Runtime port lives in route.hpp. Units: metres/radians.

The rail curve contains front-bogie pivot positions. The rigid body follows
the chord joining bogie pivots, with zero roll (roll is not observable here).
"""
from pathlib import Path
import numpy as np

ORIGIN=np.array([55.805,37.425,170.])
MASTER=-9.873
ROVER=2.563
LENGTH=7.55
HEIGHT=3.
REFERENCE_POLICY='fix-monotonic-record-high-v1'

def ecef(lla):
    lla=np.asarray(lla);lat=np.deg2rad(lla[...,0]);lon=np.deg2rad(lla[...,1]);h=lla[...,2]
    a=6378137.;e2=6.6943799901413165e-3;n=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.stack(((n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-e2)+h)*np.sin(lat)),axis=-1)

def enu(lla,origin=ORIGIN):
    la,lo=np.deg2rad(origin[:2]);sl,cl=np.sin(lo),np.cos(lo);sp,cp=np.sin(la),np.cos(la)
    rot=np.array([[-sl,cl,0],[-sp*cl,-sp*sl,cp],[cp*cl,cp*sl,sp]])
    return (ecef(lla)-ecef(origin))@rot.T

def body_axes(direction):
    ex=np.asarray(direction);ex=ex/np.maximum(np.linalg.norm(ex,axis=-1,keepdims=True),1e-12)
    ey=np.stack((-ex[...,1],ex[...,0],np.zeros_like(ex[...,0])),axis=-1)
    ey/=np.maximum(np.linalg.norm(ey,axis=-1,keepdims=True),1e-12)
    ez=np.cross(ex,ey)
    return ex,ey,ez

def base_from_antennas(master,rover):
    ex,_,ez=body_axes(np.asarray(rover)-np.asarray(master))
    return np.asarray(master)-MASTER*ex-HEIGHT*ez,ex

class Route:
    def __init__(self,xyz,name=''):
        self.xyz=np.asarray(xyz,dtype=float);self.name=name
        if self.xyz.ndim!=2 or self.xyz.shape[1]!=3 or len(self.xyz)<2 or not np.isfinite(self.xyz).all():raise ValueError('Invalid route')
        good=np.r_[True,np.linalg.norm(np.diff(self.xyz,axis=0),axis=1)>1e-6];self.xyz=self.xyz[good]
        self.vec=np.diff(self.xyz,axis=0);self.ds=np.linalg.norm(self.vec,axis=1)
        if len(self.ds)==0:raise ValueError('Empty route')
        self.s=np.r_[0,np.cumsum(self.ds)];self.tang=self.vec/self.ds[:,None]
    @classmethod
    def load(cls,path):
        return cls(np.loadtxt(path,delimiter=',',skiprows=1)[:,1:4],Path(path).stem)
    def save(self,path):
        np.savetxt(path,np.column_stack([self.s,self.xyz]),delimiter=',',header='s,x,y,z',comments='',fmt='%.9f')
    def at(self,s):
        s=np.asarray(s);i=np.searchsorted(self.s,s,side='right')-1;i=np.clip(i,0,len(self.ds)-1)
        return self.xyz[i]+((s-self.s[i])/self.ds[i])[...,None]*self.vec[i]
    def pose(self,s,x=0.,z=0.):
        s=np.asarray(s);front=self.at(s);lo=s-2*LENGTH;hi=s
        # Nearest preceding rear point, valid for normal tram curve radii > L/2.
        for _ in range(26):
            mid=(lo+hi)*.5;far=np.linalg.norm(front-self.at(mid),axis=-1)>LENGTH
            lo=np.where(far,mid,lo);hi=np.where(far,hi,mid)
        ex,ey,ez=body_axes(front-self.at((lo+hi)*.5))
        return front+x*ex+z*ez,ex,ey,ez
    def horizontal_speed_factor(self,s,x=MASTER,z=HEIGHT):
        a=self.pose(np.asarray(s)-.05,x,z)[0];b=self.pose(np.asarray(s)+.05,x,z)[0]
        return np.linalg.norm((b-a)[...,:2],axis=-1)/.1
    def project(self,p,heading=None):
        p=np.asarray(p);w=np.clip(((p-self.xyz[:-1])*self.vec).sum(1)/self.ds**2,0.,1.)
        nearest=self.xyz[:-1]+w[:,None]*self.vec;cost=((nearest-p)**2).sum(1)
        if heading is not None:
            # Cheap tangent screening first; final candidate uses exact body chord.
            cosine=self.tang@np.asarray(heading)
            cost+=25.*(1.-cosine)**2
        ids=np.argsort(cost)[:8];best=None
        for i in ids:
            s=self.s[i]+w[i]*self.ds[i];pos,ex,_,_=self.pose(s)
            value=float(np.dot(pos-p,pos-p))
            if heading is not None:value+=100.*(1.-float(ex@heading))**2
            if best is None or value<best[0]:best=(value,float(s),float(np.linalg.norm(pos-p)))
        return best

def nearest_indices(h,q):
    if len(h)==0:return np.zeros(len(q),dtype=int),np.full(len(q),np.inf)
    i=np.searchsorted(h,q).clip(0,len(h)-1);j=(i-1).clip(0);i=np.where(abs(h[j]-q)<abs(h[i]-q),j,i)
    return i,abs(h[i]-q)*1e-9

def _ordered_fix(cache,name):
    """Keep record-high headers, matching the velocity-reference policy.

    Payload rows use the same mask. Do not sort late or duplicate messages back
    into the reference stream: that would define a different offline oracle.
    """
    h=cache[name+'_ht'];data=cache[name+'_data']
    if len(h)==0:return h,data
    keep=np.r_[True,h[1:]>np.maximum.accumulate(h)[:-1]]
    return h[keep],data[keep]


def paired_reference(cache,q):
    mh,mx=_ordered_fix(cache,'master_fix');rh,rx=_ordered_fix(cache,'rover_fix')
    if len(mh)==0 or len(rh)==0:return None
    mi,md=nearest_indices(mh,q);ri,rd=nearest_indices(rh,q)
    m=enu(mx[mi,:3]);r=enu(rx[ri,:3]);base,heading=base_from_antennas(m,r)
    valid=(md<=.05)&(rd<=.05)&np.isfinite(base).all(1)
    baseline=np.linalg.norm(r-m,axis=1);quality=valid&(baseline>11.9)&(baseline<12.9)
    return dict(base=base,heading=heading,master=m,rover=r,valid=valid,quality=quality,baseline=baseline)
