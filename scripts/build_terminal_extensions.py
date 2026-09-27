"""Optional TRAIN-only terminal geometry around the official pathgraph.

This is offline map calibration, not a runtime GNSS correction. The exact source
bags are training bags. No validation or former holdout coordinates are used.
"""
import argparse,json
from pathlib import Path
import numpy as np
from map_geometry import Route,paired_reference
from physics_model import fresh_baseline

def run(work):
    config=Path(__file__).resolve().parents[1]/'config';report=[]
    for number,bag in enumerate(['30618_0652866c','30618_073f08d1']):
        route=Route.load(config/f'route_{number}.csv');z=np.load(work/'cache'/f'{bag}.npz');f=np.load(work/'features'/f'{bag}.npz')
        velocity=fresh_baseline(f['X']);distance=np.r_[0.,np.cumsum(np.diff(f['t'])*.5*(velocity[1:]+velocity[:-1]))]
        q=f['q'];ref=paired_reference(z,q);good=ref['quality'];ids=[];last=-1e9
        for i in np.flatnonzero(good):
            if distance[i]-last>=1.5:ids.append(i);last=distance[i]
        ids=np.array(ids);p=ref['base'][ids];h=ref['heading'][ids];d=distance[ids]
        projections=np.array([route.project(a,b) for a,b in zip(p,h)])
        near=(projections[:,2]<2.)&(projections[:,0]<8.)
        begin=near&(projections[:,1]<80);end=near&(projections[:,1]>route.s[-1]-80)
        if begin.sum()<5 or end.sum()<5:raise RuntimeError('Training trajectory lacks directed endpoint overlap')
        d0=float(np.median(d[begin]-projections[begin,1]));d1=float(np.median(d[end]+route.s[-1]-projections[end,1]))
        # Interpolate distance-binned robust points; short spatial smoothing removes
        # centimeter GNSS jitter while preserving terminal curvature.
        def extension(mask,lo,hi):
            dd=d[mask];pp=p[mask];grid=np.arange(lo,hi,1.)
            if len(dd)<5 or len(grid)<2:return np.empty((0,3))
            out=np.column_stack([np.interp(grid,dd,pp[:,j]) for j in range(3)])
            smooth=np.stack([np.convolve(np.pad(out[:,j],(2,2),mode='edge'),np.ones(5)/5,mode='valid') for j in range(3)],axis=1)
            return smooth
        prefix=extension(d<d0+10,max(float(d[0]),0.),d0)
        suffix=extension(d>d1-10,d1+1,float(d[-1]))
        if len(prefix):
            delta=route.xyz[0]-prefix[-1];blend=np.linspace(0,1,min(25,len(prefix)))[:,None];prefix[-len(blend):]+=blend*delta
            prefix=prefix[:-1]
        if len(suffix):
            delta=route.xyz[-1]-suffix[0];blend=np.linspace(1,0,min(25,len(suffix)))[:,None];suffix[:len(blend)]+=blend*delta
            suffix=suffix[1:]
        extended=Route(np.vstack([prefix,route.xyz,suffix]));extended.save(config/f'route_extended_{number}.csv')
        report.append(dict(route=number,train_bag=bag,official_length=float(route.s[-1]),extended_length=float(extended.s[-1]),prefix_points=len(prefix),suffix_points=len(suffix),wheel_start=d0,wheel_end=d1,source='train-only GNSS base_link geometry; static official middle unchanged',assumption='offline geometry calibration permitted; official-only mode remains available'))
    (config/'terminal_extensions.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--work',type=Path,required=True);run(a.parse_args().work)
