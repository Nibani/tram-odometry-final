"""Join directed TRAIN terminal extensions with smooth, explicitly approximate seams."""
from pathlib import Path
import json
import numpy as np
from map_geometry import Route

def join(a,b,width=30.):
    p0=a.at(a.s[-1]-width);p1=b.at(width)
    u=(a.at(a.s[-1]-width+1)-a.at(a.s[-1]-width-1))/2
    v=(b.at(width+1)-b.at(width-1))/2
    length=np.linalg.norm(p1-p0);u=u/np.linalg.norm(u)*length;v=v/np.linalg.norm(v)*length
    t=np.linspace(0,1,max(3,int(length)+1))[:,None]
    curve=(2*t**3-3*t**2+1)*p0+(t**3-2*t**2+t)*u+(-2*t**3+3*t**2)*p1+(t**3-t**2)*v
    return curve

def main():
    config=Path(__file__).resolve().parents[1]/'config';a=Route.load(config/'route_extended_0.csv');b=Route.load(config/'route_extended_1.csv');width=30.
    middle_a=a.xyz[(a.s>=width)&(a.s<a.s[-1]-width)];middle_b=b.xyz[(b.s>=width)&(b.s<b.s[-1]-width)]
    ab=join(a,b,width);ba=join(b,a,width)
    points=np.vstack([a.at(width),middle_a,ab,middle_b,ba]);points[-1]=points[0]
    loop=Route(points);loop.save(config/'route_loop.csv')
    report={'length_m':float(loop.s[-1]),'join_width_each_side_m':width,'method':'zero-roll rigid body on directed closed curve; cubic Hermite terminal joins between TRAIN-only extensions','uncertainty':'terminal rail connectivity is inferred from training tracks; seam geometry is an approximation, not organizer-surveyed track','seam_raw_gaps_m':[float(np.linalg.norm(a.xyz[-1]-b.xyz[0])),float(np.linalg.norm(b.xyz[-1]-a.xyz[0]))]}
    (config/'route_loop_meta.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
