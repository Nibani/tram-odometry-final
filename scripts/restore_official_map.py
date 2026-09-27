"""Apply the frozen TRAIN-only rigid calibration to organizer Pathgraph JSON files."""
import argparse,json
from pathlib import Path
import numpy as np
def main():
    p=argparse.ArgumentParser();p.add_argument('--pathgraphs',nargs=2,type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--calibration',type=Path,default=Path(__file__).resolve().parents[1]/'config/map_calibration.json');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    fit=json.loads(a.calibration.read_text());rot=np.array(fit['enu_to_map_R']);offset=np.array(fit['enu_to_map_translation'])
    for i,path in enumerate(a.pathgraphs):
        d=json.loads(path.read_text(encoding='utf-8-sig'));xyz=np.array([[p[k] for k in ('x','y','z')] for p in d['points']]);xyz[:,:2]=(xyz[:,:2]-offset)@rot;xyz[:,2]-=fit['map_z_minus_enu_up']
        s=np.r_[0.,np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).cumsum()]
        # Stored legacy s is horizontal; runtime always recomputes true 3D arclength.
        np.savetxt(a.out/f'route_{i}.csv',np.column_stack([s,xyz]),delimiter=',',header='s,x,y,z',comments='',fmt='%.9f')
if __name__=='__main__':main()
