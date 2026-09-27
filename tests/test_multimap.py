"""Behavioral checks for first3s selection, ties and postwindow isolation."""
import sys,json,hashlib,argparse
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;C=HERE.parent
sys.path.insert(0,str(C/'tests'))
from test_pipeline import invoke,lla_from_enu,Route

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--exe',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args();exe=args.exe;out=args.out;out.mkdir(parents=True,exist_ok=True)
    primary=out/'primary.csv';shifted=out/'shifted.csv';ambiguous=out/'ambiguous.csv'
    Route([[0,0,0],[250,0,0]]).save(primary)
    Route([[0,5,0],[250,5,0]]).save(shifted)
    Route([[-999.732,0,0],[-.357,0,0],[85,0,0],[105,2,0],[140,20,0],[250,20,0]]).save(ambiguous)
    t0=10_000_000_000
    def events(y,poison=False):
        rows=[]
        for i in range(401):
            stamp=t0+i*50_000_000;rows.extend([f'F {stamp} {stamp} 18\n',f'R {stamp} {stamp} 18\n'])
            if i%2==0 and (i<=56 or poison and i>60):
                for key,x in [('M',-9.873),('G',2.563)]:
                    # Plausible post-window fixes lie exactly on the competing map.
                    # They would be admissible projections if the freeze leaked.
                    pos=lla_from_enu([50+5*i*.05+x,y if i<=56 else 5-y,3]);rows.append(f'{key} {stamp} {stamp} '+' '.join(format(v,'.17g') for v in pos)+' 0\n')
            rows.append(f'C {stamp} {stamp} 0\n')
        return ''.join(rows)
    passed=[]
    for truth,index in [(0,0),(5,1)]:
        text=events(truth);a=invoke(exe,text,[primary,shifted]);m=a[:,0]>=t0+3_000_000_000
        assert np.all(a[m,9]==index)
        assert np.max(abs(a[m,6]-truth))<.005
        b=invoke(exe,events(truth,True),[primary,shifted]);assert np.array_equal(a,b)
        prefix=''.join(text.splitlines(keepends=True)[:100]);b=invoke(exe,prefix,[primary,shifted]);assert np.array_equal(a[:len(b)],b)
        passed.append(f'clear_route_{index}_prefix_and_late_gnss')
    text=events(0);a=invoke(exe,text,[primary,ambiguous]);single=invoke(exe,text,[primary]);m=a[:,0]>=t0+3_000_000_000
    assert np.all(a[m,9]==0),'unobserved future branch changed selected map'
    assert np.array_equal(a,single),'unobservable initial tie did not retain primary'
    passed.append('indistinguishable_prefix_different_future_keeps_primary')
    report={'passed':passed,'exe_sha256':hashlib.sha256(exe.read_bytes()).hexdigest()};(out/'properties.json').write_text(json.dumps(report,indent=2));print(report)

if __name__=='__main__':main()
