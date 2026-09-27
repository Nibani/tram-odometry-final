"""Build a local DDS replay fixture from decoded organizer data; no network access."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--decoded',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if a.out.exists():raise SystemExit('Output already exists; choose a new fixture path')
arrays={}
with np.load(a.decoded/'inputs.npz',allow_pickle=False) as src:
    for kind in ('front','rear','cmd','master_fix','rover_fix'):
        for suffix in ('ht','bt','id','data'):arrays[kind+'_'+suffix]=src[kind+'_'+suffix].copy()
with np.load(a.decoded/'reference.npz',allow_pickle=False) as src:
    for suffix in ('ht','bt','id'):arrays['reference_'+suffix]=src['reference_'+suffix].copy()
    arrays['reference_data']=src['reference_data'][:,list(range(7))+list(range(43,49))].copy()
a.out.parent.mkdir(parents=True,exist_ok=True)
np.savez_compressed(a.out,**arrays)
manifest=json.loads((a.decoded/'source_manifest.json').read_text())
provenance={'source_bag_sha256':manifest['bag_sha256'],'fixture_sha256':hashlib.sha256(a.out.read_bytes()).hexdigest(),'publication':'Local user-supplied fixture; evaluation reference is isolated from estimator inputs','counts':{k:len(arrays[k+'_ht']) for k in ('front','rear','cmd','master_fix','rover_fix','reference')}}
a.out.with_name('provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
print(a.out)
