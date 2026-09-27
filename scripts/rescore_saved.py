"""Apply the current position oracle to immutable saved outputs; no model replay.

Speed and runtime statistics are copied from the source report. Only position
reference fields and their coverage are recomputed. The report records both
provenances explicitly. Output is a new JSON file, never an in-place edit.
"""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
from map_geometry import paired_reference,REFERENCE_POLICY
from evaluate_pipeline import sums

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def rescore(source,cache,out):
    original=source/'metrics.json';report=json.loads(original.read_text(encoding='utf-8'))
    if out.resolve()==original.resolve():raise ValueError('Keep the source report immutable')
    if out.exists():raise FileExistsError(out)
    position_keys=[prefix+label for prefix in ['base_xyz_','base_xy_','base_z_','master_xyz_'] for label in ['valid','quality']]
    sources={}
    for row in report['per_bag']:
        bag=row['bag'];trajectory=source/(bag+'.npz');raw=cache/(bag+'.npz')
        with np.load(trajectory,allow_pickle=False) as z:q=z['q'];a=z['output']
        if a.ndim!=2 or len(q)!=len(a) or a.shape[1]<14:raise ValueError('Invalid saved output: '+bag)
        with np.load(raw,allow_pickle=False) as z:ref=paired_reference(z,q)
        for key in position_keys+['paired_reference_count','quality_reference_count','uninitialized_reference_count']:row.pop(key,None)
        if ref is not None:
            flags=a[:,8].astype(int);startup=q-q[0]>=3000000000
            for label in ['valid','quality']:
                use=ref[label]&startup&((flags&8)==0)
                row['base_xyz_'+label]=sums(a[use,5:8]-ref['base'][use]);row['base_xy_'+label]=sums(a[use,5:7]-ref['base'][use,:2])
                row['base_z_'+label]=sums(a[use,7]-ref['base'][use,2]);row['master_xyz_'+label]=sums(a[use,11:14]-ref['master'][use])
            row['paired_reference_count']=int((ref['valid']&startup).sum());row['quality_reference_count']=int((ref['quality']&startup).sum())
            row['uninitialized_reference_count']=int((ref['valid']&startup&((flags&8)!=0)).sum())
        sources[bag]={'output_sha256':sha(trajectory),'cache_sha256':sha(raw)}
    for group,metrics in report['summary'].items():
        rows=[r for r in report['per_bag'] if group=='all' or r['bag'].startswith(group)]
        for key in position_keys:
            metrics.pop(key,None);values=[r[key] for r in rows if r.get(key,{}).get('n',0)>0];n=sum(v['n'] for v in values)
            if n:metrics[key]=dict(n=n,rmse=float(np.sqrt(sum(v['sse'] for v in values)/n)),mae=sum(v['sae'] for v in values)/n,
                                  macro_rmse=float(np.mean([v['rmse'] for v in values])),worst_bag_rmse=max(v['rmse'] for v in values))
    report.update(oracle_revision='3.6',reference_policy=REFERENCE_POLICY,reference_geometry_sha256=sha(Path(__file__).parent/'map_geometry.py'))
    report['rescore_provenance']={'method':'Position only; no model replay. Speed/runtime values and original scorer hash retained from source.',
        'source_report_sha256':sha(original),'rescorer_sha256':sha(Path(__file__)),'position_statistics_sha256':sha(Path(__file__).parent/'evaluate_pipeline.py'),
        'source_files':sources}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
    print(json.dumps({'part':report['part'],'reference_policy':REFERENCE_POLICY,'raw':report['summary']['all'].get('base_xyz_valid'),
                      'quality':report['summary']['all'].get('base_xyz_quality')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--cache',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();rescore(a.source,a.cache,a.out)
