"""Fixed sparse-input regression on the existing exposed validation/audit sets."""
from pathlib import Path
import os,sys,json,subprocess,io,hashlib,time,ctypes
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
if os.name=='nt':ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(),0x4000)
import numpy as np
import argparse
p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--exe',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--offmap-departure',type=float,default=0.);args=p.parse_args()
S=Path(__file__).resolve().parents[1];O=args.out.resolve();O.mkdir(parents=True,exist_ok=True);assert not any(O.iterdir()), 'Output directory must be empty'
sys.path.insert(0,str(S/'scripts'))
from evaluate_pipeline import events,sums
from map_geometry import paired_reference
split=json.loads((S/'config/split_v1.json').read_text());exe=args.exe.resolve()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan={'initial_window_s':3,'master_burst_s':1,'master_period_s':120,'master_first_s':120,'rover_first_s':60,'rover_period_s':120,'rover_burst_s':1,
      'selection':'all22validation and4exposed audit; no fitting or tuning','scoring':'legacy paired GNSS geometry, quality and raw; after first3s; distinct from official kinematic checker',
      'exe_sha256':sha(exe),'script_sha256':sha(__file__),'split_sha256':sha(S/'config/split_v1.json')}
(O/'plan.json').write_text(json.dumps(plan,indent=2));rows=[]
for bag,meta in split.items():
    if meta['part'] not in ('validation','holdout'):continue
    source=args.cache/f'{bag}.npz'
    with np.load(source) as data:z={k:data[k] for k in data.files}
    masked=dict(z);t0=z['cmd_ht'][0];counts={}
    for side,offset in [('master',120),('rover',60)]:
        name=side+'_fix';elapsed=(z[name+'_ht']-t0)*1e-9
        mask=(elapsed<=3)|((elapsed>=offset)&((elapsed-offset)%120<1))
        for suffix in ['_ht','_bt','_data']:masked[name+suffix]=z[name+suffix][mask]
        counts[side]=int(mask.sum())
    text=events(masked);base=None;row={'bag':bag,'part':meta['part'],'date':meta['date'],'cache_sha256':sha(source),'input_fix_counts':counts,'arms':{}}
    for name,flags in [('off',[]),('phase',['--sparse-gnss']),('on',['--sparse-gnss','--xy-residual'])]:
        cmd=[str(exe),str(S/'config/route_loop.csv'),str(S/'config/route_hairpin_hypothesis.csv'),'--loop','--speed-scale','1.0003350854241781',*flags]
        if name=='on' and args.offmap_departure>0:cmd+=['--offmap-departure',str(args.offmap_departure)]
        begin=time.perf_counter();p=subprocess.run(cmd,input=text,text=True,capture_output=True,check=True,timeout=90)
        a=np.loadtxt(io.StringIO(p.stdout),ndmin=2);q=np.array([int(line.split(' ',1)[0]) for line in p.stdout.splitlines()],dtype=np.int64)
        if base is None:base=a
        else:assert np.array_equal(a[:,[0,1,2,3]],base[:,[0,1,2,3]]),'GNSS changed core/distance'
        ref=paired_reference(z,q);init=(a[:,8].astype(int)&8)==0;startup=q-q[0]>=3_000_000_000
        scores={'n_outputs':len(a),'initialized':int(init.sum()),'elapsed_s':time.perf_counter()-begin}
        if ref is not None:
            for kind in ['valid','quality']:
                use=ref[kind]&startup&init;scores['xyz_'+kind]=sums(a[use,5:8]-ref['base'][use])
        row['arms'][name]=scores
        np.savez_compressed(O/f'{bag}_{name}.npz',q=q,output=a)
    rows.append(row);print(bag,round(row['arms']['off'].get('xyz_quality',{}).get('rmse',float('nan')),3),round(row['arms']['on'].get('xyz_quality',{}).get('rmse',float('nan')),3),flush=True)
summary={}
for part in ['validation','holdout']:
    for vehicle in ['all','30618','30639']:
        group=[r for r in rows if r['part']==part and (vehicle=='all' or r['bag'].startswith(vehicle))]
        if not group:continue
        arm_summary={}
        for arm in ['off','phase','on']:
            arm_summary[arm]={}
            for metric in ['xyz_valid','xyz_quality']:
                vals=[r['arms'][arm][metric] for r in group if r['arms'][arm].get(metric,{}).get('n',0)>0]
                n=sum(v['n'] for v in vals)
                if n:arm_summary[arm][metric]={'n':n,'rmse':float(np.sqrt(sum(v['sse'] for v in vals)/n)),'macro_rmse':float(np.mean([v['rmse'] for v in vals])),'max':max(v['max'] for v in vals)}
        summary[part+'_'+vehicle]=arm_summary
report={'plan':plan,'rows':rows,'summary':summary,'core_speed_distance_invariance':'PASS','exposed_data':True}
(O/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(summary,indent=2),flush=True)
