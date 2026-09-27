"""Arrival-ordered native pipeline replay, with isolated GNSS reference scoring."""
import argparse,io,json,subprocess,time,hashlib
from pathlib import Path
import numpy as np
from map_geometry import paired_reference,REFERENCE_POLICY

def events(z,poison_after=None):
    lines=[]
    for name,kind in [('front','F'),('rear','R'),('cmd','C'),('master_fix','M'),('rover_fix','G')]:
        for h,b,data in zip(z[name+'_ht'],z[name+'_bt'],z[name+'_data']):
            if name.endswith('_fix'):
                if poison_after is not None and h>poison_after:data=np.array([0.,0.,0.,*data[3:]])
                text=f'{kind} {h} {b} {data[0]:.15g} {data[1]:.15g} {data[2]:.15g} {int(data[12])}\n'
            else:text=f'{kind} {h} {b} {data[0]:.15g}\n'
            lines.append((int(b),text))
    lines.sort(key=lambda x:x[0]);return ''.join(v for _,v in lines)

def replay(exe,z,routes,base_speed=False):
    cmd=[str(exe),*map(str,routes)]+(['--base-speed'] if base_speed else [])
    t=time.perf_counter();r=subprocess.run(cmd,input=events(z),capture_output=True,text=True,check=True)
    out=np.loadtxt(io.StringIO(r.stdout),ndmin=2);q=np.array([int(line.split(' ',1)[0]) for line in r.stdout.splitlines()],dtype=np.int64);return q,out,time.perf_counter()-t

def sums(error):
    e=np.asarray(error)
    if len(e)==0:return {'n':0}
    if e.ndim==1:e=e[:,None]
    return dict(n=len(e),sse=float((e*e).sum()),sae=float(np.linalg.norm(e,axis=1).sum()),bias=e.mean(0).tolist(),rmse=float(np.sqrt((e*e).sum()/len(e))),mae=float(np.linalg.norm(e,axis=1).mean()),p95=float(np.quantile(np.linalg.norm(e,axis=1),.95)),max=float(np.linalg.norm(e,axis=1).max()),endpoint=float(np.linalg.norm(e[-1])))

def evaluate(work,exe,part,mode,outdir,align_initial=False,speed_scale=1.):
    root=Path(__file__).resolve().parents[1];split=json.loads((root/'config/split_v1.json').read_text());outdir.mkdir(parents=True,exist_ok=True)
    routes=[root/'config'/f'route_{"extended_" if mode=="extended" else ""}{i}.csv' for i in range(2)]
    if mode=='loop':routes=[root/'config/route_loop.csv','--loop']
    if mode=='hypotheses':routes=[root/'config/route_loop.csv',root/'config/route_hairpin_hypothesis.csv','--loop']
    if align_initial:routes+=['--align-initial']
    if speed_scale!=1.:routes+=['--speed-scale',str(speed_scale)]
    rows=[]
    for bag,meta in split.items():
        if meta['part']!=part:continue
        with np.load(work/'cache'/f'{bag}.npz') as z:
            q,a,elapsed=replay(exe,z,routes);flags=a[:,8].astype(int)
            with np.load(work/'features'/f'{bag}.npz') as f:
                ix=np.searchsorted(f['q'],q);assert np.array_equal(f['q'][ix],q)
                y=f['y'][ix];startup=q-q[0]>=3000000000;ok=np.isfinite(y)&startup
                ym=f['y_master'][ix];yr=f['y_rover'][ix];mok=np.isfinite(ym)&startup;rok=np.isfinite(yr)&startup
            row=dict(bag=bag,part=part,mode=mode,count=len(a),elapsed_s=elapsed,init_fraction=float(np.mean((flags[startup]&8)==0)) if startup.any() else 0.,outside_fraction=float(np.mean((flags[startup]&4)!=0)) if startup.any() else 0.,route=int(a[-1,9]),speed_bogie=sums(a[ok,2]-y[ok]),speed_master=sums(a[ok,1]-y[ok]))
            row.update(startup_excluded_count=int((~startup).sum()),speed_reference_missing_count=int((startup&~np.isfinite(y)).sum()),degraded_init_fraction=float(np.mean((flags[startup]&32)!=0)) if startup.any() else 0.,speed_vs_master=sums(a[mok,1]-ym[mok]),speed_vs_rover=sums(a[rok,1]-yr[rok]),gnss_speed_disagreement=sums(ym[mok&rok]-yr[mok&rok]))
            ref=paired_reference(z,q)
            if ref is not None:
                for label in ['valid','quality']:
                    # Relative fallback is in another frame: never score it as ENU.
                    use=ref[label]&startup&((flags&8)==0)
                    row['base_xyz_'+label]=sums(a[use,5:8]-ref['base'][use]);row['base_xy_'+label]=sums(a[use,5:7]-ref['base'][use,:2]);row['base_z_'+label]=sums(a[use,7]-ref['base'][use,2]);row['master_xyz_'+label]=sums(a[use,11:14]-ref['master'][use])
                row['paired_reference_count']=int((ref['valid']&startup).sum());row['quality_reference_count']=int((ref['quality']&startup).sum());row['uninitialized_reference_count']=int((ref['valid']&startup&((flags&8)!=0)).sum())
            rows.append(row);np.savez_compressed(outdir/f'{bag}.npz',output=a,q=q)
            print(bag,'route',row['route'],'init',round(row['init_fraction'],3),'XYZ',round(row.get('base_xyz_quality',{}).get('rmse',float('nan')),3),flush=True)
    keys=['speed_bogie','speed_master','speed_vs_master','speed_vs_rover','gnss_speed_disagreement','base_xyz_valid','base_xyz_quality','base_xy_valid','base_xy_quality','base_z_valid','base_z_quality','master_xyz_valid','master_xyz_quality'];summary={}
    for group in ['all','30618','30639']:
        records=[r for r in rows if group=='all' or r['bag'].startswith(group)]
        summary[group]={}
        for k in keys:
            vals=[r[k] for r in records if r.get(k,{}).get('n',0)>0];n=sum(v['n'] for v in vals)
            if n:summary[group][k]={'n':n,'rmse':float(np.sqrt(sum(v['sse'] for v in vals)/n)),'mae':sum(v['sae'] for v in vals)/n,'macro_rmse':float(np.mean([v['rmse'] for v in vals])),'worst_bag_rmse':max(v['rmse'] for v in vals)}
    report=dict(part=part,mode=mode,align_initial=align_initial,speed_scale=speed_scale,scorer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),route_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in routes if isinstance(p,Path)},summary=summary,per_bag=rows)
    report.update(oracle_revision='3.6',reference_policy=REFERENCE_POLICY,
                  reference_geometry_sha256=hashlib.sha256((root/'scripts/map_geometry.py').read_bytes()).hexdigest())
    (outdir/'metrics.json').write_text(json.dumps(report,indent=2));print(json.dumps(summary['all'],indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--exe',type=Path,required=True);p.add_argument('--part',choices=['train','validation','holdout'],required=True);p.add_argument('--mode',choices=['official','extended','loop','hypotheses'],required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--align-initial',action='store_true');p.add_argument('--speed-scale',type=float,default=1.);a=p.parse_args();evaluate(a.work,a.exe,a.part,a.mode,a.out,a.align_initial,a.speed_scale)
