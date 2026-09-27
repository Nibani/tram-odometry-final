"""Prepare/seal fixed input perturbations, then replay immutable native runtime."""
from pathlib import Path
import argparse,collections,ctypes,hashlib,json,math,os,subprocess,sys,time
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
sys.dont_write_bytecode=True
if os.name=='nt':ctypes.windll.kernel32.SetPriorityClass(ctypes.c_void_p(-1),0x4000)

ARMS=('provided','no_late','even_bursts','odd_bursts','outlier_north','bias_north')
KINDS={'front':'F','rear':'R','cmd':'C','master_fix':'M','rover_fix':'G'}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(path,obj):Path(path).write_text(json.dumps(obj,indent=2,allow_nan=False),encoding='utf-8')
def external(args):
    return {**{n:args.decoded/n for n in ['events.txt','inputs.npz','reference.npz','source_manifest.json','map_calibration.json','official_metrics.py']},'scorer':args.harness,'script':Path(__file__),'plan':Path(__file__).with_name('PLAN.md')}

def prepare(args):
    import numpy as np
    out=args.out;out.mkdir(parents=True,exist_ok=False);rt=out/'runtime';rt.mkdir()
    # Pin bytes: execution never uses a mutable external executable or map.
    sources={'pipeline_cli.exe':args.candidate/'build/pipeline_cli.exe',
        **{n:args.candidate/'config'/n for n in ['route_loop.csv','route_hairpin_hypothesis.csv','map_calibration.json']}}
    for name,p in sources.items():(rt/name).write_bytes(p.read_bytes())
    raw=(args.decoded/'events.txt').read_bytes();lines=raw.decode('ascii').splitlines(keepends=True)
    rows=[line.split() for line in lines]
    assert all(r[0] in KINDS.values() for r in rows)
    first_command=next(int(r[1]) for r in rows if r[0]=='C');boundary=first_command+3_000_000_000
    # Validate provenance order from input metadata only; no reference arrays opened.
    records=[]
    with np.load(args.decoded/'inputs.npz',allow_pickle=False) as z:
        for alias,kind in KINDS.items():
            for mid,ht,bt in zip(z[alias+'_id'],z[alias+'_ht'],z[alias+'_bt']):records.append((int(bt),int(mid),kind,int(ht)))
    records.sort()
    assert len(records)==len(rows)
    for r,(bt,mid,kind,ht) in zip(rows,records):assert (r[0],int(r[1]),int(r[2]))==(kind,ht,bt)
    late=sorted((int(r[1]),i) for i,r in enumerate(rows) if r[0] in ['M','G'] and int(r[1])>boundary)
    burst_for={};bursts=[];last=None
    for ht,i in late:
        if last is None or ht-last>2_000_000_000:bursts.append({'burst_id':len(bursts),'start_header_ns':ht,'end_header_ns':ht,'source_line_indices':[],'receiver_counts':{'M':0,'G':0}})
        b=bursts[-1];b['end_header_ns']=ht;b['source_line_indices'].append(i);b['receiver_counts'][rows[i][0]]+=1;burst_for[i]=b['burst_id'];last=ht
    manifest={'artificial_stress':True,'exposed_bag':True,'first_command_header_ns':first_command,'initial_boundary_inclusive_ns':boundary,
        'burst_gap_strict_ns':2_000_000_000,'bursts':bursts,'arms':{},'reference_used_to_construct_arms':False,
        'runtime_origin':{n:{'path':str(p.resolve()),'sha256':sha(p)} for n,p in sources.items()},
        'source_origin':{n:sha(args.candidate/n) for n in ['src/reserve_odometry/include/reserve_odometry/pipeline.hpp','src/reserve_odometry/src/pipeline_cli.cpp','src/reserve_odometry/src/odometry_node.cpp']}}
    for arm in ARMS:
        target=out/arm;target.mkdir();kept=[];output=[];injected=[]
        counts={kind:{'initial':0,'late_kept':0,'late_dropped':0,'injected':0} for kind in ['M','G']}
        for i,(r,line) in enumerate(zip(rows,lines)):
            kind=r[0];isfix=kind in ['M','G'];islate=isfix and int(r[1])>boundary;keep=True;value=line
            if islate:
                if arm=='no_late':keep=False
                elif arm=='even_bursts':keep=burst_for[i]%2==0
                elif arm=='odd_bursts':keep=burst_for[i]%2==1
                if keep and arm in ['outlier_north','bias_north']:
                    tokens=r.copy();tokens[3]=format(float(tokens[3])+(.001 if arm=='outlier_north' else 3./111320.),'.17g')
                    # Original generated events use a single-space delimiter.
                    assert line.rstrip('\r\n')==' '.join(r)
                    end=line[len(line.rstrip('\r\n')):];value=' '.join(tokens)+end;injected.append(i)
                    assert value.split()[:3]==r[:3] and value.split()[4:]==r[4:]
                counts[kind]['late_kept' if keep else 'late_dropped']+=1
                if i in injected:counts[kind]['injected']+=1
            elif isfix:counts[kind]['initial']+=1
            if keep:output.append(value);kept.append(i)
        content=''.join(output).encode('ascii');(target/'events.txt').write_bytes(content)
        if arm=='provided':assert content==raw
        assert kept==sorted(kept)
        np.savez_compressed(target/'input_order.npz',source_line_index=np.asarray(kept,dtype=np.int64),record_ns=np.asarray([records[i][0] for i in kept],dtype=np.int64),message_id=np.asarray([records[i][1] for i in kept],dtype=np.int64),injected_source_line_index=np.asarray(injected,dtype=np.int64))
        manifest['arms'][arm]={'counts':counts,'n_events':len(kept),'n_dropped':len(rows)-len(kept),'n_injected':len(injected),'events_sha256':sha(target/'events.txt'),'input_order_sha256':sha(target/'input_order.npz')}
    save(out/'design.json',manifest)
    files={str(p.relative_to(out)).replace('\\','/'):sha(p) for p in out.rglob('*') if p.is_file()}
    save(out/'seal.json',{'prepared_before_outcomes':True,'local':files,'external':{n:sha(p) for n,p in external(args).items()}})
    print(json.dumps({'prepared':str(out),'seal_sha256':sha(out/'seal.json'),'burst_count':len(bursts),'arms':manifest['arms']}),flush=True)

def run(args):
    import numpy as np
    start=time.perf_counter();out=args.out;seal=json.loads((out/'seal.json').read_text());assert not (out/'report.json').exists()
    for n,h in seal['local'].items():assert sha(out/n)==h,n
    for n,p in external(args).items():assert sha(p)==seal['external'][n],n
    design=json.loads((out/'design.json').read_text());rt=out/'runtime';cal=json.loads((rt/'map_calibration.json').read_text());R=cal['enu_to_map_R']
    cmd=[str(rt/'pipeline_cli.exe'),str(rt/'route_loop.csv'),str(rt/'route_hairpin_hypothesis.csv'),'--loop','--speed-scale','1.0003350854241781','--map-transform',repr(math.atan2(R[1][0],R[0][0])),*map(str,cal['enu_to_map_translation']),str(cal['map_z_minus_enu_up']),'--sparse-gnss','--xy-residual']
    report={'schema':'fixed-gnss-stress-v1','artificial_stress':True,'exposed_bag':True,'acceptance_claim':False,'seal_sha256':sha(out/'seal.json'),'command':cmd,'arms':{},'assertions':{},'failed_assertions':[]}
    arrays={};stamps={};reference_hash=sha(args.decoded/'reference.npz')
    for arm in ARMS:
        target=out/arm;begin=time.perf_counter()
        with (target/'events.txt').open('rb') as source,(target/'predictions.txt').open('wb') as result:
            p=subprocess.run(cmd,stdin=source,stdout=result,stderr=subprocess.PIPE,timeout=30)
        (target/'runtime.log').write_bytes(p.stderr);assert p.returncode==0,(arm,p.stderr)
        a=np.loadtxt(target/'predictions.txt',ndmin=2);arrays[arm]=a
        stamps[arm]=np.asarray([int(line.split(' ',1)[0]) for line in (target/'predictions.txt').read_text().splitlines()],dtype=np.int64)
        scorecmd=[sys.executable,'-B',str(args.harness),'score','--decoded',str(args.decoded),'--predictions',str(target/'predictions.txt'),'--out',str(target/'score_0ms.json'),'--solution-frame','map','--absolute-position-only','--latency-ms','0']
        p=subprocess.run(scorecmd,capture_output=True,text=True,timeout=30);(target/'score.log').write_text(p.stdout+p.stderr);assert p.returncode==0,(arm,p.stderr)
        score=json.loads((target/'score_0ms.json').read_text());report['arms'][arm]={'inputs':design['arms'][arm],'score_command':scorecmd,'score':score,'elapsed_s':time.perf_counter()-begin,'prediction_sha256':sha(target/'predictions.txt')}
        print(arm,json.dumps({'xyz':score['metrics']['all']['position']['distance'],'speed':score['metrics']['all']['velocity_scalar'],'counts':design['arms'][arm]['counts']}),flush=True)
    def check(name,value):
        report['assertions'][name]=bool(value)
        if not value:report['failed_assertions'].append(name)
    base=arrays['provided'];base_score=report['arms']['provided']['score']
    for arm,a in arrays.items():
        check(arm+'_timestamps_exact',np.array_equal(stamps[arm],stamps['provided']))
        check(arm+'_scalar_core_distance_exact',np.array_equal(a[:,[1,2,3]],base[:,[1,2,3]]))
        check(arm+'_initialized_mask_exact',np.array_equal(a[:,8].astype(np.int64)&8,base[:,8].astype(np.int64)&8))
        check(arm+'_coverage_exact',report['arms'][arm]['score']['coverage']==base_score['coverage'])
        check(arm+'_initial_publication_exact',report['arms'][arm]['score']['position_publication']==base_score['position_publication'])
    check('outlier_equals_no_late_position_exact',np.array_equal(arrays['outlier_north'][:,5:8],arrays['no_late'][:,5:8]))
    check('outlier_equals_no_late_all_outputs_exact',np.array_equal(arrays['outlier_north'],arrays['no_late']))
    check('reference_bytes_unchanged',reference_hash==sha(args.decoded/'reference.npz'))
    report['elapsed_s']=time.perf_counter()-start;report['status']='STRESS_INVARIANTS_PASS' if not report['failed_assertions'] else 'STRESS_INVARIANTS_FAIL'
    report['limitations']=['Artificial fixed perturbations on an exposed bag; no acceptance claim.','Native0ms assumed publication latency, not actual ROS/DDS timing.','Reference retained unchanged and used only by scorer.','No GNSS runtime accept/reject counters exported; outlier rejection assessed through output invariance.','Artificial burst masking uses future input schedule only to define stress arms; estimator remains causal.']
    save(out/'report.json',report)
    save(out/'result_hashes.json',{str(p.relative_to(out)).replace('\\','/'):sha(p) for p in out.rglob('*') if p.is_file() and p.name!='result_hashes.json'})
    print(report['status'],json.dumps(report['failed_assertions']),report['elapsed_s'],flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--decoded',type=Path,required=True);p.add_argument('--harness',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    for key in ['candidate','decoded','harness','out']:setattr(args,key,getattr(args,key).resolve())
    (prepare if args.mode=='prepare' else run)(args)
if __name__=='__main__':main()
