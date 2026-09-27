"""Portable standard-library stress checks for the supplied native core_cli ABI.

Usage: python native_speed_stress.py --core /path/core_cli[.exe] --out results.json
Optional --candidate /path/experimental_core_cli adds a paired comparison.
No ROS, dataset, NumPy, network, model training or runtime GNSS is needed.
Synthetic accuracy numbers supplement real-bag metrics; they do not replace them.
"""
from __future__ import annotations
import argparse,bisect,hashlib,json,math,struct,subprocess,tempfile
from pathlib import Path

DT=.05;START=10.;DURATION=30.;BASE=8.
N=int(DURATION/DT)+1
SCENARIOS=['clean','front_step','both_step','both_ramp_return','both_negative_ramp_return',
    'both_smooth_sine','both_missing_1s','rear_missing_whole_run','unexpected_real_acceleration',
    'real_accel_then_opposite_slip','ambiguous_constant_with_slip','ambiguous_real_accel_with_slip',
    'nonfinite_wheel','timestamp_reset']

def clip(x,a,b):return min(b,max(a,x))

def wheel_value(mode,t,side):
    q=t-START;v=BASE
    if mode in ['unexpected_real_acceleration','real_accel_then_opposite_slip']:
        v+=clip(.5*q,0.,1.5)
    if mode=='real_accel_then_opposite_slip' and 4<=q<5:v-=3.
    if mode=='both_step' or (mode=='front_step' and side==0):
        if 0<=q<1:v+=3.
    if mode in ['both_ramp_return','both_negative_ramp_return'] and 0<=q<3:
        v+=(1 if mode=='both_ramp_return' else -1)*.5*q
    if mode=='both_smooth_sine' and 0<=q<6:v+=1.5*math.sin(math.pi*q/6)**2
    # Identical observable inputs in two physically distinct hidden worlds.
    if mode in ['ambiguous_constant_with_slip','ambiguous_real_accel_with_slip'] and 0<=q<3:
        v+=.5*q
    return v

def truth_value(mode,t):
    if mode in ['unexpected_real_acceleration','real_accel_then_opposite_slip','ambiguous_real_accel_with_slip']:
        return BASE+clip(.5*(t-START),0.,1.5)
    return BASE

def make_inputs(mode):
    histories=[([],[]),([],[])];records=[];times=[];truth=[]
    for k in range(N):
        t=k*DT;wheel=[]
        for side in range(2):
            tt,vv=histories[side]
            present=not(mode=='both_missing_1s' and START<=t<START+1) and not(mode=='rear_missing_whole_run' and side==1)
            if present:tt.append(t);vv.append(wheel_value(mode,t,side))
            if not tt:wheel.append((0.,10.,0.,[0.,0.,0.],[0.,0.,0.],0.));continue
            age=t-tt[-1];value=vv[-1]
            short=clip((vv[-1]-vv[-2])/(tt[-1]-tt[-2]),-4,4) if len(tt)>1 else 0.
            slopes=[];lags=[]
            for sec in [.3,1.,3.]:
                j=max(0,bisect.bisect_right(tt,tt[-1]-sec)-1);delta=tt[-1]-tt[j]
                slopes.append(clip((vv[-1]-vv[j])/delta,-4,4) if delta>1e-6 else 0.);lags.append(vv[j])
            wheel.append((value,min(age,10.),short,slopes,lags,max(0.,value+min(age,.3)*slopes[0])))
        f,af,df,sf,lf,ef=wheel[0];r,ar,dr,sr,lr,er=wheel[1]
        x=[f,r,(f+r)/2,f-r,abs(f-r),af,ar,df,dr,sf[0],sr[0],sf[1],sr[1],sf[2],sr[2],ef,er,(ef+er)/2,0.,0.,0.,0.,lf[1],lr[1],lf[2],lr[2]]
        stamp=t
        if mode=='nonfinite_wheel' and START<=t<START+1:x[15]=float('nan')
        if mode=='timestamp_reset' and t>=START:stamp=t-START
        records.append(struct.pack('<d26f',stamp,*x));times.append(t);truth.append(truth_value(mode,t))
    return b''.join(records),times,truth

def run(core,payload,folder):
    inp=folder/'input.bin';out=folder/'output.bin';inp.write_bytes(payload)
    p=subprocess.run([str(core),str(inp),str(out)],capture_output=True,text=True,check=True)
    data=out.read_bytes();assert len(data)==N*8,(len(data),N)
    values=list(struct.unpack('<'+'d'*N,data))
    assert all(math.isfinite(v) and v>=0 for v in values),'Nonfinite or negative prediction'
    return values,json.loads(p.stderr.strip().splitlines()[-1])

def metric(values,truth,times):
    err=[v-y for v,y,t in zip(values,truth,times) if START<=t<START+9]
    return {'n':len(err),'rmse':math.sqrt(sum(e*e for e in err)/len(err)),
        'mae':sum(abs(e) for e in err)/len(err),'max_abs':max(abs(e) for e in err)}

def main():
    p=argparse.ArgumentParser();p.add_argument('--core',type=Path,required=True);p.add_argument('--candidate',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    cores={'baseline':a.core.resolve()}
    if a.candidate:cores['candidate']=a.candidate.resolve()
    rows=[];ambiguous={}
    with tempfile.TemporaryDirectory(prefix='tram_native_stress_') as td:
        folder=Path(td)
        for mode in SCENARIOS:
            payload,times,truth=make_inputs(mode)
            for name,core in cores.items():
                vals,timing=run(core,payload,folder);m=metric(vals,truth,times)
                row={'scenario':mode,'implementation':name,'input_sha256':hashlib.sha256(payload).hexdigest(),'finite_nonnegative':True,**m,'kernel_timing':timing};rows.append(row)
                print(mode,name,round(m['rmse'],9),flush=True)
                if mode.startswith('ambiguous_'):ambiguous[(name,mode)]=(payload,vals)
                if mode in ['clean','unexpected_real_acceleration']:assert m['rmse']<.08,(mode,name,m)
        for name in cores:
            pa,va=ambiguous[(name,'ambiguous_constant_with_slip')];pb,vb=ambiguous[(name,'ambiguous_real_accel_with_slip')]
            assert pa==pb and va==vb,'Identical observations must produce identical native outputs'
    result={'status':'PASS_STRUCTURAL_CONTROLS_WITH_DECLARED_ACCURACY_FAILURES','input_frequency_hz':1/DT,
        'synthetic_speed_unit':'m/s; core feature ABI after configurable raw-wheel normalization',
        'limitation':'No arbitrary upper accuracy threshold is imposed on ambiguous slip fixtures; actual errors must be reported.',
        'indistinguishability':'Both ambiguous cases have byte-identical wheel/controller features. Their true velocities differ by1.5m/s after13s. Every estimator must have >=.75m/s absolute error in at least one of these two worlds at each such instant. Startup and pre-anomaly histories are identical.',
        'rows':rows}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':main()
