from __future__ import annotations
import sys,struct,json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from audit_data import decode,align
from prepare_features import build
from oracle_v1 import metrics
from observer_v3 import observer_v3
from physics_model import fresh_baseline
from tree_runtime import load_arrays

CACHE=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/mnt/data/tram_work/cache')
results=[]
def checked(name, fn):
    fn();results.append({'test':name,'status':'PASS'});print('PASS',name,flush=True)

def encode(alias,frame='base_link', value=12.345):
    s=frame.encode()+b'\0';b=bytearray(b'\x00\x01\x00\x00'+struct.pack('<iII',123456,789,len(s))+s)
    if alias=='cmd':b.extend(struct.pack('<b',-15))
    else:
        b.extend(bytes(align(len(b),8)-len(b)));b.extend(struct.pack('<d',value))
    return b

def cdr():
    for alias in ('front','rear','cmd'):
        for frame in ('','x','base_link','0123456789abcdef'):
            h,v=decode(encode(alias,frame),alias)
            assert h==123456000000789 and v[0]==(-15 if alias=='cmd' else 12.345)
    for bad in (b'1234'+encode('front')[4:], encode('front')[:12]+b'\0'*4+encode('front')[16:]):
        try: decode(bad,'front')
        except (ValueError,struct.error): pass
        else: raise AssertionError('Malformed CDR accepted')

def subset(z,cut):
    d={}
    for n in ['front','rear','cmd','master_vel','rover_vel','master_fix','rover_fix']:
        m=z[n+'_bt']<=cut
        for suf in ('ht','bt','data'):d[n+'_'+suf]=z[n+'_'+suf][m].copy()
    return d

def causal():
    maxdiff=0.;checks=0
    for bag in ('30618_0e41eac3','30639_d927f360','30618_2050d396'):
        with np.load(CACHE/(bag+'.npz')) as zz:z={k:zz[k] for k in zz.files}
        whole=build(z)
        for sec in (.15,1.,3.,10.,100.,400.):
            cut=int(z['cmd_bt'][0]+sec*1e9)
            prefix=build(subset(z,cut));n=len(prefix['q'])
            assert np.array_equal(prefix['q'],whole['q'][:n])
            err=float(np.max(abs(prefix['X']-whole['X'][:n])))
            assert err==0.,(bag,sec,err);maxdiff=max(maxdiff,err);checks+=1
        for n in ('master_vel','rover_vel','master_fix','rover_fix'):
            z[n+'_data']=np.full_like(z[n+'_data'],1e20)
        changed=build(z)
        assert np.array_equal(changed['X'],whole['X'])
    results.append({'prefixes':checks,'maximum_feature_difference':maxdiff})

def oracle_controls():
    t=np.arange(0.,10.,.05);truth=np.full(len(t),8.)
    assert metrics(t,truth,truth)['rmse']==0.
    assert metrics(t,truth,np.zeros(len(t)))['rmse']>1.
    assert metrics(t,truth,truth*3.6)['rmse']>10.
    try: metrics(t,truth,np.full(len(t),np.nan))
    except ValueError: pass
    else:raise AssertionError('Oracle accepted nonfinite predictions')

def absence():
    z=np.load(CACHE/'30618_0e41eac3.npz');d={k:z[k] for k in z.files}
    for n in ('front','rear'):
        for s in ('ht','bt','data'):d[n+'_'+s]=d[n+'_'+s][:0]
    a=build(d);assert np.all(np.isfinite(a['X'])) and np.all(a['X'][:,[5,6]]==10.)
    tree=load_arrays(ROOT/'config/drive_tree_v1.npz')
    c=json.loads((ROOT/'config/drive_parametric_v1.json').read_text());coef=np.asarray(c['coef'])
    p,flags,acc=observer_v3(a['t'],a['X'],tree,coef,True)
    assert np.all(np.isfinite(p)) and np.all(p>=0.) and np.all(flags==3)
    results.append({'all_wheels_absent':'finite output, NOT evidence of correct motion'})

checked('CDR known values, alignment and malformed headers',cdr)
checked('Causal prefix equivalence and no GNSS in X',causal)
checked('Oracle positive and negative controls',oracle_controls)
checked('Completely absent wheel channels',absence)
(ROOT/'results/reference_tests.json').write_text(json.dumps(results,indent=2))
