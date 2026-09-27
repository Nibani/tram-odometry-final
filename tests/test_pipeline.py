"""Native integration properties. Requires a built pipeline_cli and NumPy."""
import argparse,io,json,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from evaluate_pipeline import events,sums
from map_geometry import ecef,ORIGIN,Route,base_from_antennas

def invoke(exe,text,routes):
    r=subprocess.run([str(exe),*map(str,routes)],input=text,capture_output=True,text=True,check=True)
    return np.loadtxt(io.StringIO(r.stdout),ndmin=2)

def lla_from_enu(p):
    la,lo=np.deg2rad(ORIGIN[:2]);rot=np.array([[-np.sin(lo),np.cos(lo),0],[-np.sin(la)*np.cos(lo),-np.sin(la)*np.sin(lo),np.cos(la)],[np.cos(la)*np.cos(lo),np.cos(la)*np.sin(lo),np.sin(la)]])
    xyz=np.asarray(p)@rot+ecef(ORIGIN);x,y,z=xyz;lon=np.arctan2(y,x);rho=np.hypot(x,y);lat=np.arctan2(z,rho*(1-6.6943799901413165e-3))
    for _ in range(10):
        n=6378137./np.sqrt(1-6.6943799901413165e-3*np.sin(lat)**2);h=rho/np.cos(lat)-n;lat=np.arctan2(z,rho*(1-6.6943799901413165e-3*n/(n+h)))
    return np.array([np.rad2deg(lat),np.rad2deg(lon),h])

def test(exe,work,outdir):
    outdir.mkdir(parents=True,exist_ok=True);passed=[]
    route=Route([[0,0,0],[300,0,0]]);path=outdir/'line.csv';route.save(path)
    t0=10000000000;rows=[]
    # Moving start, fixes delayed by 0.20s and available only for the first3s.
    # Correct initialization requires propagating the fix from measurement time.
    for i in range(121):
        h=t0+i*50000000;b=h+1000000
        rows.extend([(b,f'F {h} {b} 18\n'),(b+1,f'R {h} {b+1} 18\n')])
        rows.append((b+2,f'C {h} {b+2} 0\n'))
        if i%2==0 and i<=56:
            for key,x in [('M',-9.873),('G',2.563)]:
                p=lla_from_enu([50+5*i*.05+x,0,3]);arrival=h+200000000
                rows.append((arrival,f'{key} {h} {arrival} '+ ' '.join(format(v,'.17g') for v in p)+' 0\n'))
    rows.sort(key=lambda p:p[0]);text=''.join(r for _,r in rows);a=invoke(exe,text,[path]);q=a[:,0].astype(np.int64);m=q>=t0+3000000000
    assert np.max(abs(a[m,5]-(50+5*(q[m]-t0)*1e-9)))<.005,'moving startup compensation'
    assert np.all((a[m,8].astype(int)&8)==0),'absolute initialization'
    passed.extend(['moving_start_delayed_fixes','absolute_init_from_prefix'])
    # Poison all later GNSS. Both header and arrival are after the sealed window.
    poison=[]
    for i in range(61,121):
        h=t0+i*50000000;poison.extend([(h-1,f'M {h} {h-1} 0 0 0 0\n'),(h-1,f'G {h} {h-1} 10 10 1000 0\n')])
    attacked=sorted(rows+poison,key=lambda p:p[0]);b=invoke(exe,''.join(r for _,r in attacked),[path]);assert np.array_equal(a,b),'post-init GNSS leakage';passed.append('post_window_GNSS_isolation')
    # Future-dated wheel values arrive early but must not influence earlier outputs.
    future=f'F {t0+9000000000} {t0+4000000000} 10000\n'
    attack=rows+[(t0+4000000000,future)];attack.sort(key=lambda p:p[0]);b=invoke(exe,''.join(r for _,r in attack),[path]);
    # This timestamp poisons future ordering only; existing past data still drives
    # earlier outputs. We check before the future message would starve freshness.
    assert np.array_equal(a,b),'future header affected estimator';passed.append('future_header_not_used_early')
    future_cmd=f'C {t0+9000000000} {t0+4000000000} 15\n'
    attack=sorted(rows+[(t0+4000000000,future_cmd)],key=lambda p:p[0]);b=invoke(exe,''.join(r for _,r in attack),[path]);assert np.array_equal(a,b),'future controller corrupted state';passed.append('future_controller_rejected')
    early_future=f'M {t0+9000000000} {t0-1000000} 55.805 37.425 170 0\n'
    b=invoke(exe,early_future+text,[path]);assert np.array_equal(a,b),'prestart future GNSS poisoned queue';passed.append('future_GNSS_rejected_before_start')
    prefix=[r for ar,r in rows if ar<=t0+2000000000];b=invoke(exe,''.join(prefix),[path]);assert np.array_equal(b,a[:len(b)]),'prefix dependence';passed.append('prefix_invariance')
    # Explicit sample-stamp reset; user must reset all state before another bag.
    b=invoke(exe,text+f'X {t0} {t0}\n'+text,[path]);assert np.array_equal(b[:len(a)],b[len(a):]),'reset mismatch';passed.append('clock_reset')
    assert sums(np.zeros(10))['rmse']<.1 and sums(np.full(10,5.))['rmse']>.1,'oracle controls';passed.append('good_and_wrong_speed_controls')
    # Analytic circular motion tests body-frame twist, including front-pivot lateral velocity.
    angle=np.linspace(0,2*np.pi,6001);circle=Route(np.column_stack([30*np.cos(angle),30*np.sin(angle),np.zeros(len(angle))]));cpath=outdir/'circle.csv';circle.save(cpath);circle_rows=[]
    half=np.arcsin(7.55/60.)
    for i in range(121):
        h=t0+i*50000000;theta=1.+5*i*.05/30.;forward=np.array([np.cos(theta+np.pi/2-half),np.sin(theta+np.pi/2-half),0.]);base=np.array([30*np.cos(theta),30*np.sin(theta),0.])
        circle_rows+=[f'F {h} {h} 18\n',f'R {h} {h} 18\n']
        if i<=56 and i%2==0:
            for key,x in [('M',-9.873),('G',2.563)]:
                p=lla_from_enu(base+x*forward+[0,0,3]);circle_rows.append(f'{key} {h} {h} '+' '.join(format(v,'.17g') for v in p)+' 0\n')
        circle_rows.append(f'C {h} {h} 0\n')
    c=invoke(exe,''.join(circle_rows),[cpath,'--loop']);expected=np.array([5*np.cos(half),5*np.sin(half),0.,0.,0.,5/30.]);assert np.max(abs(c[61:,14:20]-expected))<.003,'body twist on circle';passed.append('analytic_circle_body_twist')
    for horizontal in [0.,1.]:
        degraded_rows=[]
        for i in range(121):
            h=t0+i*50000000;degraded_rows += [f'F {h} {h} 18\n',f'R {h} {h} 18\n']
            if i<=56 and i%2==0:
                for key,x,zerr in [('M',-9.873,-1.5),('G',2.563+horizontal,1.5)]:
                    p=lla_from_enu([50+5*i*.05+x,0.,3.+zerr]);degraded_rows.append(f'{key} {h} {h} '+' '.join(format(v,'.17g') for v in p)+' 0\n')
            degraded_rows.append(f'C {h} {h} 0\n')
        payload=''.join(degraded_rows);c=invoke(exe,payload,[path]);use=c[:,0]>=t0+3_000_000_000
        assert np.all((c[use,8].astype(int)&8)==0),'degraded initialization missing'
        assert np.max(abs(c[use,5]-(50+5*(c[use,0]-t0)*1e-9+horizontal/2)))<.005,'XY fit biased by false GNSS pitch'
        if horizontal:
            assert np.all((c[use,8].astype(int)&32)!=0),'degraded quality flag missing'
            strict=invoke(exe,payload,[path,'--strict-init']);assert np.all((strict[use,8].astype(int)&8)!=0),'strict negative control unexpectedly passed'
            late=f'M {t0+7_000_000_000} {t0+7_000_000_000} 0 0 0 0\n'
            assert np.array_equal(c,invoke(exe,payload+late,[path])),'degraded late GNSS leakage'
        passed.append('joint_antenna_XY_fit_Z_error'+str(horizontal))
    report={'status':'PASS','tests':passed,'n_outputs':len(a),'moving_start_max_error_m':float(np.max(abs(a[m,5]-(50+5*(q[m]-t0)*1e-9))))}
    (outdir/'properties.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--exe',type=Path,required=True);p.add_argument('--work',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();test(a.exe,a.work,a.out)
