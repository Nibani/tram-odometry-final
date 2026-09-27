"""Turn fixed physical regression probes into failing CI checks.

The C++ fixtures print errors against known trajectories. This wrapper checks
the declared thresholds; a successful process exit alone is not acceptance.
"""
import argparse, json, math, subprocess
from pathlib import Path

def validate(output):
    rows=[]
    for line in output.splitlines():
        if not line.startswith('SUMMARY '): continue
        tokens=line.split();row={'case':tokens[1]}
        for token in tokens[2:]:
            if '=' in token:
                key,value=token.split('=',1);row[key]=float(value)
        rows.append(row)
    recovery=[r for r in rows if r['case']=='recovery_detail']
    assert len(recovery)==3, 'missing recovery families'
    for r in recovery:
        assert all(math.isfinite(v) for k,v in r.items() if k!='case')
        assert r['maxstep']<=5.25+1e-6, r
        if r['common_y']==0:
            assert 0<=r['first_below1_seconds']<=2 and r['error_at2s']<1, r
        else:
            assert r['error_at2s']<=abs(r['common_y'])+1, r
    healthy=[r for r in rows if r['case']=='true_async' and r['enabled']==1]
    assert len(healthy)==3, 'missing asynchronous controls'
    for r in healthy:
        assert r['first']>=0 and r['after_entry_max']<.5 and r['finalerr']<.01, r
    corrupt=[r for r in rows if r['case']=='async' and r['enabled']==1]
    assert len(corrupt)==4, 'missing conflicting receiver controls'
    for r in corrupt: assert r['first']==-1, r
    rejoin=[r for r in rows if r['case']=='phase_return' and r['enabled']==1]
    assert len(rejoin)==1 and rejoin[0]['finalerr']<.01, rejoin
    assert 'PASS future_mate_prefix' in output and 'PASS reset_conflict_then_branch' in output
    return rows

def main():
    p=argparse.ArgumentParser();p.add_argument('--exe',type=Path,required=True);p.add_argument('--out',type=Path);a=p.parse_args()
    run=subprocess.run([str(a.exe.resolve())],capture_output=True,text=True,check=True,timeout=60)
    print(run.stdout,end='');rows=validate(run.stdout)
    # Negative oracle control: an otherwise identical report with a dangerous
    # step must fail. This checks the gate independently of the estimator.
    import re
    poisoned=re.sub(r'maxstep=[^\s]+','maxstep=99',run.stdout)
    try: validate(poisoned)
    except AssertionError: pass
    else: raise AssertionError('oracle accepted an unsafe step')
    result={'status':'PASS','oracle_negative_control':'PASS','rows':rows}
    if a.out:a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2))
    print('PASS_OFFMAP_NUMERIC_GATES')

if __name__=='__main__':main()
