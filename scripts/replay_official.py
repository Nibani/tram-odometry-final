"""Replay the fixed competition candidate; compare with the provided checker offline."""
from pathlib import Path
import argparse,hashlib,json,math,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--decoded',type=Path,required=True)
p.add_argument('--exe',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--disable-correction',action='store_true')
p.add_argument('--geometric-velocity',action='store_true')
p.add_argument('--phase-only',action='store_true')
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
cal=json.loads((ROOT/'config/map_calibration.json').read_text())
r=cal['enu_to_map_R']
cmd=[str(a.exe.resolve()),str(ROOT/'config/route_loop.csv'),str(ROOT/'config/route_hairpin_hypothesis.csv'),
     '--loop','--speed-scale','1.0003350854241781','--map-transform',str(math.atan2(r[1][0],r[0][0])),
     *map(str,cal['enu_to_map_translation']),str(cal['map_z_minus_enu_up']),'--offmap-departure','10']
if not a.disable_correction:cmd.append('--sparse-gnss')
if not a.disable_correction and not a.phase_only:cmd.append('--xy-residual')
if a.geometric_velocity:cmd.append('--body-velocity')
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
manifest={'command':cmd,'exe_sha256':sha(a.exe),'script_sha256':sha(__file__),
          'events_sha256':sha(a.decoded/'events.txt'),'calibration_sha256':sha(ROOT/'config/map_calibration.json'),
          'comparison':'Fixed candidate, no training; official exposed development example; offline assumed latency'}
(a.out/'manifest.json').write_text(json.dumps(manifest,indent=2))
with (a.decoded/'events.txt').open() as src,(a.out/'predictions.txt').open('w') as dest:
    subprocess.run(cmd,stdin=src,stdout=dest,check=True)
for latency in [0,3]:
    subprocess.run([sys.executable,str(ROOT/'scripts/check_official.py'),'score','--decoded',str(a.decoded),
        '--predictions',str(a.out/'predictions.txt'),'--out',str(a.out/f'score_{latency}ms.json'),
        '--solution-frame','map','--absolute-position-only','--latency-ms',str(latency)],check=True)
