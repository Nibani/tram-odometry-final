"""Replay the fixed candidate. Evaluation does not retrain or select a model."""
import argparse,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--bags',type=Path,required=True);p.add_argument('--work',type=Path,required=True);p.add_argument('--exe',type=Path);p.add_argument('--cxx',default='g++');p.add_argument('--parts',nargs='+',choices=['train','validation','holdout'],default=['validation','holdout']);p.add_argument('--speed-scale',type=float,default=1.0003350854241781);p.add_argument('--map-mode',choices=['hypotheses','loop','official'],default='hypotheses');a=p.parse_args()
    a.bags=a.bags.resolve();a.work=a.work.resolve();a.work.mkdir(parents=True,exist_ok=True)
    if not list(a.bags.glob('*/*.db3')):raise SystemExit('Expected --bags directory with <bag>/<bag>_0.db3')
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',NUMBA_NUM_THREADS='1')
    def run(*cmd):
        print('+',*map(str,cmd),flush=True);subprocess.run(list(map(str,cmd)),cwd=ROOT,env=env,check=True)
    run(sys.executable,ROOT/'scripts/audit_data.py','--data-root',a.bags,'--out',a.work/'cache')
    actual={r['bag']:r['sha256'] for r in json.loads((a.work/'cache/inventory.json').read_text())}
    for bag,row in json.loads((ROOT/'config/split_v1.json').read_text()).items():
        if actual.get(bag)!=row['sha256']:raise SystemExit('Dataset SHA256 mismatch: '+bag)
    run(sys.executable,ROOT/'scripts/prepare_features.py','--cache',a.work/'cache','--out',a.work/'features','--split',ROOT/'config/split_v1.json')
    exe=a.exe.resolve() if a.exe else a.work/('pipeline_cli.exe' if os.name=='nt' else 'pipeline_cli')
    if a.exe is None:run(a.cxx,'-O2','-std=c++17','-I'+str(ROOT/'src/reserve_odometry/include'),ROOT/'src/reserve_odometry/src/pipeline_cli.cpp','-o',exe)
    run(sys.executable,ROOT/'tests/test_pipeline.py','--exe',exe,'--work',a.work,'--out',a.work/'properties')
    run(sys.executable,ROOT/'tests/test_multimap.py','--exe',exe,'--out',a.work/'multimap_properties')
    for part in a.parts:run(sys.executable,ROOT/'scripts/evaluate_pipeline.py','--work',a.work,'--exe',exe,'--part',part,'--mode',a.map_mode,'--speed-scale',a.speed_scale,'--out',a.work/('evaluation_'+part))
    print('Replay completed. holdout is previously exposed audit data. ROS/DDS checks run separately.')
if __name__=='__main__':main()
