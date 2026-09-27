from pathlib import Path
import numpy as np,sys,json,subprocess
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from observer_v3 import observer_v3
from tree_runtime import load_arrays
features=Path(sys.argv[1]);coef=np.array(json.loads((ROOT/'config/drive_parametric_v1.json').read_text())['coef']);tree=load_arrays(ROOT/'config/drive_tree_v1.npz')
rows=[]
for bag in ('30618_2050d396','30618_33bec73f','30639_d927f360','30618_616ec56b','30618_0686195f'):
 z=np.load(features/(bag+'.npz'));t=z['t'];X=z['X'];r=np.empty(len(t),dtype=[('t','<f8'),('X','<f4',(26,))]);r['t']=t;r['X']=X
 inp=ROOT/'build/input.bin';out=ROOT/'build/output.bin';r.tofile(inp)
 runs=[]
 for k in range(3):
  p=subprocess.run([str(ROOT/'build/core_cli'),str(inp),str(out)],capture_output=True,text=True,check=True);runs.append(json.loads(p.stderr))
 actual=np.fromfile(out,dtype='<f8');expected=observer_v3(t,X,tree,coef,True)[0];error=float(np.max(abs(actual-expected)))
 assert error<1e-9,(bag,error)
 row={'bag':bag,'max_abs_difference_python_cpp':error,'runs':runs,'status':'PASS'};rows.append(row);print(row,flush=True)
(ROOT/'results/cpp_tests_and_timing.json').write_text(json.dumps(rows,indent=2))
