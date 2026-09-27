from pathlib import Path
import numpy as np,sys,json,subprocess
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from observer_v3 import observer_v3
from tree_runtime import load_arrays
from prepare_features import build
cache=Path(sys.argv[1]);coef=np.array(json.loads((ROOT/'config/drive_parametric_v1.json').read_text())['coef']);tree=load_arrays(ROOT/'config/drive_tree_v1.npz')
rows=[]
for bag in ('30618_2050d396','30618_33bec73f','30639_d927f360','30618_616ec56b','30618_0686195f'):
 z=np.load(cache/(bag+'.npz'));d=build(z);events=[];arrival=[]
 for j,n in enumerate(['front','rear','cmd']):
  r=np.empty(len(z[n+'_ht']),dtype=[('kind','u1'),('stamp','<i8'),('value','<f8')]);r['kind']=j;r['stamp']=z[n+'_ht'];r['value']=z[n+'_data'][:,0];events.append(r);arrival.append(z[n+'_bt'])
 ix=np.argsort(np.concatenate(arrival),kind='stable');e=np.concatenate(events)[ix];e.tofile(ROOT/'build/events.bin')
 subprocess.run([str(ROOT/'build/stream_cli'),str(ROOT/'build/events.bin'),str(ROOT/'build/stream_output.bin')],check=True,capture_output=True)
 p=np.fromfile(ROOT/'build/stream_output.bin',dtype='<f8');ref=observer_v3(d['t'],d['X'],tree,coef,True)[0]
 assert p.shape==ref.shape
 err=float(np.max(abs(p-ref)));assert err<2e-6,(bag,err)
 row={'bag':bag,'n':len(p),'maximum_velocity_error_m_s':err,'status':'PASS'};rows.append(row);print(row,flush=True)
(ROOT/'results/cpp_stream_tests.json').write_text(json.dumps(rows,indent=2))
