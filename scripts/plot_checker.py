"""Static, shareable plots of the actual matched offline pairs; no smoothing of errors."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--decoded',type=Path,required=True);p.add_argument('--predictions',type=Path,required=True);p.add_argument('--pairs',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
a.out.parent.mkdir(parents=True,exist_ok=True)
pred=np.loadtxt(a.predictions);base=np.loadtxt(a.baseline)
assert np.array_equal(pred[:,0],base[:,0]),'Unequal output populations'
with np.load(a.decoded/'reference.npz') as d:ref=d['reference_data'].copy()
with np.load(a.pairs) as d:
 ri=d['position_reference_index'];pi=d['position_prediction_index'];vr=d['reference_index'];vp=d['prediction_index'];q=d['prediction_header_ns']
t=(q-q[0])*1e-9/60
xyz=ref[ri,:3];origin=xyz[0,:2]
err=np.linalg.norm(pred[pi,5:8]-xyz,axis=1);eb=np.linalg.norm(base[pi,5:8]-xyz,axis=1)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
ax=axs[0,0];ax.plot(xyz[:,0]-origin[0],xyz[:,1]-origin[1],color='#333333',lw=2,label='Эталон чекера');ax.plot(pred[pi,5]-origin[0],pred[pi,6]-origin[1],color='#067b80',lw=1,label='Решение');ax.set_aspect('equal');ax.set(xlabel='X относительно старта, м',ylabel='Y относительно старта, м',title='Траектория');ax.legend()
ax=axs[0,1];ax.plot(t[pi],eb,color='#bc7152',lw=.8,label='Без поздних GNSS');ax.plot(t[pi],err,color='#067b80',lw=.8,label='С поздними GNSS');ax.set(xlabel='Время по header, мин',ylabel='Ошибка XYZ, м',title='Все сопоставленные позиции, без отсечения ошибок');ax.legend()
ax=axs[1,0];ax.plot(t[vp],ref[vr,43],color='#333333',lw=1,label='Эталон');ax.plot(t[vp],pred[vp,1],color='#067b80',lw=.7,label='B6');ax.set(xlabel='Время по header, мин',ylabel='Скорость, м/с',title='Скорость');ax.legend()
ax=axs[1,1];ax.plot(t[vp],pred[vp,1]-ref[vr,43],color='#067b80',lw=.7);ax.axhline(0,color='#444444',lw=.5);ax.set(xlabel='Время по header, мин',ylabel='Ошибка скорости, м/с',title='Без геометрического пересчёта скорости')
for ax in axs.flat:ax.grid(alpha=.2)
fig.suptitle('Открытый пример организаторов · 30618_88aea4d9\nОфлайн ATS, предполагаемая задержка 0 мс; начальная непубликация позиции учитывается отдельно',fontsize=13)
fig.savefig(a.out.with_suffix('.png'),dpi=170);fig.savefig(a.out.with_suffix('.svg'));plt.close(fig)
print(str(a.out.with_suffix('.png')))
