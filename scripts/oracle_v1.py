"""Fixed author-written benchmark. Not the missing official judging script."""
from __future__ import annotations
import numpy as np

def metrics(t: np.ndarray, y: np.ndarray, p: np.ndarray):
    if y.shape!=p.shape or y.shape!=t.shape:raise ValueError('shape mismatch')
    m=np.isfinite(y);v=np.isfinite(p)
    if not np.all(v):raise ValueError('nonfinite prediction')
    if not m.any():return {'n':0,'coverage':0.}
    e=p[m]-y[m];dt=np.r_[0.,np.diff(t)]
    # This is only a proxy over GNSS-covered, continuous intervals, NOT 3D position drift.
    covered=m & np.r_[False,m[:-1]] & (dt>0)&(dt<.2)
    ei=p-y;integ=np.sum(.5*(ei[covered]+np.r_[0,ei[:-1]][covered])*dt[covered])
    moving=m&(y>1);stopped=m&(y<.1)
    return {'n':int(m.sum()),'coverage':float(m.mean()),'sse':float(e@e),
      'sae':float(abs(e).sum()),'se':float(e.sum()),'rmse':float(np.sqrt(np.mean(e**2))),
      'mae':float(abs(e).mean()),'bias':float(e.mean()),'p95_abs':float(np.quantile(abs(e),.95)),
      'p99_abs':float(np.quantile(abs(e),.99)),
      'covered_integral_error_m':float(integ),'covered_seconds':float(dt[covered].sum()),
      'moving_rmse':float(np.sqrt(np.mean((p[moving]-y[moving])**2))) if moving.any() else None,
      'stopped_rmse':float(np.sqrt(np.mean((p[stopped]-y[stopped])**2))) if stopped.any() else None}

def aggregate(rows):
    a=[r for r in rows if r.get('n',0)>0];n=sum(r['n'] for r in a)
    if not n:return {'n':0}
    return {'n':n,'bags':len(a),'rmse':float(np.sqrt(sum(r['sse'] for r in a)/n)),
      'mae':sum(r['sae'] for r in a)/n,'bias':sum(r['se'] for r in a)/n,
      'macro_rmse':float(np.mean([r['rmse'] for r in a])),
      'mean_abs_covered_integral_error_m':float(np.mean([abs(r['covered_integral_error_m']) for r in a])),
      'median_coverage':float(np.median([r['coverage'] for r in a]))}
