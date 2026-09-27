"""Parse the three supplied CDR layouts; no ROS installation required.
Times are stored separately: recording/arrival time versus sensor header time.
"""
from __future__ import annotations
import argparse, hashlib, json, sqlite3, struct, time
from pathlib import Path
import numpy as np

ALIASES = {'/vehicle/front_bogie_velocity':'front',
 '/vehicle/rear_bogie_velocity':'rear','/vehicle/driver_position_cmd':'cmd',
 '/sensing/gnss/master/vel':'master_vel','/sensing/gnss/rover/vel':'rover_vel',
 '/sensing/gnss/master/fix':'master_fix','/sensing/gnss/rover/fix':'rover_fix'}

def align(p: int, n: int) -> int:
    return 4 + ((p-4+n-1)//n)*n

def decode(b: bytes, alias: str):
    if b[:4] != b'\x00\x01\x00\x00':
        raise ValueError('Expected little-endian CDR encapsulation')
    sec,ns,length=struct.unpack_from('<iII',b,4)
    if ns >= 1_000_000_000 or not 1<=length<=10000 or 16+length>len(b):
        raise ValueError('Invalid header')
    if b[15+length] != 0: raise ValueError('Non-terminated frame_id')
    p=16+length
    if alias in ('front','rear'):
        val=struct.unpack_from('<d',b,align(p,8))
    elif alias=='cmd': val=struct.unpack_from('<b',b,p)
    elif alias.endswith('_vel'): val=struct.unpack_from('<6d',b,align(p,8))
    else:
        status=struct.unpack_from('<b',b,p)[0];p=align(p+1,2)
        service=struct.unpack_from('<H',b,p)[0];p=align(p+2,8)
        vals=struct.unpack_from('<12d',b,p)
        cov_type=b[p+96]
        val=(*vals,status,service,cov_type)
    return sec*1_000_000_000+ns,val

def main():
    pa=argparse.ArgumentParser();pa.add_argument('--data-root',type=Path,required=True)
    pa.add_argument('--out',type=Path,required=True);a=pa.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    inventory=[]; seen={};begin=time.perf_counter()
    for db in sorted(a.data_root.glob('*/*.db3')):
        digest=hashlib.sha256()
        with db.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
        sha=digest.hexdigest()
        rec={'bag':db.parent.name,'sha256':sha,'bytes':db.stat().st_size}
        if sha in seen:
            rec['duplicate_of']=seen[sha];inventory.append(rec);continue
        seen[sha]=db.parent.name
        con=sqlite3.connect(db);topics={i:ALIASES[n] for i,n in con.execute('select id,name from topics')}
        data={n:[] for n in topics.values()};bt={n:[] for n in topics.values()};ht={n:[] for n in topics.values()}
        for tid,t,b in con.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
            n=topics[tid];h,v=decode(b,n);data[n].append(v);bt[n].append(t);ht[n].append(h)
        con.close();out={};audit={}
        for n in data:
            x=np.asarray(data[n],dtype=np.float64);b=np.asarray(bt[n],dtype=np.int64);h=np.asarray(ht[n],dtype=np.int64)
            out[n+'_data']=x;out[n+'_bt']=b;out[n+'_ht']=h
            if len(x)==0:
                audit[n]={'n':0};continue
            delta=(b-h)/1e9;dbt=np.diff(b)/1e9;dht=np.diff(h)/1e9
            audit[n]={'n':len(x),'record_start_ns':int(b[0]),'record_end_ns':int(b[-1]),
              'header_start_ns':int(h[0]),'header_end_ns':int(h[-1]),
              'record_duration_s':float((b[-1]-b[0])/1e9),'header_duration_s':float((h[-1]-h[0])/1e9),
              'bt_minus_ht_quantiles':np.quantile(delta,[0,.01,.5,.99,1]).tolist(),
              'header_dt_quantiles':np.quantile(dht,[0,.01,.5,.99,1]).tolist() if len(h)>1 else [],
              'record_dt_quantiles':np.quantile(dbt,[0,.01,.5,.99,1]).tolist() if len(b)>1 else [],
              'nonincreasing_header':int((dht<=0).sum()),
              'min':np.nanmin(x,axis=0).tolist(),'max':np.nanmax(x,axis=0).tolist()}
        np.savez_compressed(a.out/(db.parent.name+'.npz'),**out)
        rec['topics']=audit;inventory.append(rec)
        if len(seen)%10==0:print('decoded',len(seen),'time',round(time.perf_counter()-begin,1),flush=True)
    (a.out/'inventory.json').write_text(json.dumps(inventory,indent=2),encoding='utf8')
    print('DONE',len(inventory),'bags',len(seen),'unique','elapsed',time.perf_counter()-begin)

if __name__=='__main__': main()
