"""Independent arrival-ordered scorer. Reference labels never enter events.txt.

python -B checker_harness.py decode --bag BAG.db3 --out decoded
python -B checker_harness.py score --decoded decoded --predictions output.txt --out score.json
python -B checker_harness.py selftest
"""
from __future__ import annotations
import argparse
import collections
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / 'tests/vendor/official_metrics.py'
CALIBRATION = ROOT / 'config/map_calibration.json'
ATS_SOURCE = 'https://raw.githubusercontent.com/ros2/message_filters/humble/src/message_filters/__init__.py'
ALIASES = {
    '/vehicle/front_bogie_velocity': 'front',
    '/vehicle/rear_bogie_velocity': 'rear',
    '/vehicle/driver_position_cmd': 'cmd',
    '/sensing/gnss/master/fix': 'master_fix',
    '/sensing/gnss/rover/fix': 'rover_fix',
    '/sensing/gnss/master/vel': 'master_vel',
    '/sensing/gnss/rover/vel': 'rover_vel',
    '/localization/kinematic_state': 'reference',
}
KINDS = {'front': 'F', 'rear': 'R', 'cmd': 'C', 'master_fix': 'M', 'rover_fix': 'G'}
WIDTHS = {'front': 1, 'rear': 1, 'cmd': 1, 'master_fix': 15,
          'rover_fix': 15, 'master_vel': 6, 'rover_vel': 6, 'reference': 85}

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()

def save_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf8')

def align(p, n):
    return 4 + ((p - 4 + n - 1) // n) * n

def header(b):
    if b[:4] != b'\x00\x01\x00\x00':
        raise ValueError('Expected little-endian CDR encapsulation')
    sec, ns, n = struct.unpack_from('<iII', b, 4)
    if ns >= 1_000_000_000 or not 1 <= n <= 10000 or 16 + n > len(b) or b[15+n] != 0:
        raise ValueError('Invalid CDR header')
    return sec * 10**9 + ns, b[16:15+n].decode('utf8'), 16+n

def decode_payload(b, alias):
    h, frame, p = header(b)
    child = ''
    if alias in ('front', 'rear'):
        vals = struct.unpack_from('<d', b, align(p, 8))
    elif alias == 'cmd':
        vals = struct.unpack_from('<b', b, p)
    elif alias.endswith('_vel'):
        vals = struct.unpack_from('<6d', b, align(p, 8))
    elif alias.endswith('_fix'):
        status = struct.unpack_from('<b', b, p)[0]
        p = align(p+1, 2)
        service = struct.unpack_from('<H', b, p)[0]
        p = align(p+2, 8)
        vals = (*struct.unpack_from('<12d', b, p), status, service, b[p+96])
    elif alias == 'reference':
        p = align(p, 4)
        n = struct.unpack_from('<I', b, p)[0]
        p += 4
        if not 1 <= n <= 10000 or p+n > len(b) or b[p+n-1] != 0:
            raise ValueError('Invalid Odometry child_frame_id')
        child = b[p:p+n-1].decode('utf8')
        vals = struct.unpack_from('<85d', b, align(p+n, 8))
    else:
        raise ValueError(alias)
    return h, frame, child, vals

def decode_bag(args):
    bag, out = args.bag.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(bag.as_uri()+'?mode=ro', uri=True)
    topics = {i: (name, typ) for i, name, typ in con.execute('select id,name,type from topics')}
    unknown = [name for name, typ in topics.values() if name not in ALIASES]
    arrays = {a: collections.defaultdict(list) for a in ALIASES.values()}
    count, event_count = 0, 0
    with (out/'events.txt').open('w', encoding='ascii', newline='\n') as event_file:
        for mid, tid, bt, payload in con.execute('select id,topic_id,timestamp,data from messages order by timestamp,id'):
            count += 1
            name, typ = topics[tid]
            if name not in ALIASES:
                continue
            a = ALIASES[name]
            h, frame, child, vals = decode_payload(payload, a)
            for key, val in [('id', mid), ('bt', bt), ('ht', h), ('data', vals), ('frame', frame), ('child', child)]:
                arrays[a][key].append(val)
            if a in KINDS:
                if a.endswith('_fix'):
                    body = ' '.join(format(v, '.17g') for v in vals[:3]) + ' ' + str(int(vals[12]))
                elif a == 'cmd':
                    body = str(int(vals[0]))
                else:
                    body = format(vals[0], '.17g')
                event_file.write(f'{KINDS[a]} {h} {bt} {body}\n')
                event_count += 1
    con.close()
    inputs, refs = {}, {}
    counts = {}
    for a, fields in arrays.items():
        dest = refs if a == 'reference' else inputs
        for key in ('id', 'bt', 'ht', 'data', 'frame', 'child'):
            dtype = np.int64 if key in ('id', 'bt', 'ht') else (np.float64 if key == 'data' else str)
            v = np.asarray(fields[key], dtype=dtype)
            if key == 'data':
                v = v.reshape(-1, WIDTHS[a])
            dest[a+'_'+key] = v
        counts[a] = len(fields['ht'])
    if not counts['reference'] or not counts['cmd']:
        raise ValueError('Bag must contain reference and command messages')
    # Uncompressed NPZ is intentional: quick transparent small local artifacts.
    np.savez(out/'inputs.npz', **inputs)
    np.savez(out/'reference.npz', **refs)
    shutil.copyfile(CHECKER, out/'official_metrics.py')
    shutil.copyfile(args.calibration, out/'map_calibration.json')
    files = ['inputs.npz', 'reference.npz', 'events.txt', 'official_metrics.py', 'map_calibration.json']
    manifest = {'schema': 'checker-harness-v1', 'bag': str(bag), 'bag_sha256': sha(bag),
        'message_count': count, 'event_count': event_count, 'counts': counts,
        'unknown_topics_omitted': unknown, 'events_allowed_kinds': KINDS,
        'events_order': 'SQLite messages ORDER BY timestamp,id; original int64 header and record ns',
        'labels_policy': 'reference only in reference.npz; never in events.txt or inputs.npz',
        'ats_source': ATS_SOURCE, 'ats_verified_source_lines': '308-378, read 2026-09-27',
        'calibration_source': str(args.calibration.resolve()),
        'harness_source': str(Path(__file__).resolve()), 'harness_sha256': sha(__file__),
        'files_sha256': {f: sha(out/f) for f in files}}
    save_json(out/'source_manifest.json', manifest)
    print(json.dumps({'out': str(out), 'counts': counts, 'event_count': event_count}))

class ArrivalATS:
    """Equivalent two-stream Humble ATS queue/matching rules, integer nanoseconds."""
    def __init__(self, slop_ns=50_000_000, queue_size=100):
        self.slop = int(slop_ns)
        self.size = int(queue_size)
        if self.slop < 0 or self.size < 1:
            raise ValueError('Tolerance must be nonnegative and queue size positive')
        self.queues = [{}, {}]
        self.pairs = []
        self.overflow = [0, 0]
        self.overwritten = [0, 0]

    def add(self, side, stamp, index):
        q = self.queues[side]
        if stamp in q:
            self.overwritten[side] += 1
        q[stamp] = index
        while len(q) > self.size:
            del q[min(q)]
            self.overflow[side] += 1
        if stamp not in q:
            return
        other = self.queues[1-side]
        # Python min preserves insertion order on equal deltas, as upstream stable sort.
        candidates = [h for h in other if abs(h-stamp) < self.slop]
        if not candidates:
            return
        h = min(candidates, key=lambda t: abs(t-stamp))
        pair = (q[stamp], other[h]) if side == 0 else (other[h], q[stamp])
        self.pairs.append(pair)
        del q[stamp]
        del other[h]

def read_predictions(path):
    headers, values = [], []
    with Path(path).open(encoding='utf8') as f:
        for line_no, line in enumerate(f, 1):
            if not line.strip():
                continue
            cols = line.split()
            if len(cols) != 20:
                raise ValueError(f'{path}:{line_no}: expected 20 columns, got {len(cols)}')
            headers.append(int(cols[0]))  # Never round 1e18 ns through float64.
            values.append([float(v) for v in cols[1:]])
    return np.asarray(headers, dtype=np.int64), np.asarray(values, dtype=float).reshape(-1, 19)

def link_commands(headers, inputs):
    available = collections.defaultdict(collections.deque)
    for i, h in enumerate(inputs['cmd_ht']):
        available[int(h)].append(i)
    indexes = []
    for h in headers:
        if not available[int(h)]:
            raise ValueError(f'Prediction stamp {h} has no unused matching command; timer/custom query outputs need their own recorded arrival timeline')
        if len(available[int(h)]) > 1:
            raise ValueError(f'Command stamp {h} occurs more than once; publication arrival cannot be uniquely recovered from this output format')
        indexes.append(available[int(h)].popleft())
    indexes = np.asarray(indexes, dtype=np.int64)
    if len(indexes) > 1 and np.any(indexes[1:] <= indexes[:-1]):
        raise ValueError('Predictions do not preserve source command arrival order')
    return indexes

def metric(errors):
    errors = np.asarray(errors, dtype=float)
    good = np.isfinite(errors)
    e = errors[good]
    return {'n': int(good.sum()), 'nonfinite': int((~good).sum()),
            'rmse': float(np.sqrt(np.mean(e*e))) if len(e) else None,
            'max': float(np.max(np.abs(e))) if len(e) else 0.0,
            'bias': float(np.mean(e)) if len(e) else None}

def score(args):
    folder = args.decoded.resolve()
    inputs = np.load(folder/'inputs.npz', allow_pickle=False)
    refs = np.load(folder/'reference.npz', allow_pickle=False)
    ph, pv = read_predictions(args.predictions)
    ci = link_commands(ph, inputs)
    latency = int(round(args.latency_ms*1e6))
    if latency < 0:
        raise ValueError('Negative publication latency is noncausal')
    pb = inputs['cmd_bt'][ci] + latency
    pid = inputs['cmd_id'][ci]
    rh, rb, rid = refs['reference_ht'], refs['reference_bt'], refs['reference_id']
    rd = refs['reference_data']
    flags = pv[:, 7]
    if not np.isfinite(flags).all() or np.any(flags != np.floor(flags)):
        raise ValueError('Invalid pipeline flag column')
    flags = flags.astype(np.int64)
    initialized = (flags & 8) == 0
    initializing = (flags & 16) != 0
    publish_position = initialized if args.absolute_position_only else np.ones(len(ph), dtype=bool)
    # At zero latency result occurs immediately after its own command bag event.
    events = [(int(t), int(mid), 0, 0, i) for i, (t, mid) in enumerate(zip(rb, rid))]
    events += [(int(t), int(mid), 1, 1, i) for i, (t, mid) in enumerate(zip(pb, pid))]
    events.sort()
    sync = ArrivalATS(args.tolerance_ns, args.queue_size)
    position_sync = ArrivalATS(args.tolerance_ns, args.queue_size)
    for bt, mid, phase, side, i in events:
        sync.add(side, int(rh[i] if side == 0 else ph[i]), i)
        if side == 0 or publish_position[i]:
            position_sync.add(side, int(rh[i] if side == 0 else ph[i]), i)
    pairs = np.asarray(sync.pairs, dtype=np.int64).reshape(-1, 2)
    r, p = pairs[:, 0], pairs[:, 1]
    position_pairs = np.asarray(position_sync.pairs, dtype=np.int64).reshape(-1, 2)
    zr, zp = position_pairs[:, 0], position_pairs[:, 1]
    pos = pv[:, 4:7].copy()  # Full columns 5,6,7.
    calibration = folder/'map_calibration.json'
    if args.solution_frame == 'enu':
        cal = json.loads(calibration.read_text())
        pos[:, :2] = pos[:, :2] @ np.asarray(cal['enu_to_map_R']).T + np.asarray(cal['enu_to_map_translation'])
        pos[:, 2] += cal['map_z_minus_enu_up']
    def metrics_for(mask):
        ix = np.flatnonzero(mask[p])
        rr, pp = r[ix], p[ix]
        zi = np.flatnonzero(mask[zp])
        delta = pos[zp[zi]] - rd[zr[zi], :3]
        return {'pairs': len(ix), 'velocity_pairs': len(ix), 'position_pairs': len(zi),
            'velocity_scalar': metric(pv[pp, 0] - rd[rr, 43]),
            'velocity_body_vx': metric(pv[pp, 13] - rd[rr, 43]),
            'position': {**{axis: metric(delta[:, k]) for k, axis in enumerate(('x', 'y', 'z'))},
                         'distance': metric(np.linalg.norm(delta, axis=1))}}
    start = int(inputs['cmd_ht'][0])
    masks = {'all': np.ones(len(ph), dtype=bool), 'initialized': initialized,
             'uninitialized': ~initialized, 'initialization_window_open': initializing,
             'first_3_header_seconds': ph < start + 3_000_000_000,
             'after_3_header_seconds': ph >= start + 3_000_000_000}
    dt = (ph[p] - rh[r]).astype(float)*1e-9
    zdt = (ph[zp] - rh[zr]).astype(float)*1e-9
    def sync_stats(synchronizer, deltas):
        return {'queue_overflow': synchronizer.overflow,
                'duplicate_stamp_overwrites': synchronizer.overwritten,
                'remaining_queued': [len(q) for q in synchronizer.queues],
                'paired_header_dt_s_quantiles': np.quantile(deltas, [0,.5,.95,1]).tolist() if len(deltas) else []}
    published_indexes = np.flatnonzero(publish_position)
    first_pos = int(published_indexes[0]) if len(published_indexes) else None
    result = {'schema': 'checker-harness-score-v1', 'predictions': str(args.predictions.resolve()),
        'prediction_sha256': sha(args.predictions), 'decoded_manifest_sha256': sha(folder/'source_manifest.json'),
        'harness_sha256': sha(__file__), 'solution_frame': args.solution_frame,
        'calibration_sha256': sha(calibration),
        'sync': {'policy': 'ROS2 Humble Python ATS two-stream arrival order', 'strict_slop_ns': args.tolerance_ns,
                 'queue_size': args.queue_size, 'publication_latency_ms_assumed': args.latency_ms,
                 'arrival_assumption': 'Each published output emitted after its linked command callback; no measured ROS/DDS scheduling. Independent reference/result queues for position and velocity.',
                 'source_url': ATS_SOURCE, 'queue_overflow': sync.overflow,
                 'duplicate_stamp_overwrites': sync.overwritten, 'remaining_queued': [len(q) for q in sync.queues],
                 'paired_header_dt_s_quantiles': np.quantile(dt, [0, .5, .95, 1]).tolist() if len(dt) else [],
                 'velocity': sync_stats(sync, dt), 'position': sync_stats(position_sync, zdt)},
        'position_publication': {'absolute_position_only': args.absolute_position_only,
                'policy': 'suppress flag8 rows before position ATS; velocity always published' if args.absolute_position_only else 'publish every row including uninitialized relative position',
                'input_rows': len(ph), 'published': int(publish_position.sum()),
                'uninitialized_intentionally_not_published': int((~publish_position).sum()),
                'first_published_header_ns': int(ph[first_pos]) if first_pos is not None else None,
                'first_published_assumed_arrival_ns': int(pb[first_pos]) if first_pos is not None else None,
                'first_publish_delay_from_first_command_record_s': float((int(pb[first_pos])-int(inputs['cmd_bt'][0]))/1e9) if first_pos is not None else None,
                'first_publish_delay_from_first_command_header_s': float((int(ph[first_pos])-start)/1e9) if first_pos is not None else None},
        'coverage': {'reference_messages': len(rh), 'command_queries': len(inputs['cmd_ht']),
                     'output_messages': len(ph), 'matched_pairs': len(p),
                     'output_fraction_of_commands': len(ph)/len(inputs['cmd_ht']),
                     'matched_fraction_of_output': len(p)/len(ph) if len(ph) else 0.,
                     'matched_fraction_of_reference': len(r)/len(rh),
                     'unmatched_output': len(ph)-len(p), 'unmatched_reference': len(rh)-len(r),
                     'output_initialized': int(initialized.sum()), 'output_uninitialized': int((~initialized).sum()),
                     'velocity_output_messages': len(ph), 'velocity_matched_pairs': len(p),
                     'position_output_messages': int(publish_position.sum()), 'position_matched_pairs': len(zp),
                     'position_matched_fraction_of_published': len(zp)/int(publish_position.sum()) if publish_position.any() else 0.,
                     'position_published_fraction_of_commands': int(publish_position.sum())/len(inputs['cmd_ht']),
                     'position_matched_fraction_of_reference': len(zr)/len(rh),
                     'position_unmatched_published': int(publish_position.sum())-len(zp),
                     'position_unmatched_reference': len(rh)-len(zr)},
        'metrics': {name: metrics_for(mask) for name, mask in masks.items()},
        'no_silent_exclusions': 'Primary metrics.all includes every matched published output. Optional flag8 position suppression occurs before its independent ATS and is counted explicitly; velocity retains every row. Nonfinite counted per official accumulator.'}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    save_json(args.out, result)
    np.savez(args.out.with_suffix('.pairs.npz'), reference_index=r, prediction_index=p,
             prediction_header_ns=ph, prediction_arrival_ns=pb,
             initialized=initialized, initializing=initializing, position_published=publish_position,
             matched_header_delta_ns=ph[p]-rh[r], position_reference_index=zr,
             position_prediction_index=zp, position_matched_header_delta_ns=ph[zp]-rh[zr])
    print(json.dumps({'out': str(args.out.resolve()), 'coverage': result['coverage'], 'all': result['metrics']['all']}))

def selftest():
    s = ArrivalATS(); s.add(0, 0, 0); s.add(1, 50_000_000, 0)
    assert not s.pairs, 'exact slop must not pair'
    s.add(1, 49_999_999, 1); assert s.pairs == [(0, 1)]
    s.add(1, 1, 2); assert len(s.pairs) == 1, 'reference consumed once'
    s = ArrivalATS(); s.add(0, 110, 0); s.add(0, 90, 1); s.add(1, 100, 0)
    assert s.pairs == [(0, 0)], 'equal-delta tie must follow queue insertion'
    s = ArrivalATS(); s.add(0, 110, 0); s.add(0, 105, 1); s.add(1, 100, 0)
    assert s.pairs == [(1, 0)], 'nearest queued reference can have a future header'
    s = ArrivalATS(); s.add(0, 100, 0); s.add(0, 100, 1); s.add(1, 100, 0)
    assert s.pairs == [(1, 0)] and s.overwritten == [1, 0]
    s = ArrivalATS(queue_size=2); s.add(0, 100, 0); s.add(0, 200, 1); s.add(0, 50, 2)
    assert list(s.queues[0]) == [100, 200] and s.overflow == [1, 0]
    assert metric([3., -4., np.nan])['rmse'] == np.sqrt(12.5)
    assert metric([3., -4., np.nan])['nonfinite'] == 1
    print('PASS: strict boundary, one-use, insertion tie, nearest/future header, duplicate overwrite, queue eviction, accumulator')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    d = sub.add_parser('decode'); d.add_argument('--bag', type=Path, required=True)
    d.add_argument('--out', type=Path, required=True); d.add_argument('--calibration', type=Path, default=CALIBRATION)
    s = sub.add_parser('score'); s.add_argument('--decoded', type=Path, required=True)
    s.add_argument('--predictions', type=Path, required=True); s.add_argument('--out', type=Path, required=True)
    s.add_argument('--solution-frame', choices=['raw', 'enu', 'map'], default='enu')
    s.add_argument('--absolute-position-only', action='store_true')
    s.add_argument('--latency-ms', type=float, default=0.)
    s.add_argument('--tolerance-ns', type=int, default=50_000_000)
    s.add_argument('--queue-size', type=int, default=100)
    sub.add_parser('selftest')
    args = p.parse_args()
    if args.action == 'decode': decode_bag(args)
    elif args.action == 'score': score(args)
    else: selftest()

if __name__ == '__main__':
    main()
