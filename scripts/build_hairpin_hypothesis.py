"""Rebuild the fixed TRAIN hairpin hypothesis from decoded cache and features.

Dependencies: Python >=3.10, NumPy, sibling map_geometry.py, and a separately
compiled route_project_cli using the release's reserve_odometry/route.hpp.
No training-bag list, saved projections, research directory or fitted coefficient
file is needed. Selection and the six-coefficient fit are reproduced from data.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import time

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMBA_NUM_THREADS'):
    os.environ[key] = '1'
sys.dont_write_bytecode = True
import numpy as np
from map_geometry import Route, paired_reference, REFERENCE_POLICY

# Geometry constants belong to this frozen baseline map, not to bag identity.
# Canonical LF hash also accepts a Git checkout with CRLF line endings.
BASELINE_LF_SHA256 = '6ee15c70018210cf0e1474fddffd33bb6f460a93bb65bdccae16e7247d984ba0'
EXPECTED_CSV_SHA256 = 'ebadd38dab45bf746c2cbe7914aa6a428a021ea5be961fcaa3a67c4c6dc881bb'
OFFICIAL_RANGES = ((551.3593682118322, 5260.286678058046),
                   (5781.905803932912, 10490.467453783569))
FIT_LOOP_LENGTH = 11036.254598166197
TERMINAL_ORIGIN = 10490.467453783569
START, END = 430., 650.


def sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def basis(distance):
    u = np.clip((np.asarray(distance) - START) / (END - START), 0, 1)
    envelope = 16*u*u*(1-u)*(1-u)
    return envelope[:, None] * np.polynomial.legendre.legvander(2*u-1, 5)


def fit_coefficients(design, response, bag):
    weight = np.zeros(len(response))
    for name in np.unique(bag):
        weight[bag == name] = 1/np.sum(bag == name)
    weight /= weight.sum()
    coefficients = np.zeros(design.shape[1])
    for _ in range(30):
        residual = response - design @ coefficients
        w = weight * np.minimum(1, .5/np.maximum(abs(residual), 1e-9))
        coefficients = np.linalg.solve(
            design.T @ (w[:, None]*design) + .001*np.eye(design.shape[1]),
            design.T @ (w*response))
    return coefficients


def sample_ids(reference, t, features):
    # This order is significant: the original fixed 5 s quality sampling was
    # done BEFORE rejecting stationary points. Do not move the speed gate up.
    valid = reference['quality'] & (t >= 3.)
    ids, last = [], -1e9
    for i in np.flatnonzero(valid):
        if t[i] - last >= 5.:
            ids.append(i)
            last = t[i]
    ids = np.asarray(ids, dtype=int)
    return ids[features[ids, 17] > .5], len(ids)


def project(executable, baseline_map, points, heading):
    payload = ''.join(' '.join(format(float(v), '.17g') for v in [*p, *h])+'\n'
                      for p, h in zip(points, heading))
    process = subprocess.run([str(executable), str(baseline_map)], input=payload,
                             text=True, capture_output=True, check=True)
    result = np.loadtxt(io.StringIO(process.stdout), ndmin=2)
    if result.shape != (len(points), 7) or not np.isfinite(result).all():
        raise ValueError('Projector returned invalid shape or nonfinite values')
    return result


def collect_samples(cache, feature_dir, split, loop, projector, baseline_map):
    rows, batches, all_points, all_headings = [], [], [], []
    for bag, metadata in split.items():
        if metadata['part'] != 'train':
            continue
        row = {'bag': bag, 'part': 'train', 'date': metadata['date']}
        with np.load(feature_dir/(bag+'.npz'), allow_pickle=False) as features:
            q, t, x = features['q'], features['t'], features['X']
        with np.load(cache/(bag+'.npz'), allow_pickle=False) as raw:
            reference = paired_reference(raw, q)
        if reference is None:
            row['status'] = 'no_dual_gnss'
            rows.append(row)
            continue
        ids, sampled = sample_ids(reference, t, x)
        row.update(sampled_quality_count=sampled, moving_sample_count=len(ids))
        if len(ids) < 10:
            row['status'] = 'fewer_than_10_moving_samples'
            rows.append(row)
            continue
        points, heading = reference['base'][ids], reference['heading'][ids]
        row['status'] = 'projected'
        rows.append(row)
        batches.append((row, points))
        all_points.append(points)
        all_headings.append(heading)
    if not batches:
        raise ValueError('No TRAIN projection samples; check cache/features/split')
    # One process avoids repeated map loading; row arithmetic/order is unchanged.
    projections = project(projector, baseline_map, np.concatenate(all_points),
                          np.concatenate(all_headings))
    samples, labels, eligible, training_bags = [], [], [], []
    offset = 0
    for row, points in batches:
        projection = projections[offset:offset+len(points)]
        offset += len(points)
        s = projection[:, 0]
        tangent = loop.at(np.mod(s+1, loop.s[-1]))[:, :2] - loop.at(np.mod(s-1, loop.s[-1]))[:, :2]
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        error = points-projection[:, 3:6]
        cross = (error[:, :2]*normal).sum(1)
        central = np.zeros(len(points), bool)
        for low, high in OFFICIAL_RANGES:
            central |= (s > low+150) & (s < high-150)
        good = (projection[:, 6] > .98) & (np.linalg.norm(error[:, :2], axis=1) < 12)
        cc = central & good
        central_rmse = float(np.sqrt(np.mean((error[cc, :2]**2).sum(1)))) if cc.any() else None
        row.update(central_count=int(cc.sum()), central_xy_rmse_m=central_rmse)
        if cc.sum() < 20 or central_rmse >= 2:
            row['status'] = 'rejected_central_support'
            continue
        eligible.append(row['bag'])
        distance = np.mod(s-TERMINAL_ORIGIN, FIT_LOOP_LENGTH)
        local = good & (distance >= START) & (distance <= END)
        row.update(status='central_eligible', local_fit_count=int(local.sum()))
        if not local.any():
            continue
        samples.extend(np.column_stack([distance[local], cross[local]]))
        labels.extend([row['bag']]*int(local.sum()))
        training_bags.append(row['bag'])
    if not samples:
        raise ValueError('No eligible local TRAIN samples')
    return np.array(samples), np.array(labels), rows, eligible, training_bags


def save_map(loop, coefficients, path):
    distance = np.mod(loop.s-TERMINAL_ORIGIN, loop.s[-1])
    mask = (distance >= START) & (distance <= END)
    points = loop.xyz.copy()
    tangent = loop.at(np.mod(loop.s+1, loop.s[-1]))[:, :2] - loop.at(np.mod(loop.s-1, loop.s[-1]))[:, :2]
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    points[mask, :2] += normal[mask]*(basis(distance) @ coefficients)[mask, None]
    candidate = Route(points)
    # Reproduce the frozen Windows-generated CSV byte-for-byte on any OS.
    buffer = io.StringIO()
    np.savetxt(buffer, np.column_stack([candidate.s, candidate.xyz]), delimiter=',',
               header='s,x,y,z', comments='', fmt='%.9f', newline='\r\n')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buffer.getvalue().encode('ascii'))
    return {'sha256': sha256(path.read_bytes()),
            'canonical_lf_sha256': sha256(path.read_bytes().replace(b'\r\n', b'\n')),
            'length_m': float(candidate.s[-1]),
            'endpoint_gap_m': float(np.linalg.norm(points[0]-points[-1])),
            'changed_vertices': int(mask.sum())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', required=True, type=Path)
    parser.add_argument('--features', required=True, type=Path)
    parser.add_argument('--split', required=True, type=Path)
    parser.add_argument('--baseline-map', required=True, type=Path)
    parser.add_argument('--projector', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--expect-sha256', default=EXPECTED_CSV_SHA256,
                        help='Expected raw CSV hash; a mismatch exits nonzero after writing diagnostic report')
    args = parser.parse_args()
    begin = time.perf_counter()
    source = args.baseline_map.read_bytes()
    if sha256(source.replace(b'\r\n', b'\n')) != BASELINE_LF_SHA256:
        raise ValueError('Baseline geometry differs from the frozen map (line endings normalized)')
    split = json.loads(args.split.read_text(encoding='utf-8'))
    loop = Route.load(args.baseline_map)
    if abs(loop.s[-1]-FIT_LOOP_LENGTH) > 1e-6:
        raise ValueError('Baseline length differs from frozen geometry')
    samples, labels, rows, eligible, training_bags = collect_samples(
        args.cache, args.features, split, loop, args.projector.resolve(), args.baseline_map.resolve())
    coefficients = fit_coefficients(basis(samples[:, 0]), samples[:, 1], labels)
    generated = save_map(loop, coefficients, args.out)
    matched = generated['sha256'] == args.expect_sha256
    report = {'status': 'EXACT_REBUILD_PASS' if matched else 'HASH_MISMATCH',
              'training_part': 'train', 'train_bags_considered': len(rows),
              'projected_bags': sum(r.get('moving_sample_count', 0) >= 10 for r in rows),
              'central_eligible_bags': eligible, 'training_bags': training_bags,
              'local_sample_count': len(samples), 'coefficients': coefficients.tolist(),
              'method': 'quality&t>=3 sampled every5s, then X17>0.5; native closed Route.project; XY<12, cos>0.98; central>=20 and XYRMSE<2; terminal d430..650; C1 envelope times Legendre degree0..5; equal bag Huber0.5 ridge0.001,30IRLS iterations',
              'official_ranges_m': OFFICIAL_RANGES, 'terminal_origin_m': TERMINAL_ORIGIN,
              'baseline_map_raw_sha256': sha256(source), 'baseline_map_canonical_lf_sha256': BASELINE_LF_SHA256,
              'split_sha256': sha256(args.split.read_bytes()),
              'projector_sha256': sha256(args.projector.read_bytes()),
              'builder_sha256': sha256(Path(__file__).read_bytes()),
              'reference_policy': REFERENCE_POLICY,
              'reference_geometry_sha256': sha256((Path(__file__).parent/'map_geometry.py').read_bytes()),
              'source_training_sha256': {b: split[b]['sha256'] for b in training_bags},
              'candidate_map': generated, 'expected_raw_csv_sha256': args.expect_sha256,
              'format': 'ASCII CSV,9 decimal places,explicit CRLF',
              'seed': 'N/A deterministic', 'threads': 1,
              'per_bag': rows, 'elapsed_seconds': time.perf_counter()-begin}
    report_path = args.report or args.out.with_suffix('.json')
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('status', 'train_bags_considered', 'projected_bags',
                                           'local_sample_count', 'coefficients', 'candidate_map', 'elapsed_seconds')}, indent=2))
    if not matched:
        raise SystemExit('Rebuilt CSV differs from expected hash; inspect the report, do not silently adopt it')


if __name__ == '__main__':
    main()
