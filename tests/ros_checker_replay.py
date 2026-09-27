"""Private-fixture actual DDS replay and unmodified official ATS metrics.

Run from a sourced ROS 2 Humble candidate workspace:
  python3 tests/ros_checker_replay.py --fixture tests/fixtures/checker_sample.npz \
      --official-metrics tests/vendor/official_metrics.py --out results/checker_dds

The estimator receives only the five allowed input topics and /clock. Reference
Odometry is published solely for the official checker and recorder; subscription
graph assertions verify that it is not an estimator input. Publish order is the
original SQLite (record timestamp, row id), with exact int64 header timestamps.
Clock ticks use original record time and never pass an unpublished record.

The official MetricsNode and real message_filters synchronizers run unchanged.
A subclass observes their callbacks after the original accumulation, saving the
actual scored pairs. No nearest-neighbor substitute or assumed output latency is
used. Accuracy metrics are reported, not used as pass/fail tuning thresholds.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import inspect
import json
import math
import os
import platform
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np


KINDS = ('front', 'rear', 'cmd', 'master_fix', 'rover_fix', 'reference')
TOPICS = {
    'front': '/vehicle/front_bogie_velocity',
    'rear': '/vehicle/rear_bogie_velocity',
    'cmd': '/vehicle/driver_position_cmd',
    'master_fix': '/sensing/gnss/master/fix',
    'rover_fix': '/sensing/gnss/rover/fix',
    'reference': '/localization/kinematic_state',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    return value


def dump(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False), encoding='utf-8')


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(int(value), 10**9)


def load_fixture(path, provenance_path):
    with np.load(path, allow_pickle=False) as source:
        arrays = {name: source[name].copy() for name in source.files}
    events, ids, counts = [], [], {}
    for kind in KINDS:
        ht, bt, mid, data = (arrays[kind+'_'+suffix] for suffix in ('ht', 'bt', 'id', 'data'))
        expected_width = 13 if kind == 'reference' else 15 if kind.endswith('_fix') else 1
        assert ht.dtype == bt.dtype == mid.dtype == np.dtype('int64'), kind+' timestamp dtype'
        assert ht.ndim == bt.ndim == mid.ndim == 1 and len(ht) == len(bt) == len(mid)
        assert data.shape == (len(ht), expected_width), (kind, data.shape)
        assert (ht > 0).all() and (bt > 0).all() and (mid > 0).all()
        counts[kind] = len(ht)
        ids.extend(mid.tolist())
        events.extend((int(bt[i]), int(mid[i]), kind, i) for i in range(len(ht)))
    assert len(ids) == len(set(ids)), 'Duplicate original SQLite row ids'
    events.sort(key=lambda event: (event[0], event[1]))
    assert events and all(b[:2] > a[:2] for a, b in zip(events, events[1:]))
    provenance = json.loads(provenance_path.read_text()) if provenance_path and provenance_path.exists() else None
    fixture_sha = sha(path)
    if provenance is not None and 'fixture_sha256' in provenance:
        assert fixture_sha == provenance['fixture_sha256'], 'Fixture/provenance hash mismatch'
    metadata = dict(fixture_sha256=fixture_sha, provenance=provenance, counts=counts,
                    allowed_input_events=sum(counts[k] for k in KINDS if k != 'reference'),
                    reference_events=counts['reference'], merged_events=len(events),
                    first_record_ns=events[0][0], last_record_ns=events[-1][0],
                    duration_s=(events[-1][0]-events[0][0])*1e-9,
                    order='Original SQLite ORDER BY timestamp,id; header order unchanged',
                    reconstructed_frames='reference map/base_link; sensor frames descriptive only',
                    omitted_fields='reference covariance and original frame strings are absent from fixture; official metrics ignores them')
    return arrays, events, metadata


def make_message(kind, header, row, classes):
    """Reconstruct all retained original numeric fields; never derive an input from a reference."""
    if kind in ('front', 'rear'):
        message = classes['velocity']()
        message.header.frame_id = 'base_link'
        message.velocity = float(row[0])
    elif kind == 'cmd':
        message = classes['command']()
        message.header.frame_id = 'base_link'
        message.position = int(row[0])
        assert -128 <= message.position <= 127 and float(message.position) == float(row[0])
    elif kind.endswith('_fix'):
        message = classes['fix']()
        message.header.frame_id = kind.removesuffix('_fix')
        message.latitude, message.longitude, message.altitude = map(float, row[:3])
        message.position_covariance = [float(v) for v in row[3:12]]
        message.status.status, message.status.service = int(row[12]), int(row[13])
        message.position_covariance_type = int(row[14])
    else:
        assert kind == 'reference'
        message = classes['odometry']()
        message.header.frame_id, message.child_frame_id = 'map', 'base_link'
        p, q, v, w = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear, message.twist.twist.angular
        p.x, p.y, p.z = map(float, row[:3])
        q.x, q.y, q.z, q.w = map(float, row[3:7])
        v.x, v.y, v.z = map(float, row[7:10])
        w.x, w.y, w.z = map(float, row[10:13])
    set_stamp(message.header.stamp, header)
    return message


def affinity():
    if hasattr(os, 'sched_getaffinity'):
        os.sched_setaffinity(0, set(sorted(os.sched_getaffinity(0))[:2]))


def stop(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=3.)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3.)


def quantiles(values):
    return None if not len(values) else dict(zip(('min', 'median', 'p95', 'p99', 'max'),
                                                np.quantile(values, [0., .5, .95, .99, 1.]).tolist()))


def performance_checks(report):
    """Pre-existing project budgets, with coverage required before a PASS."""
    latency = report.get('command_to_result_latency_wall_s') or {}
    resources = report.get('node_resources') or {}
    coverage = report.get('coverage') or {}
    result = []
    def add(name, passed, evidence):
        result.append(dict(name=name, passed=bool(passed), evidence=evidence))
    for kind in ('velocity', 'position'):
        stats = latency.get(kind) or {}
        n, total = latency.get(kind+'_n', 0), coverage.get(kind+'_outputs', 0)
        hz = report.get(kind+'_output_hz_in_header_time')
        add(kind+'_latency_measurement_coverage', total > 0 and n >= .995*total, dict(measured=n, outputs=total))
        add(kind+'_latency_p99_below_100ms', 0 <= stats.get('p99', math.inf) <= .1, stats.get('p99'))
        add(kind+'_latency_max_below_250ms', 0 <= stats.get('max', math.inf) <= .25, stats.get('max'))
        add(kind+'_publication_at_least_10Hz', hz is not None and math.isfinite(hz) and hz >= 10., hz)
    rss, cpu, affinity_count = resources.get('peak_sampled_rss_kib'), resources.get('mean_cpu_cores'), resources.get('affinity_cores')
    add('node_resource_measurement_present', resources.get('samples', 0) > 1, resources)
    add('sampled_node_memory_below_512MiB', rss is not None and 0 < rss <= 512*1024, rss)
    add('node_mean_cpu_below_two_cores', cpu is not None and math.isfinite(cpu) and 0 <= cpu <= 2., cpu)
    add('node_affinity_at_most_two_cores', affinity_count is not None and 0 < affinity_count <= 2, affinity_count)
    return result


def run(args):
    args.out.mkdir(parents=True, exist_ok=True)
    arrays, events, fixture = load_fixture(args.fixture, args.provenance)
    report = dict(status='NOT_COMPLETED', fixture=fixture, assertions=[],
                  interpretation='Actual DDS/official ATS measurement; no rank or unseen-data claim',
                  publication_policy='Private CI fixture; reference never enters estimator inputs')
    if args.inspect_fixture_only:
        dump(args.out/'fixture_inspection.json', fixture)
        print(json.dumps(safe(fixture), indent=2))
        return 0
    import message_filters
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64MultiArray
    from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

    module_name = 'checker_official_metrics'
    spec = importlib.util.spec_from_file_location(module_name, args.official_metrics)
    official = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = official
    spec.loader.exec_module(official)

    class RecordingMetrics(official.MetricsNode):
        def __init__(self):
            self.velocity_pairs, self.position_pairs = [], []
            super().__init__()

        def _on_velocity_pair(self, reference, solution):
            before = self.velocity_metric.count
            super()._on_velocity_pair(reference, solution)
            if self.velocity_metric.count > before:
                self.velocity_pairs.append((stamp_ns(reference.header.stamp), stamp_ns(solution.header.stamp),
                    official.get_numeric_field(reference, self.reference_velocity_field),
                    official.get_numeric_field(solution, self.result_velocity_field)))

        def _on_position_pair(self, reference, solution):
            super()._on_position_pair(reference, solution)
            a, b = reference.pose.pose.position, solution.pose.pose.position
            self.position_pairs.append((stamp_ns(reference.header.stamp), stamp_ns(solution.header.stamp),
                                        (a.x, a.y, a.z), (b.x, b.y, b.z)))

    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'report_period_sec:=0.0'])
    node, metrics = Node('checker_dds_replay'), RecordingMetrics()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(metrics)
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    classes = dict(velocity=VelocitySensor, command=DriverControllerCommand, fix=NavSatFix, odometry=Odometry)
    message_types = {kind: Odometry if kind == 'reference' else NavSatFix if kind.endswith('_fix') else
                     DriverControllerCommand if kind == 'cmd' else VelocitySensor for kind in KINDS}
    publishers = {kind: node.create_publisher(message_types[kind], TOPICS[kind], qos) for kind in KINDS}
    clock = node.create_publisher(Clock, '/clock', 10)
    outputs, positions, received_references, diagnostics = [], [], [], []
    command_wall_ns, velocity_latency, position_latency = {}, [], []
    resource_samples = []
    last_resource_wall = 0.

    def velocity_received(message):
        h = stamp_ns(message.header.stamp)
        if h in command_wall_ns:
            velocity_latency.append((time.monotonic_ns()-command_wall_ns[h])*1e-9)
        outputs.append((stamp_ns(message.header.stamp), node.get_clock().now().nanoseconds,
                        message.velocity, message.header.frame_id))

    def position_received(message):
        h = stamp_ns(message.header.stamp)
        if h in command_wall_ns:
            position_latency.append((time.monotonic_ns()-command_wall_ns[h])*1e-9)
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
        positions.append((stamp_ns(message.header.stamp), node.get_clock().now().nanoseconds,
                          (p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z),
                          message.header.frame_id, message.child_frame_id))

    node.create_subscription(VelocitySensor, '/result/velocity', velocity_received, qos)
    node.create_subscription(Odometry, '/result/position', position_received, qos)
    node.create_subscription(Odometry, TOPICS['reference'],
                             lambda m: received_references.append(stamp_ns(m.header.stamp)), qos)
    node.create_subscription(Float64MultiArray, '/result/diagnostics',
                             lambda m: diagnostics.append((node.get_clock().now().nanoseconds, list(m.data))), qos)
    process, log = None, None
    counts = {kind: 0 for kind in KINDS}
    clock_count, last_clock, lag_samples = 0, 0, []
    start_wall = time.monotonic()
    playback_wall = end_playback_wall = None

    def check(name, passed, evidence):
        report['assertions'].append(dict(name=name, passed=bool(passed), evidence=evidence))

    def spin_until(target):
        while time.monotonic() < target:
            executor.spin_once(timeout_sec=min(.005, max(0., target-time.monotonic())))
            if time.monotonic()-start_wall > args.wall_timeout:
                raise TimeoutError('Replay wall-time limit exceeded')

    def graph_snapshot():
        return {topic: [{'name': item.node_name, 'namespace': item.node_namespace}
                        for item in node.get_subscriptions_info_by_topic(topic)]
                for topic in [TOPICS['front'], TOPICS['reference']]}

    def sample_node_resources():
        # Only inspect descendants of our own launch process. Linux /proc gives
        # node RSS and CPU independently of this Python publisher/checker.
        nonlocal last_resource_wall
        now = time.monotonic()
        if process is None or now-last_resource_wall < 1.:
            return
        last_resource_wall = now
        todo, seen = [process.pid], set()
        while todo:
            pid = todo.pop()
            if pid in seen:
                continue
            seen.add(pid)
            base = Path('/proc')/str(pid)
            try:
                todo.extend(map(int, (base/'task'/str(pid)/'children').read_text().split()))
                command_line = (base/'cmdline').read_bytes().replace(b'\0', b' ')
                if b'/lib/reserve_odometry/reserve_odometry_node' not in command_line:
                    continue
                fields = (base/'stat').read_text().rsplit(')', 1)[1].split()
                cpu_s = (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
                rss = next(int(line.split()[1]) for line in (base/'status').read_text().splitlines() if line.startswith('VmRSS:'))
                resource_samples.append((now, pid, rss, cpu_s, len(os.sched_getaffinity(pid))))
            except (OSError, ValueError, StopIteration):
                # A disappearing child is recorded by the existing process-life check.
                continue

    try:
        share = Path(get_package_share_directory('reserve_odometry'))
        report['installed_sources'] = {str(path.relative_to(share)): sha(path)
            for path in [share/'launch'/args.launch_file, share/'config/map_calibration.json',
                         share/'config/route_loop.csv', share/'config/route_hairpin_hypothesis.csv'] if path.exists()}
        binary = Path(get_package_prefix('reserve_odometry'))/'lib/reserve_odometry/reserve_odometry_node'
        report['installed_binary_sha256'] = sha(binary)
        report['ci_commit'] = os.environ.get('GITHUB_SHA')
        report['official_metrics_sha256'] = sha(args.official_metrics)
        report['message_filters_ats_sha256'] = hashlib.sha256(inspect.getsource(
            message_filters.ApproximateTimeSynchronizer).encode()).hexdigest()
        report['python'] = platform.python_version()
        cmd = ['ros2', 'launch', 'reserve_odometry', args.launch_file, 'use_sim_time:=true']+args.launch_arg
        report['estimator_command'] = cmd
        report['sync'] = {'engine': 'unmodified official MetricsNode and installed message_filters ATS',
            'tolerance_s': metrics.get_parameter('sync_tolerance_sec').value,
            'queue_size': metrics.get_parameter('sync_queue_size').value,
            'reference_field': metrics.reference_velocity_field, 'result_field': metrics.result_velocity_field,
            'arrival_policy': 'Actual independent DDS subscribers; no assumed output publication delay'}
        log = (args.out/'estimator_node.log').open('w')
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, preexec_fn=affinity)
        deadline = time.monotonic()+args.discovery_timeout
        ready = False
        while time.monotonic() < deadline:
            spin_until(time.monotonic()+.02)
            ready = all(publishers[k].get_subscription_count() >= 1 for k in KINDS if k != 'reference') and \
                    publishers['reference'].get_subscription_count() >= 3 and \
                    node.count_publishers('/result/velocity') >= 1 and node.count_publishers('/result/position') >= 1 and \
                    clock.get_subscription_count() >= 3
            if ready:
                break
            if process.poll() is not None:
                raise RuntimeError('Estimator launch exited during discovery')
        graph = graph_snapshot()
        report['subscription_graph'] = graph
        estimator_names = {(item['name'], item['namespace']) for item in graph[TOPICS['front']]}
        reference_names = {(item['name'], item['namespace']) for item in graph[TOPICS['reference']]}
        check('DDS_endpoints_ready', ready, {kind: pub.get_subscription_count() for kind, pub in publishers.items()})
        check('reference_is_not_an_estimator_subscription', bool(estimator_names) and not(estimator_names & reference_names), graph)
        if not ready or estimator_names & reference_names:
            raise RuntimeError('DDS discovery or reference/input isolation failed')
        first, last = fixture['first_record_ns'], fixture['last_record_ns']
        next_tick, tick_step, index = first, max(1, round(1e9/args.clock_hz)), 0
        playback_wall = time.monotonic()+.1
        while index < len(events) or next_tick <= last:
            event_time = events[index][0] if index < len(events) else last+1
            tick_time = next_tick if next_tick <= last else last+1
            now = min(event_time, tick_time)
            target_wall = playback_wall+(now-first)*1e-9/args.rate
            spin_until(target_wall)
            lateness = max(0., time.monotonic()-target_wall)
            if now == tick_time:
                lag_samples.append(lateness)
            if now > last_clock:
                msg = Clock()
                set_stamp(msg.clock, now)
                clock.publish(msg)
                clock_count += 1
                last_clock = now
            if now == tick_time:
                next_tick += tick_step
            while index < len(events) and events[index][0] == now:
                record, row_id, kind, source_index = events[index]
                assert record == now and now == last_clock
                message = make_message(kind, arrays[kind+'_ht'][source_index], arrays[kind+'_data'][source_index], classes)
                if kind == 'cmd':
                    command_wall_ns[int(arrays[kind+'_ht'][source_index])] = time.monotonic_ns()
                publishers[kind].publish(message)
                counts[kind] += 1
                index += 1
            # Drain multiple ready callbacks, keeping separate checker queues live
            # during dense bursts without changing any original event timestamp.
            for _ in range(4):
                executor.spin_once(timeout_sec=0.)
            sample_node_resources()
            if process.poll() is not None:
                raise RuntimeError('Estimator launch exited during replay')
            if time.monotonic()-start_wall > args.wall_timeout:
                raise TimeoutError('Replay wall-time limit exceeded')
        end_playback_wall = time.monotonic()
        spin_until(time.monotonic()+1.)
        metrics.report()
        check('all_original_fixture_events_published', counts == fixture['counts'], counts)
        check('reference_transport_coverage', len(received_references) >= .995*fixture['reference_events'],
              dict(received=len(received_references), expected=fixture['reference_events']))
        check('nontrivial_velocity_output_and_real_ATS_pairs', len(outputs) > 1000 and metrics.velocity_metric.count > 1000,
              dict(outputs=len(outputs), pairs=metrics.velocity_metric.count))
        check('nontrivial_position_output_and_real_ATS_pairs', len(positions) > 1000 and metrics.position_metrics['distance'].count > 1000,
              dict(outputs=len(positions), pairs=metrics.position_metrics['distance'].count))
        check('finite_result_velocities', bool(outputs) and all(math.isfinite(row[2]) for row in outputs), len(outputs))
        check('absolute_map_positions_only', bool(positions) and all(row[3] == 'map' and row[4] == 'base_link' for row in positions), len(positions))
        check('finite_result_pose_and_body_velocity', bool(positions) and all(math.isfinite(v) for row in positions for v in row[2]), len(positions))
        check('monotonic_result_stamps', all(b[0] > a[0] for a, b in zip(outputs, outputs[1:])) and
              all(b[0] > a[0] for a, b in zip(positions, positions[1:])), [len(outputs), len(positions)])
        final_graph = graph_snapshot()
        final_inputs = {(x['name'], x['namespace']) for x in final_graph[TOPICS['front']]}
        final_refs = {(x['name'], x['namespace']) for x in final_graph[TOPICS['reference']]}
        check('reference_isolation_preserved_after_replay', bool(final_inputs) and not(final_inputs & final_refs), final_graph)
        report['status'] = 'PASS' if all(x['passed'] for x in report['assertions']) else 'FAIL'
    except Exception:
        report['status'] = 'ERROR'
        report['exception'] = traceback.format_exc()
    finally:
        stop(process)
        if log:
            log.close()
        def metric(accumulator):
            return dict(n=accumulator.count, rmse=accumulator.rmse, max=accumulator.maximum_error)
        report['official_metrics'] = {'velocity': metric(metrics.velocity_metric),
                                      'position': {k: metric(v) for k, v in metrics.position_metrics.items()}}
        report['coverage'] = dict(input_events_published=counts, reference_received=len(received_references),
            velocity_outputs=len(outputs), position_outputs=len(positions),
            velocity_pairs=metrics.velocity_metric.count, position_pairs=metrics.position_metrics['distance'].count,
            velocity_pair_fraction_of_output=metrics.velocity_metric.count/max(1, len(outputs)),
            position_pair_fraction_of_output=metrics.position_metrics['distance'].count/max(1, len(positions)))
        report['first_position'] = None if not positions else dict(header_ns=positions[0][0], received_clock_ns=positions[0][1],
            relative_to_first_record_s=(positions[0][1]-fixture['first_record_ns'])*1e-9,
            relative_to_first_command_header_s=(positions[0][0]-int(arrays['cmd_ht'][0]))*1e-9,
            frame=positions[0][3], xyz=positions[0][2][:3])
        report['replay'] = dict(requested_rate=args.rate, clock_hz_minimum=args.clock_hz, clock_publications=clock_count,
            simulated_duration_s=fixture['duration_s'], wall_elapsed_s=time.monotonic()-start_wall,
            playback_wall_s=None if playback_wall is None or end_playback_wall is None else end_playback_wall-playback_wall,
            schedule_lateness_wall_s=quantiles(lag_samples))
        if playback_wall is not None and end_playback_wall is not None:
            report['replay']['achieved_rate'] = fixture['duration_s']/max(1e-9, end_playback_wall-playback_wall)
        report['replay']['ordering_scope'] = 'Publication order is exact; receiver callback order and output delays are measured DDS behavior.'
        report['velocity_output_header_gap_s'] = quantiles([(b[0]-a[0])*1e-9 for a, b in zip(outputs, outputs[1:])])
        report['command_to_result_latency_wall_s'] = {
            'definition': 'Before command publisher.publish to result subscriber callback, matched by exact header; includes DDS and checker executor scheduling at requested replay rate',
            'velocity': quantiles(velocity_latency), 'position': quantiles(position_latency),
            'velocity_n': len(velocity_latency), 'position_n': len(position_latency)}
        report['node_resources'] = None
        if resource_samples:
            first_r, last_r = resource_samples[0], resource_samples[-1]
            interval = last_r[0]-first_r[0]
            report['node_resources'] = dict(sampling_wall_s=1., samples=len(resource_samples),
                peak_sampled_rss_kib=max(row[2] for row in resource_samples),
                affinity_cores=max(row[4] for row in resource_samples),
                mean_cpu_cores=(last_r[3]-first_r[3])/interval if interval > 0 and last_r[1] == first_r[1] else None,
                cpu_observation_wall_s=interval,
                caveat='One-second sampled node RSS; launch/checker memory excluded; sampled peak is not an absolute allocation peak')
        dump(args.out/'checker_dds_resources.json', resource_samples)
        report['position_output_header_gap_s'] = quantiles([(b[0]-a[0])*1e-9 for a, b in zip(positions, positions[1:])])
        report['velocity_pair_abs_header_delta_s'] = quantiles([abs(row[0]-row[1])*1e-9 for row in metrics.velocity_pairs])
        report['position_pair_abs_header_delta_s'] = quantiles([abs(row[0]-row[1])*1e-9 for row in metrics.position_pairs])
        for name, records in [('velocity', outputs), ('position', positions)]:
            duration = (records[-1][0]-records[0][0])*1e-9 if len(records) > 1 else 0.
            report[name+'_output_hz_in_header_time'] = (len(records)-1)/duration if duration > 0 else None
        report['assertions'].extend(performance_checks(report))
        if report['status'] != 'ERROR':
            report['status'] = 'PASS' if all(x['passed'] for x in report['assertions']) else 'FAIL'
        if args.native_score:
            native = json.loads(args.native_score.read_text())
            report['native_assumption_comparison'] = dict(source_sha256=sha(args.native_score),
                native_metrics_all=native.get('metrics', {}).get('all'), native_sync=native.get('sync'),
                native_coverage=native.get('coverage'),
                caveat='No equality expected: native assumed output arrival; this run measures actual DDS scheduling and ATS pairs. Check candidate settings before comparing.')
        np.savez_compressed(args.out/'checker_dds_pairs.npz',
            velocity_ref_stamp=np.array([r[0] for r in metrics.velocity_pairs], dtype=np.int64),
            velocity_result_stamp=np.array([r[1] for r in metrics.velocity_pairs], dtype=np.int64),
            velocity_values=np.array([r[2:] for r in metrics.velocity_pairs], dtype=float).reshape(-1, 2),
            position_ref_stamp=np.array([r[0] for r in metrics.position_pairs], dtype=np.int64),
            position_result_stamp=np.array([r[1] for r in metrics.position_pairs], dtype=np.int64),
            position_reference=np.array([r[2] for r in metrics.position_pairs], dtype=float).reshape(-1, 3),
            position_result=np.array([r[3] for r in metrics.position_pairs], dtype=float).reshape(-1, 3))
        np.savez_compressed(args.out/'checker_dds_outputs.npz',
            velocity_stamp=np.array([r[0] for r in outputs], dtype=np.int64),
            velocity_received_clock=np.array([r[1] for r in outputs], dtype=np.int64),
            velocity=np.array([r[2] for r in outputs], dtype=float),
            position_stamp=np.array([r[0] for r in positions], dtype=np.int64),
            position_received_clock=np.array([r[1] for r in positions], dtype=np.int64),
            position=np.array([r[2] for r in positions], dtype=float).reshape(-1, 10),
            reference_received_stamp=np.array(received_references, dtype=np.int64))
        np.savez_compressed(args.out/'checker_dds_latency.npz', velocity_wall_s=np.array(velocity_latency),
                            position_wall_s=np.array(position_latency))
        dump(args.out/'checker_dds_diagnostics.json', diagnostics)
        dump(args.out/'checker_dds.json', report)
        executor.shutdown()
        metrics.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(safe(report), indent=2, allow_nan=False))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--official-metrics', type=Path,
                        default=Path(__file__).resolve().parent/'vendor/official_metrics.py')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=4.)
    parser.add_argument('--clock-hz', type=float, default=100.)
    parser.add_argument('--wall-timeout', type=float, default=350.)
    parser.add_argument('--discovery-timeout', type=float, default=8.)
    parser.add_argument('--domain', type=int, default=48)
    parser.add_argument('--launch-file', default='odometry.launch.py')
    parser.add_argument('--launch-arg', action='append', default=[])
    parser.add_argument('--native-score', type=Path)
    parser.add_argument('--inspect-fixture-only', action='store_true')
    options = parser.parse_args()
    if options.provenance is None:
        options.provenance = options.fixture.with_name('provenance.json')
    if options.rate <= 0 or options.clock_hz <= 0:
        parser.error('rate and clock-hz must be positive')
    try:
        return_code = run(options)
    except Exception:
        options.out.mkdir(parents=True, exist_ok=True)
        failure = dict(status='ERROR_BEFORE_REPLAY_REPORT', exception=traceback.format_exc())
        dump(options.out/'checker_dds_startup_error.json', failure)
        print(json.dumps(failure, indent=2), file=sys.stderr)
        return_code = 1
    raise SystemExit(return_code)
