"""Deterministic actual-node DDS contract test; run in a sourced ROS 2 Humble tree.

python3 tests/ros_checker_compat.py --out results/ci_checker_compat

Four remapped ``ros2 run`` processes share an eight-second simulated-clock input
schedule. Normal wall runtime is about 11--12 seconds including discovery. This
uses synthetic geometry only; it never publishes or reads kinematic_state.
Later +/-4 m along-track GNSS innovations are deliberately synthetic measurement
errors that test update semantics, not a claim of real-world GNSS accuracy.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import tempfile
import time
import traceback
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor


SPEED = 5.0
MASTER, ROVER, HEIGHT, BOGIE = -9.873, 2.563, 3.0, 7.55
EPOCH_NS = 100_000_000_000
DT_NS = 50_000_000
CASES = ('active', 'disabled', 'no_gnss', 'circle')
LINE_YAW, LINE_PITCH = .4, .1
FORWARD = (math.cos(LINE_PITCH)*math.cos(LINE_YAW),
           math.cos(LINE_PITCH)*math.sin(LINE_YAW), math.sin(LINE_PITCH))
UP = (-math.sin(LINE_PITCH)*math.cos(LINE_YAW),
      -math.sin(LINE_PITCH)*math.sin(LINE_YAW), math.cos(LINE_PITCH))
RADIUS = 20.0


def stamp_ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


def set_stamp(stamp, ns):
    stamp.sec, stamp.nanosec = divmod(int(ns), 10**9)


def ecef(lla):
    lat, lon, height = math.radians(lla[0]), math.radians(lla[1]), lla[2]
    e2 = 6.6943799901413165e-3
    n = 6378137. / math.sqrt(1-e2*math.sin(lat)**2)
    return ((n+height)*math.cos(lat)*math.cos(lon),
            (n+height)*math.cos(lat)*math.sin(lon), (n*(1-e2)+height)*math.sin(lat))


def enu_to_lla(enu, origin):
    """Independent WGS84 inverse for small synthetic local ENU fixtures."""
    lat, lon = math.radians(origin[0]), math.radians(origin[1])
    rows = ((-math.sin(lon), math.cos(lon), 0.),
            (-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)),
            (math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)))
    center = ecef(origin)
    x, y, z = [center[j]+sum(enu[i]*rows[i][j] for i in range(3)) for j in range(3)]
    rho, e2 = math.hypot(x, y), 6.6943799901413165e-3
    latitude = math.atan2(z, rho*(1-e2))
    for _ in range(12):
        n = 6378137./math.sqrt(1-e2*math.sin(latitude)**2)
        height = rho/math.cos(latitude)-n
        latitude = math.atan2(z, rho*(1-e2*n/(n+height)))
    return (math.degrees(latitude), math.degrees(math.atan2(y, x)), height)


def line_point(s):
    return tuple(s*x for x in FORWARD)


def antenna_point(kind, elapsed, lever, shift=0.):
    if kind == 'circle':
        angle = (100.+SPEED*elapsed)/RADIUS
        yaw = angle+math.pi/2-math.asin(BOGIE/(2*RADIUS))
        return (RADIUS*math.cos(angle)+lever*math.cos(yaw),
                RADIUS*math.sin(angle)+lever*math.sin(yaw), HEIGHT)
    base = line_point(50.+SPEED*elapsed+shift)
    return tuple(base[k]+lever*FORWARD[k]+HEIGHT*UP[k] for k in range(3))


def map_point(enu, yaw, translation):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*enu[0]-s*enu[1]+translation[0],
            s*enu[0]+c*enu[1]+translation[1], enu[2]+translation[2])


def unmap_point(point, yaw, translation):
    x, y, z = [point[k]-translation[k] for k in range(3)]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*x+s*y, -s*x+c*y, z)


def quaternion(yaw, pitch):
    return (-math.sin(yaw/2)*math.sin(pitch/2),
            math.cos(yaw/2)*math.sin(pitch/2),
            math.sin(yaw/2)*math.cos(pitch/2), math.cos(yaw/2)*math.cos(pitch/2))


def quaternion_error(a, b):
    denominator = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    if denominator == 0.:
        return math.inf
    cosine = min(1., abs(sum(x*y for x, y in zip(a, b)))/denominator)
    return 2*math.acos(cosine)


def parameters(calibration, route, case):
    rotation = calibration['enu_to_map_R']
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    translation = calibration['enu_to_map_translation'] + [calibration['map_z_minus_enu_up']]
    origin = calibration['origin_lla']
    return {
        'use_sim_time': True, 'route_files': [str(route)], 'closed_route': case == 'circle',
        'initial_gnss_seconds': 2.0, 'wheel_divisor': 3.6, 'speed_scale': 1.0,
        'map_output': True, 'map_rotation_yaw': yaw, 'map_translation': translation,
        'body_velocity_output': True, 'publish_relative_position': False,
        'sparse_gnss_correction': case != 'disabled', 'gnss_correction_gain': .25,
        'gnss_max_step': 5., 'gnss_innovation_gate': 30., 'gnss_residual_gate': 8.,
        'gnss_max_age': 1., 'align_initial_position': False,
        'compensate_velocity_lever_arm': False, 'allow_degraded_initialization': False,
        'output_point_x': 0., 'output_point_z': 0., 'frame_id': 'map',
        'child_frame_id': 'base_link', 'origin_lat': origin[0],
        'origin_lon': origin[1], 'origin_alt': origin[2],
    }


def command(parameters_, case):
    args = ['ros2', 'run', 'reserve_odometry', 'reserve_odometry_node', '--ros-args',
            '-r', '__node:=checker_compat_'+case]
    for name, value in parameters_.items():
        args += ['-p', name+':='+json.dumps(value)]
    for source, target in [
        ('/vehicle/front_bogie_velocity', '/compat/input/front'),
        ('/vehicle/rear_bogie_velocity', '/compat/input/rear'),
        ('/vehicle/driver_position_cmd', '/compat/input/command'),
        ('/sensing/gnss/master/fix', '/compat/'+('line' if case in ('active', 'disabled') else case)+'/master'),
        ('/sensing/gnss/rover/fix', '/compat/'+('line' if case in ('active', 'disabled') else case)+'/rover'),
        ('/result/velocity', '/compat/'+case+'/velocity'),
        ('/result/position', '/compat/'+case+'/position'),
        ('/result/diagnostics', '/compat/'+case+'/diagnostics'),
        ('/result/slip_status', '/compat/'+case+'/slip_status'),
    ]:
        args += ['-r', source+':='+target]
    return args


def child_setup():
    if hasattr(os, 'sched_getaffinity'):
        os.sched_setaffinity(0, set(sorted(os.sched_getaffinity(0))[:2]))


def stop(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.)


def json_safe(value):
    """Keep failure reports strict JSON even when a missing sample gives infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def run(args):
    args.out.mkdir(parents=True, exist_ok=True)
    calibration = json.loads(args.calibration.read_text())
    rotation = calibration['enu_to_map_R']
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    translation = calibration['enu_to_map_translation'] + [calibration['map_z_minus_enu_up']]
    origin = calibration['origin_lla']
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = Node('checker_compat_fixture')
    samples = {case: {'velocity': [], 'position': [], 'diagnostics': []} for case in CASES}
    processes, logs, events, assertions = [], [], [], []
    report = {'status': 'NOT_COMPLETED', 'assertions': assertions, 'events': events,
              'calibration_sha256': hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
              'coverage': {}, 'commands': {}, 'synthetic_only': True,
              'kinematic_state_used': False, 'ros': 'Humble'}
    started = time.monotonic()

    def check(name, passed, evidence):
        assertions.append({'name': name, 'pass': bool(passed), 'evidence': evidence})

    def spin_until(deadline):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.003, max(0., deadline-time.monotonic())))

    def pose_record(case, message):
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
        samples[case]['position'].append({'stamp': stamp_ns(message.header.stamp),
            'xyz': [p.x, p.y, p.z], 'quaternion': [q.x, q.y, q.z, q.w],
            'body_velocity': [v.x, v.y, v.z], 'frame': message.header.frame_id,
            'child': message.child_frame_id})

    clock = node.create_publisher(Clock, '/clock', 10)
    wheels = [node.create_publisher(VelocitySensor, '/compat/input/'+name, 10) for name in ('front', 'rear')]
    controller = node.create_publisher(DriverControllerCommand, '/compat/input/command', 10)
    fixes = {(kind, channel): node.create_publisher(NavSatFix, '/compat/'+kind+'/'+channel, 10)
             for kind in ('line', 'circle') for channel in ('master', 'rover')}
    for case in CASES:
        node.create_subscription(VelocitySensor, '/compat/'+case+'/velocity',
            lambda m, c=case: samples[c]['velocity'].append(
                {'stamp': stamp_ns(m.header.stamp), 'value': m.velocity, 'frame': m.header.frame_id}), 200)
        node.create_subscription(Odometry, '/compat/'+case+'/position',
            lambda m, c=case: pose_record(c, m), 200)
        node.create_subscription(Float64MultiArray, '/compat/'+case+'/diagnostics',
            lambda m, c=case: samples[c]['diagnostics'].append(list(m.data)), 100)

    def publish_fix(kind, channel, index, shift=0., header_delta=0., status=0):
        ns = EPOCH_NS+index*DT_NS+int(header_delta*1e9)
        elapsed = (ns-EPOCH_NS)*1e-9
        msg = NavSatFix()
        set_stamp(msg.header.stamp, ns)
        msg.header.frame_id = channel
        msg.status.status = status
        msg.latitude, msg.longitude, msg.altitude = enu_to_lla(
            antenna_point(kind, elapsed, MASTER if channel == 'master' else ROVER, shift), origin)
        fixes[kind, channel].publish(msg)

    try:
        with tempfile.TemporaryDirectory(prefix='checker_compat_') as temp:
            temp = Path(temp)
            line = temp/'line.csv'
            end = line_point(500.)
            line.write_text('s,x,y,z\n0,0,0,0\n500,'+','.join(map(repr, end))+'\n')
            circle = temp/'circle.csv'
            count = 8000
            points = [(RADIUS*math.cos(2*math.pi*i/count), RADIUS*math.sin(2*math.pi*i/count), 0.)
                      for i in range(count)]
            points.append(points[0])
            arc, rows = 0., ['s,x,y,z']
            for i, point in enumerate(points):
                if i:
                    arc += math.dist(points[i-1], point)
                rows.append(','.join(map(repr, (arc, *point))))
            circle.write_text('\n'.join(rows)+'\n')
            for case in CASES:
                config = parameters(calibration, circle if case == 'circle' else line, case)
                cmd = command(config, case)
                report['commands'][case] = cmd
                log = (args.out/(case+'_node.log')).open('w')
                logs.append(log)
                processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True, preexec_fn=child_setup))
            # Wait for actual endpoint discovery, with a bounded CI allowance.
            discovery = time.monotonic()+args.discovery_timeout
            while time.monotonic() < discovery:
                spin_until(time.monotonic()+.02)
                if all(p.get_subscription_count() >= 4 for p in wheels+[controller]) and \
                   all(fixes[k, c].get_subscription_count() >= (2 if k == 'line' else 1)
                       for k, c in fixes) and \
                   clock.get_subscription_count() >= 4:
                    break
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A ros2 run process exited during discovery')
            ready = all(p.get_subscription_count() >= 4 for p in wheels+[controller]) and \
                    all(fixes[k, c].get_subscription_count() >= (2 if k == 'line' else 1)
                        for k, c in fixes) and clock.get_subscription_count() >= 4
            check('DDS_endpoint_discovery', ready, {'wheel_subscribers': [p.get_subscription_count() for p in wheels],
                  'clock_subscribers': clock.get_subscription_count()})
            if not ready:
                raise RuntimeError('DDS endpoints did not become ready before deadline')
            wall = time.monotonic()
            for index in range(160):
                spin_until(wall+index*.05)
                ns = EPOCH_NS+index*DT_NS
                clk = Clock()
                set_stamp(clk.clock, ns)
                clock.publish(clk)
                spin_until(time.monotonic()+.006)
                for publisher in wheels:
                    msg = VelocitySensor()
                    set_stamp(msg.header.stamp, ns)
                    msg.header.frame_id = 'base_link'
                    msg.velocity = SPEED*3.6
                    publisher.publish(msg)
                if 8 <= index <= 34 and index % 2 == 0:
                    for kind in ('line', 'circle'):
                        for channel in ('master', 'rover'):
                            publish_fix(kind, channel, index)
                # Each postinit receiver fix is isolated; no paired GNSS is needed.
                event = {60: ('master', 4., 0., 0, 'isolated_master'),
                         84: ('rover', -4., 0., 0, 'isolated_rover'),
                         100: ('master', 20., 2., 0, 'future_rejected'),
                         108: ('master', 0., 0., 0, 'valid_after_future'),
                         112: ('rover', 20., -1.1, 0, 'stale_rejected'),
                         120: ('master', 100., 0., 0, 'outlier_rejected'),
                         128: ('rover', 20., 0., -1, 'invalid_status_rejected'),
                         136: ('master', 10., -.8, 0, 'duplicate_rejected')}.get(index)
                if event:
                    channel, shift, delta, status, name = event
                    publish_fix('line', channel, index, shift, delta, status)
                    events.append({'name': name, 'at_seconds': index*.05,
                                   'channel': channel, 'along_shift_m': shift,
                                   'header_delta_s': delta, 'status': status})
                # Allow wheel/fix callbacks before the command-triggered output.
                spin_until(time.monotonic()+.006)
                msg = DriverControllerCommand()
                set_stamp(msg.header.stamp, ns)
                msg.position = 0
                controller.publish(msg)
                spin_until(time.monotonic()+.003)
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A ros2 run process exited during input')
            spin_until(time.monotonic()+.3)
            check('node_processes_alive', all(p.poll() is None for p in processes), [p.poll() for p in processes])

        def elapsed(row):
            return (row['stamp']-EPOCH_NS)*1e-9

        def selected(case, low, high):
            return [p for p in samples[case]['position'] if low <= elapsed(p) <= high]

        for case in CASES:
            data = samples[case]
            report['coverage'][case] = {key: len(value) for key, value in data.items()}
            velocity = data['velocity']
            check(case+'_velocity_coverage', len(velocity) >= 140, len(velocity))
            check(case+'_finite_base_link_velocity', bool(velocity) and all(
                  math.isfinite(v['value']) and v['frame'] == 'base_link' for v in velocity), len(velocity))
            check(case+'_strict_output_stamps', all(b['stamp'] > a['stamp'] for a, b in zip(velocity, velocity[1:])), len(velocity))
            check(case+'_no_relative_positions', all(p['frame'] == 'map' and p['child'] == 'base_link' for p in data['position']), len(data['position']))
            check(case+'_no_position_before_GNSS', all(elapsed(p) >= .4 for p in data['position']), len(data['position']))
            by_stamp = {v['stamp']: v for v in velocity}
            paired = [(p, by_stamp[p['stamp']]) for p in data['position'] if p['stamp'] in by_stamp]
            if case != 'no_gnss':
                check(case+'_position_coverage', len(paired) >= 120, len(paired))
                binding = max((abs(p['body_velocity'][0]-v['value']) for p, v in paired), default=math.inf)
                check(case+'_scalar_equals_body_twist_x', binding < 1e-9, binding)
            else:
                check('no_GNSS_suppresses_all_absolute_positions', len(data['position']) == 0, len(data['position']))
                check('no_GNSS_keeps_finite_wheel_velocity', bool(velocity) and all(math.isfinite(v['value']) and abs(v['value']-SPEED) < .001 for v in velocity), len(velocity))

        # Startup must transform all XYZ coordinates and the entire body attitude.
        for case in ('active', 'disabled'):
            points = selected(case, .7, 2.8)
            errors = [math.dist(p['xyz'], map_point(line_point(50.+SPEED*elapsed(p)), yaw, translation)) for p in points]
            angles = [quaternion_error(p['quaternion'], quaternion(LINE_YAW+yaw, -LINE_PITCH)) for p in points]
            check(case+'_startup_map_XYZ', bool(errors) and max(errors) < .03, {'count': len(errors), 'max_m': max(errors, default=None)})
            check(case+'_map_rotated_orientation', bool(angles) and max(angles) < .001, {'count': len(angles), 'max_rad': max(angles, default=None)})
            norm_error = max((abs(sum(q*q for q in p['quaternion'])-1.) for p in points), default=math.inf)
            check(case+'_unit_orientation_quaternion', norm_error < 1e-9, norm_error)
            check(case+'_world_transform_does_not_rotate_body_twist', bool(points) and max(math.dist(p['body_velocity'], (SPEED, 0., 0.)) for p in points) < .001, len(points))

        base = {p['stamp']: p for p in samples['disabled']['position']}
        differences = []
        for p in samples['active']['position']:
            if p['stamp'] not in base:
                continue
            a = unmap_point(p['xyz'], yaw, translation)
            b = unmap_point(base[p['stamp']]['xyz'], yaw, translation)
            differences.append((elapsed(p), sum((a[k]-b[k])*FORWARD[k] for k in range(3))))
        for name, low, high, expected in [
            ('before_sparse_fix_identical', .7, 2.8, 0.),
            ('isolated_master_corrects_phase', 3.15, 3.9, 1.),
            ('isolated_rover_corrects_phase', 4.35, 4.9, -.25),
            ('future_fix_does_not_jump', 5.05, 5.25, -.25),
            ('valid_after_future_and_invalid_fixes_stable', 5.55, 7.9, -.1875),
        ]:
            values = [v for t, v in differences if low <= t <= high]
            error = max((abs(v-expected) for v in values), default=math.inf)
            check(name, len(values) >= 3 and error < .04,
                  {'count': len(values), 'expected_m': expected, 'max_error_m': error})
        disabled_points = selected('disabled', 2.2, 7.9)
        disabled_error = max((math.dist(p['xyz'], map_point(line_point(50.+SPEED*elapsed(p)), yaw, translation))
                              for p in disabled_points), default=math.inf)
        check('disabled_sparse_GNSS_ignores_all_later_fixes', disabled_error < .03, disabled_error)
        counters = samples['active']['diagnostics']
        accepted = max((d[5] for d in counters if len(d) >= 8), default=-1)
        rejected = max((d[6] for d in counters if len(d) >= 8), default=-1)
        check('exactly_three_valid_sparse_corrections', accepted == 3, accepted)
        check('stale_and_outlier_are_counted_rejected', rejected >= 2, rejected)

        circle_points = selected('circle', .7, 7.9)
        expected_speed = SPEED*math.sqrt(1-(BOGIE/(2*RADIUS))**2)
        speed_errors = [abs(p['body_velocity'][0]-expected_speed) for p in circle_points]
        position_errors, orientation_errors = [], []
        for p in circle_points:
            angle = (100.+SPEED*elapsed(p))/RADIUS
            position_errors.append(math.dist(p['xyz'], map_point((RADIUS*math.cos(angle), RADIUS*math.sin(angle), 0.), yaw, translation)))
            heading = angle+math.pi/2-math.asin(BOGIE/(2*RADIUS))+yaw
            orientation_errors.append(quaternion_error(p['quaternion'], quaternion(heading, 0.)))
        check('circle_body_speed_differs_from_scalar_route_speed', bool(speed_errors) and max(speed_errors) < .003 and abs(expected_speed-SPEED) > .08,
              {'count': len(speed_errors), 'expected_body_x': expected_speed, 'scalar_route_speed': SPEED, 'max_error': max(speed_errors, default=None)})
        check('circle_map_XYZ_including_periodic_seam', bool(position_errors) and max(position_errors) < .04, max(position_errors, default=None))
        check('circle_map_attitude_including_periodic_seam', bool(orientation_errors) and max(orientation_errors) < .003, max(orientation_errors, default=None))
        report['status'] = 'PASS' if all(a['pass'] for a in assertions) else 'FAIL'
    except Exception:
        report['status'] = 'ERROR'
        report['exception'] = traceback.format_exc()
    finally:
        for process in processes:
            stop(process)
        for log in logs:
            log.close()
        report['elapsed_wall_s'] = time.monotonic()-started
        report['samples_file'] = 'checker_compat_samples.json'
        (args.out/'checker_compat_samples.json').write_text(json.dumps(json_safe(samples), indent=2, allow_nan=False))
        (args.out/'checker_compat.json').write_text(json.dumps(json_safe(report), indent=2, allow_nan=False))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(json_safe(report), indent=2, allow_nan=False))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--calibration', type=Path,
                        default=Path(__file__).resolve().parents[1]/'config/map_calibration.json')
    parser.add_argument('--domain', type=int, default=47)
    parser.add_argument('--discovery-timeout', type=float, default=4.)
    raise SystemExit(run(parser.parse_args()))
