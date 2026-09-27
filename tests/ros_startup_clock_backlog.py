"""Actual-DDS startup clock/backlog regression, synthetic only.

Copy beside ros_checker_compat.py and run in a sourced Humble environment:
  python3 tests/ros_startup_clock_backlog.py --out results/ci_startup_clock
Use --expected buggy only to establish the old negative control explicitly.
No reference or GNSS messages are used. Normal runtime is under five seconds.
"""
from pathlib import Path
import argparse, hashlib, importlib.util, json, math, os, subprocess, time, traceback


def run(args):
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor
    spec = importlib.util.spec_from_file_location('clock_backlog_helpers', args.helpers)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    args.out.mkdir(parents=True, exist_ok=True)
    calibration = json.loads(args.calibration.read_text())
    route = args.out/'synthetic_line.csv'
    route.write_text('s,x,y,z\n0,0,0,0\n500,' + ','.join(map(repr, helper.line_point(500.))) + '\n')
    parameters = helper.parameters(calibration, route.resolve(), 'startup_clock')
    parameters.update(initial_gnss_seconds=3., sparse_gnss_correction=True,
                      gnss_xy_residual_correction=True, body_velocity_output=False,
                      publish_relative_position=False, speed_scale=1.)
    command = helper.command(parameters, 'startup_clock')
    epoch, step = 100_000_000_000, 50_000_000
    wanted = [epoch-3_000_000_000+i*step for i in range(61)]
    samples, assertions = [], []
    report = {'status': 'NOT_COMPLETED', 'expected': args.expected,
              'synthetic_only': True, 'reference_topic_used': False,
              'command': command, 'assertions': assertions,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'helper_sha256': hashlib.sha256(args.helpers.read_bytes()).hexdigest(),
              'calibration_sha256': hashlib.sha256(args.calibration.read_bytes()).hexdigest()}
    process = log = node = None
    phase = 'discovery'
    started = time.monotonic()

    def check(name, passed, evidence):
        assertions.append({'name': name, 'pass': bool(passed), 'evidence': evidence})

    try:
        rclpy.init()
        node = Node('startup_clock_backlog_fixture')

        def spin(seconds):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.002, max(0., deadline-time.monotonic())))

        clock = node.create_publisher(Clock, '/clock', 10)
        wheels = [node.create_publisher(VelocitySensor, '/compat/input/'+side, 100)
                  for side in ('front', 'rear')]
        controller = node.create_publisher(DriverControllerCommand, '/compat/input/command', 100)

        def receive(message):
            samples.append({'stamp': helper.stamp_ns(message.header.stamp),
                            'velocity': message.velocity, 'frame': message.header.frame_id,
                            'phase': phase, 'wall_s': time.monotonic()-started})

        node.create_subscription(VelocitySensor, '/compat/startup_clock/velocity', receive, 200)
        log = (args.out/'estimator_node.log').open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, preexec_fn=helper.child_setup)
        publishers = [clock, *wheels, controller]
        deadline = time.monotonic()+args.discovery_timeout
        while time.monotonic() < deadline:
            spin(.02)
            if all(p.get_subscription_count() >= 1 for p in publishers):
                break
            if process.poll() is not None:
                raise RuntimeError('Node exited during discovery')
        ready = all(p.get_subscription_count() >= 1 for p in publishers)
        check('DDS_endpoint_discovery', ready, [p.get_subscription_count() for p in publishers])
        if not ready:
            raise RuntimeError('DDS discovery timeout')

        def publish_clock(ns):
            message = Clock()
            helper.set_stamp(message.clock, ns)
            clock.publish(message)

        def publish_wheels(ns):
            for publisher in wheels:
                message = VelocitySensor()
                helper.set_stamp(message.header.stamp, ns)
                message.header.frame_id = 'base_link'
                message.velocity = 18.
                publisher.publish(message)

        phase = 'clock_zero'
        publish_clock(0)
        spin(.03)
        publish_wheels(wanted[0])
        spin(.04)  # Ensure the node receives the wheel while ROS time is zero.
        phase = 'clock_jump_before_commands'
        publish_clock(epoch)
        spin(.03)
        # A second explicit tick makes the 50ms ROS timer due even if the first
        # jump reset its deadline. ROS time is held here throughout the backlog.
        publish_clock(epoch+step)
        spin(.04)
        premature = list(samples)
        phase = 'command_backlog'
        backlog_start = time.monotonic()
        for ns in wanted:
            publish_wheels(ns)
            spin(.0025)
            message = DriverControllerCommand()
            helper.set_stamp(message.header.stamp, ns)
            message.position = 0
            if ns == wanted[0]:
                report['first_command_publish_wall_s'] = time.monotonic()-started
            controller.publish(message)
            spin(.0025)
        backlog_wall = time.monotonic()-backlog_start
        phase = 'drain'
        spin(.15)
        check('node_stays_alive', process.poll() is None, process.poll())
        got = [sample['stamp'] for sample in samples]
        missing = [stamp for stamp in wanted if stamp not in got]
        unexpected = [stamp for stamp in got if stamp not in wanted]
        report.update(wanted_stamps=wanted, received_stamps=got, missing_stamps=missing,
                      premature_samples=premature, unexpected_stamps=unexpected,
                      backlog_wall_s=backlog_wall,
                      schedule={'initial_clock_ns': 0, 'preclock_wheel_header_ns': wanted[0],
                                'clock_ticks_ns': [epoch, epoch+step],
                                'held_clock_during_backlog_ns': epoch+step})
        if args.expected == 'fixed':
            check('no_output_before_first_command', not premature, premature)
            check('all_backlog_command_epochs_preserved', got == wanted,
                  {'wanted_count': len(wanted), 'received_count': len(got),
                   'missing': missing, 'unexpected': unexpected})
            check('strictly_ordered_output_headers', all(b > a for a, b in zip(got, got[1:])), got)
            check('finite_continuous_5mps', bool(samples) and all(
                  math.isfinite(row['velocity']) and abs(row['velocity']-5.) < .001
                  and row['frame'] == 'base_link' for row in samples),
                  [row['velocity'] for row in samples])
            # Separate fresh node: verify the startup guard is a bounded grace,
            # not a permanent disabling of command-free timer output.
            helper.stop(process)
            log.close()
            spin(.04)
            fallback = []
            node.create_subscription(VelocitySensor, '/compat/startup_wheelonly/velocity',
                lambda m: fallback.append({'stamp': helper.stamp_ns(m.header.stamp),
                                           'velocity': m.velocity}), 100)
            wheel_command = helper.command(parameters, 'startup_wheelonly')
            wheel_command = [value.replace('/compat/input/', '/compat/wheelonly_input/') for value in wheel_command]
            wheels = [node.create_publisher(VelocitySensor, '/compat/wheelonly_input/'+side, 100)
                      for side in ('front', 'rear')]
            controller = node.create_publisher(DriverControllerCommand, '/compat/wheelonly_input/command', 100)
            publishers = [clock, *wheels, controller]
            report['wheel_only_command'] = wheel_command
            log = (args.out/'wheel_only_node.log').open('w')
            process = subprocess.Popen(wheel_command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, preexec_fn=helper.child_setup)
            deadline = time.monotonic()+args.discovery_timeout
            while time.monotonic() < deadline:
                spin(.02)
                if all(p.get_subscription_count() >= 1 for p in publishers):
                    break
                if process.poll() is not None:
                    raise RuntimeError('Wheel-only node exited during discovery')
            ready = all(p.get_subscription_count() >= 1 for p in publishers)
            check('wheel_only_DDS_discovery', ready, [p.get_subscription_count() for p in publishers])
            if not ready:
                raise RuntimeError('Wheel-only DDS discovery timeout')
            publish_clock(0)
            spin(.03)
            publish_wheels(wanted[0])
            spin(.04)
            publish_clock(epoch)
            spin(.03)
            publish_clock(epoch+step)
            spin(.04)
            check('wheel_only_startup_grace_is_quiet', not fallback, list(fallback))
            publish_wheels(epoch+step)
            spin(.01)
            publish_clock(epoch+200_000_000)
            spin(.06)
            check('wheel_only_timer_recovers_after_grace', bool(fallback) and all(
                  row['stamp'] >= epoch+100_000_000 and math.isfinite(row['velocity'])
                  and abs(row['velocity']-5.) < .001 for row in fallback), fallback)
            report['wheel_only_samples'] = fallback
        else:
            # Negative control must actually expose the hypothesized mechanism;
            # absence of the bug is not silently reported as a useful control.
            check('old_timer_publishes_ahead_of_backlog', any(row['stamp'] >= epoch for row in premature), premature)
            check('old_timer_causes_lost_command_epochs', len(missing) >= 50, missing)
        report['status'] = 'PASS' if all(row['pass'] for row in assertions) else 'FAIL'
    except Exception:
        report['status'] = 'ERROR'
        report['exception'] = traceback.format_exc()
    finally:
        if process is not None:
            helper.stop(process)
        if log is not None:
            log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report['elapsed_wall_s'] = time.monotonic()-started
        (args.out/'startup_clock_backlog.json').write_text(json.dumps(helper.json_safe(report), indent=2, allow_nan=False))
        (args.out/'samples.json').write_text(json.dumps(helper.json_safe(samples), indent=2, allow_nan=False))
    print(json.dumps({'status': report['status'], 'out': str(args.out),
                      'expected': args.expected, 'elapsed_wall_s': report['elapsed_wall_s']}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--helpers', type=Path, default=Path(__file__).with_name('ros_checker_compat.py'))
    parser.add_argument('--calibration', type=Path, default=Path(__file__).resolve().parents[1]/'config/map_calibration.json')
    parser.add_argument('--domain', type=int, default=54)
    parser.add_argument('--discovery-timeout', type=float, default=3.)
    parser.add_argument('--expected', choices=['fixed', 'buggy'], default='fixed')
    raise SystemExit(run(parser.parse_args()))
