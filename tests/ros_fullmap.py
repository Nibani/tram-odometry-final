"""DDS performance/geometry fixture using both installed full-size release maps.

Run from the source tree after sourcing a built ROS Humble workspace:
    python3 tests/ros_fullmap.py --out results/ci_fullmap

The source-tree map_geometry module is a NumPy-only test oracle. The tested node
and its two maps come from the installed ament package, not from test fixtures.
This measures a synthetic eight-second trip, not real-bag accuracy or a hardware
worst case. Initial map matching is included in the 1--7.9 second timing window.
"""
import argparse
import hashlib
import json
import math
import os
import resource
import subprocess
import sys
import time
import traceback
from pathlib import Path

for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = "1"
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from map_geometry import HEIGHT, MASTER, ROVER, Route
from ros_smoke import executable, limit_cores, lla, rss_kib, stamp_ns


SPEED = 5.0
SPEED_SCALE = 1.0003350854241781
START_S = 800.0
INPUT_SECONDS = 8.0


def installed_maps():
    config = Path(get_package_share_directory("reserve_odometry")) / "config"
    paths = [config / "route_loop.csv", config / "route_hairpin_hypothesis.csv"]
    routes, metadata = [], []
    for path in paths:
        raw = path.read_bytes()
        route = Route.load(path)
        if len(route.xyz) < 10000:
            raise ValueError(f"Expected a full-size map, got {len(route.xyz)} vertices: {path}")
        closure = float(np.linalg.norm(route.xyz[-1] - route.xyz[0]))
        if closure > 1e-6:
            raise ValueError(f"Expected a closed release route: {path}, gap={closure}")
        routes.append(route)
        metadata.append({
            "path": str(path), "vertices": len(route.xyz),
            "length_m": float(route.s[-1]), "closure_gap_m": closure,
            "sha256": hashlib.sha256(raw).hexdigest(),
            "canonical_lf_sha256": hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest(),
        })
    return paths, routes[0], metadata


def run_case(output):
    paths, route, maps = installed_maps()
    os.environ["ROS_DOMAIN_ID"] = "46"
    rclpy.init()
    node = Node("reserve_odometry_test_fullmap")
    front = node.create_publisher(VelocitySensor, "/vehicle/front_bogie_velocity", 10)
    rear = node.create_publisher(VelocitySensor, "/vehicle/rear_bogie_velocity", 10)
    command = node.create_publisher(DriverControllerCommand, "/vehicle/driver_position_cmd", 10)
    master = node.create_publisher(NavSatFix, "/sensing/gnss/master/fix", 10)
    rover = node.create_publisher(NavSatFix, "/sensing/gnss/rover/fix", 10)
    velocities, poses, gnss_elapsed = [], [], []
    node.create_subscription(
        VelocitySensor, "/result/velocity",
        lambda m: velocities.append((stamp_ns(m.header.stamp), m.velocity, time.time_ns())), 100,
    )
    node.create_subscription(
        Odometry, "/result/position",
        lambda m: poses.append((stamp_ns(m.header.stamp), m.pose.pose.position.x,
                               m.pose.pose.position.y, m.pose.pose.position.z,
                               m.header.frame_id)), 100,
    )
    parameters = {
        "use_sim_time": "false", "route_files": json.dumps([str(p) for p in paths]),
        "closed_route": "true", "wheel_divisor": "3.6",
        "speed_scale": repr(SPEED_SCALE), "initial_gnss_seconds": "3.0",
        "align_initial_position": "false", "compensate_velocity_lever_arm": "false",
        "output_point_x": "0.0", "output_point_z": "0.0",
    }
    args = [executable(), "--ros-args"]
    for key, value in parameters.items():
        args.extend(["-p", f"{key}:={value}"])
    # Undo only the declared release calibration in the synthetic input, so the
    # reference front-bogie travel speed remains exactly SPEED metres/second.
    raw_wheel = SPEED * 3.6 / SPEED_SCALE
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    begin = time.monotonic()
    process, start, peak_rss, affinity = None, None, 0, []
    input_count = 0
    try:
        with (output / "fullmap_node.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       preexec_fn=limit_cores)
            affinity = sorted(os.sched_getaffinity(process.pid))
            until = time.monotonic() + 2.0
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
                peak_rss = max(peak_rss, rss_kib(process.pid))
            if process.poll() is not None:
                raise RuntimeError("ROS node exited during DDS discovery")
            start = node.get_clock().now().nanoseconds
            wall = time.monotonic()
            for i in range(160):
                target = wall + i * .05
                while time.monotonic() < target:
                    rclpy.spin_once(node, timeout_sec=.002)
                stamp = node.get_clock().now().to_msg()
                elapsed = (stamp_ns(stamp) - start) * 1e-9
                if elapsed >= INPUT_SECONDS:
                    raise RuntimeError("Input publisher overran its eight-second schedule")
                for publisher in (front, rear):
                    msg = VelocitySensor()
                    msg.header.stamp = stamp
                    msg.header.frame_id = "base_link"
                    msg.velocity = raw_wheel
                    publisher.publish(msg)
                if i % 2 == 0 and elapsed <= 3.0:
                    # Exact chord/zero-roll antenna geometry, including height.
                    for publisher, x in ((master, MASTER), (rover, ROVER)):
                        point = route.pose(START_S + SPEED * elapsed, x=x, z=HEIGHT)[0]
                        msg = NavSatFix()
                        msg.header.stamp = stamp
                        msg.status.status = 0
                        msg.latitude, msg.longitude, msg.altitude = lla(point)
                        publisher.publish(msg)
                    gnss_elapsed.append(elapsed)
                msg = DriverControllerCommand()
                msg.header.stamp = stamp
                msg.position = 0
                command.publish(msg)
                input_count += 1
                rclpy.spin_once(node, timeout_sec=.003)
                peak_rss = max(peak_rss, rss_kib(process.pid))
            until = wall + INPUT_SECONDS + .3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
                peak_rss = max(peak_rss, rss_kib(process.pid))
            if process.poll() is not None:
                raise RuntimeError("ROS node crashed")
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        (output / "fullmap_samples.json").write_text(json.dumps({
            "start_ns": start,
            "velocity_columns": ["stamp_ns", "velocity_mps", "receive_wall_ns"],
            "pose_columns": ["stamp_ns", "x_m", "y_m", "z_m", "frame_id"],
            "velocity": velocities, "poses": poses,
            "gnss_pair_elapsed_s": gnss_elapsed,
        }), encoding="utf-8")

    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    elapsed_wall = time.monotonic() - begin
    cpu_cores = (after.ru_utime + after.ru_stime - before.ru_utime - before.ru_stime) / elapsed_wall
    valid = [v for v in velocities if 1.0 < (v[0] - start) * 1e-9 < 7.9]
    selected = [p for p in poses if 3.2 < (p[0] - start) * 1e-9 < 7.9]
    gaps = [(b[0] - a[0]) * 1e-9 for a, b in zip(valid, valid[1:])]
    latencies = sorted((v[2] - v[0]) * 1e-6 for v in valid)
    errors = []
    for point in selected:
        expected = route.pose(START_S + SPEED * (point[0] - start) * 1e-9)[0]
        errors.append(float(np.linalg.norm(np.asarray(point[1:4]) - expected)))
    mean_hz = ((len(valid) - 1) / ((valid[-1][0] - valid[0][0]) * 1e-9)
               if len(valid) > 1 and valid[-1][0] > valid[0][0] else 0.0)
    p99 = latencies[int(.99 * (len(latencies) - 1))] if latencies else None
    max_latency = max(latencies) if latencies else None
    max_gap = max(gaps) if gaps else None
    position_error = max(errors) if errors else None
    checks = {
        "two_cpu_affinity": len(affinity) == 2,
        "node_memory_below_512_MiB": 0 < peak_rss < 512 * 1024,
        "mean_cpu_below_two_cores": cpu_cores < 2.01,
        "input_count_160": input_count == 160,
        "initial_gnss_only": bool(gnss_elapsed) and max(gnss_elapsed) <= 3.0,
        "output_count_at_least_100": len(velocities) >= 100,
        "velocity_close_to_5_mps": bool(valid) and all(math.isfinite(v[1]) and abs(v[1] - SPEED) < .1 for v in valid),
        "monotonic_stamps": bool(gaps) and min(gaps) > 0,
        "publication_at_least_10_Hz": mean_hz >= 10.0,
        "max_gap_below_250_ms": max_gap is not None and max_gap < .25,
        "latency_p99_below_100_ms": p99 is not None and p99 < 100.0,
        "latency_max_below_250_ms": max_latency is not None and max_latency < 250.0,
        "initialized_map_enu": bool(selected) and all(p[4] == "map_enu" for p in selected),
        "position_XYZ_error_below_0_3_m": bool(errors) and all(math.isfinite(e) and e < .3 for e in errors),
    }
    return {
        "mode": "installed_full_maps", "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks, "maps": maps, "closed_route": True,
        "truth_route": paths[0].name, "start_s_m": START_S, "truth_speed_mps": SPEED,
        "speed_scale": SPEED_SCALE, "raw_wheel_input": raw_wheel,
        "discovery_s": 2.0, "input_duration_s": INPUT_SECONDS, "input_hz": 20.0,
        "inputs_per_wheel": input_count, "gnss_pairs": len(gnss_elapsed),
        "last_gnss_header_elapsed_s": max(gnss_elapsed) if gnss_elapsed else None,
        "timing_window_s": [1.0, 7.9], "position_window_s": [3.2, 7.9],
        "outputs": len(velocities), "timing_outputs": len(valid), "position_outputs": len(selected),
        "mean_hz": mean_hz, "max_gap_s": max_gap,
        "latency_p99_ms": p99, "latency_max_ms": max_latency,
        "latency_definition": "input header wall-clock time to velocity subscriber receipt",
        "position_max_error_m": position_error,
        "node_peak_rss_kib": peak_rss, "mean_cpu_cores": cpu_cores,
        "cpu_observation_wall_s": elapsed_wall,
        "affinity_cores": len(affinity), "affinity_cpu_ids": affinity,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results/ci_fullmap"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    try:
        case = run_case(args.out)
    except Exception as exc:
        case = {"mode": "installed_full_maps", "status": "ERROR",
                "error": str(exc), "traceback": traceback.format_exc()}
    report = {"ros": "Humble", "cases": [case],
              "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss}
    text = json.dumps(report, indent=2, allow_nan=False)
    (args.out / "ros_fullmap.json").write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if case["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
