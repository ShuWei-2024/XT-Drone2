#!/usr/bin/env python3
"""Compare Gazebo model motion with FAST-LIVO2 odometry by simulation time."""

import argparse
import bisect
import csv
import json
import math
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GazeboNode
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from measure_endurance import Probe, WIDE_MOTION, rss_kib, stamp_ns


SHORT_MOTION = (
    ('forward', 10, 0.05, 0.0, 0.0),
    ('stop_1', 4, 0.0, 0.0, 0.0),
    ('lateral', 10, 0.0, 0.05, 0.0),
    ('stop_2', 4, 0.0, 0.0, 0.0),
    ('reverse', 10, -0.05, 0.0, 0.0),
    ('stop_3', 4, 0.0, 0.0, 0.0),
)
SHORT_LOOP_MOTION = SHORT_MOTION + (
    ('return_lateral', 10, 0.0, -0.05, 0.0),
    ('stop_4', 4, 0.0, 0.0, 0.0),
)


def route_segments(name):
    if name == 'short':
        return SHORT_MOTION
    if name == 'short-loop':
        return SHORT_LOOP_MOTION
    return tuple((f'wide_{index}', *segment)
                 for index, segment in enumerate(WIDE_MOTION, start=1))


def drive_verified(stop, start, failed, commands, recorder, route, cycles,
                   speed_scale):
    gazebo = GazeboNode()
    publisher = gazebo.advertise('/model/astra_x500_motion/cmd_vel', Twist)
    if stop.wait(max(0.0, start - time.monotonic())):
        return
    try:
        cycle = 0
        while not stop.is_set():
            for label, duration, x, y, angular_z in route_segments(route):
                if stop.is_set():
                    return
                if x or y or angular_z:
                    duration /= speed_scale
                    x *= speed_scale
                    y *= speed_scale
                    angular_z *= speed_scale
                deadline = time.monotonic() + duration
                message = Twist()
                message.linear.x, message.linear.y = x, y
                message.angular.z = angular_z
                segment = f'{cycle + 1}_{label}' if route == 'short-loop' else label
                commands.append((segment, time.monotonic(), time.time(),
                                 recorder.latest_truth_stamp(), x, y, angular_z,
                                 publisher.has_connections()))
                while not stop.is_set() and time.monotonic() < deadline:
                    if not publisher.has_connections() or not publisher.publish(message):
                        failed.set()
                        return
                    stop.wait(0.2)
            cycle += 1
            if route == 'short' or (route == 'short-loop' and cycle >= cycles):
                commands.append(('post_route', time.monotonic(), time.time(),
                                 recorder.latest_truth_stamp(), 0.0, 0.0, 0.0,
                                 publisher.has_connections()))
                return
    finally:
        publisher.publish(Twist())


class Recorder(Probe):
    def __init__(self):
        super().__init__()
        self.odom_rows = []
        self.truth_rows = []
        self.raw_clouds = {}
        self.peak_rss_kib = 0
        self.truth_lock = threading.Lock()
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(Pose_V, '/world/astra_motion/dynamic_pose/info', self.truth)
        self.create_subscription(PointCloud2, '/sensors/lidar/points',
                                 self.raw_cloud, qos_profile_sensor_data)

    def odom(self, msg):
        super().odom(msg)
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        row = (stamp_ns(msg.header), position.x, position.y, position.z,
               orientation.x, orientation.y, orientation.z, orientation.w)
        if all(math.isfinite(value) for value in row[1:]):
            self.odom_rows.append(row)

    def truth(self, msg):
        pose = next((item for item in msg.pose if item.name == 'astra_x500_motion'), None)
        if pose is None:
            return
        stamp = msg.header.stamp
        p, q = pose.position, pose.orientation
        row = (stamp.sec * 1_000_000_000 + stamp.nsec,
               p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        with self.truth_lock:
            if not self.truth_rows or row[0] - self.truth_rows[-1][0] >= 20_000_000:
                self.truth_rows.append(row)

    def latest_truth_stamp(self):
        with self.truth_lock:
            return self.truth_rows[-1][0] if self.truth_rows else 0

    def raw_cloud(self, msg):
        if not self.odom_rows:
            return
        elapsed = (stamp_ns(msg.header) - self.odom_rows[0][0]) / 1e9
        for target in (30, 60, 70, 75):
            if target not in self.raw_clouds and elapsed >= target:
                points = point_cloud2.read_points_numpy(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)
                self.raw_clouds[target] = np.asarray(points, dtype=np.float32)


def yaw(row):
    _, _, _, _, qx, qy, qz, qw = row
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def aligned_rows(odom, truth):
    truth = sorted(truth)
    stamps = [row[0] for row in truth]
    pairs = []
    for estimate in odom:
        index = bisect.bisect_left(stamps, estimate[0])
        candidates = truth[max(0, index - 1):min(len(truth), index + 1)]
        if not candidates:
            continue
        actual = min(candidates, key=lambda row: abs(row[0] - estimate[0]))
        if abs(actual[0] - estimate[0]) <= 50_000_000:
            pairs.append((estimate, actual))
    if not pairs:
        return []

    estimate_origin, actual_origin = pairs[0]
    heading = yaw(actual_origin) - yaw(estimate_origin)
    cos_heading, sin_heading = math.cos(heading), math.sin(heading)
    rows = []
    for estimate, actual in pairs:
        ex, ey = estimate[1] - estimate_origin[1], estimate[2] - estimate_origin[2]
        aligned_x = cos_heading * ex - sin_heading * ey
        aligned_y = sin_heading * ex + cos_heading * ey
        truth_x = actual[1] - actual_origin[1]
        truth_y = actual[2] - actual_origin[2]
        rows.append(((estimate[0] - estimate_origin[0]) / 1e9,
                     (actual[0] - estimate[0]) / 1e6,
                     truth_x, truth_y, aligned_x, aligned_y,
                     math.hypot(truth_x - aligned_x, truth_y - aligned_y),
                     estimate[0], actual[0]))
    return rows


def write_csv(path, header, rows):
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_prefix')
    parser.add_argument('--route', choices=('wide', 'short', 'short-loop'), default='wide')
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--speed-scale', type=float, default=1.0)
    parser.add_argument('--seconds', type=float, default=131)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--max-step-m', type=float, default=0.5)
    parser.add_argument('--max-rss-kib', type=int, default=800_000)
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('--cycles must be positive')
    if not 0.0 < args.speed_scale <= 1.0:
        parser.error('--speed-scale must be in (0, 1]')

    rclpy.init()
    recorder = Recorder()
    stop = threading.Event()
    failed = threading.Event()
    started = time.monotonic()
    commands = []
    rss_samples = []
    next_rss_sample = started
    driver = threading.Thread(target=drive_verified,
                              args=(stop, started + 10, failed, commands,
                                    recorder, args.route, args.cycles,
                                    args.speed_scale), daemon=True)
    driver.start()
    abort = ''
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if failed.is_set():
                abort = 'Gazebo velocity command failed'
            elif recorder.counts['odom_invalid']:
                abort = 'non-finite odometry'
            elif recorder.max_step_m > args.max_step_m:
                abort = 'odometry step exceeded guard'
            elif rss_kib(args.pid) > args.max_rss_kib:
                abort = 'core RSS exceeded guard'
            now = time.monotonic()
            if now >= next_rss_sample:
                sample = (recorder.latest_truth_stamp(), time.time(), rss_kib(args.pid))
                rss_samples.append(sample)
                recorder.peak_rss_kib = max(recorder.peak_rss_kib, sample[2])
                next_rss_sample += 1.0
            if abort:
                break
    finally:
        stop.set()
        driver.join(timeout=12)
        with recorder.truth_lock:
            truth = list(recorder.truth_rows)
        odom = list(recorder.odom_rows)
        sync = dict(recorder.sync)
        counts = dict(recorder.counts)
        max_step = recorder.max_step_m
        peak_rss = recorder.peak_rss_kib
        clouds = dict(recorder.raw_clouds)
        command_rows = [(label, round(sent - started, 3), epoch, stamp,
                         x, y, angular_z, connected)
                        for label, sent, epoch, stamp, x, y, angular_z, connected in commands]
        recorder.destroy_node()
        rclpy.try_shutdown()

    header = ('stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    write_csv(args.output_prefix + '_odom.csv', header, odom)
    write_csv(args.output_prefix + '_gazebo.csv', header, truth)
    aligned = aligned_rows(odom, truth)
    write_csv(args.output_prefix + '_aligned.csv',
              ('elapsed_s', 'stamp_delta_ms', 'gazebo_dx', 'gazebo_dy',
               'odom_dx_rotated', 'odom_dy_rotated', 'planar_error_m',
               'odom_stamp_ns', 'gazebo_stamp_ns'), aligned)
    for target, points in clouds.items():
        np.save(args.output_prefix + f'_cloud_{target}s.npy', points)
    write_csv(args.output_prefix + '_commands.csv',
              ('segment', 'elapsed_wall_s', 'epoch_s', 'sim_stamp_ns',
               'linear_x', 'linear_y', 'angular_z', 'connected'),
              command_rows)
    write_csv(args.output_prefix + '_rss.csv',
              ('sim_stamp_ns', 'epoch_s', 'core_rss_kib'), rss_samples)
    summary = {
        'route': args.route,
        'speed_scale': args.speed_scale,
        'abort': abort,
        'counts': counts,
        'sync': sync,
        'odom_count': len(odom),
        'gazebo_count': len(truth),
        'paired_count': len(aligned),
        'max_step_m': max_step,
        'peak_rss_kib': peak_rss,
        'max_stamp_delta_ms': max((abs(row[1]) for row in aligned), default=None),
        'max_planar_error_m': max((row[6] for row in aligned), default=None),
    }
    with open(args.output_prefix + '_summary.json', 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
        stream.write('\n')
    print('recorded:', {'odom': len(odom), 'gazebo': len(truth), 'paired': len(aligned)})
    print('sync:', sync)
    print('counts:', counts, 'max_step_m:', max_step, 'peak_rss_kib:', peak_rss,
          'clouds:', {target: len(points) for target, points in clouds.items()})
    print('commands:', command_rows)
    if aligned:
        print('alignment:', {'max_stamp_delta_ms': max(abs(row[1]) for row in aligned),
                             'max_planar_error_m': max(row[6] for row in aligned),
                             'initial_yaw_offset_rad': yaw(truth[0]) - yaw(odom[0])})
        for second in (60, 90, 120, 130):
            row = min(aligned, key=lambda item: abs(item[0] - second))
            print(f'at_{second}s:', row[:7])
    if abort:
        print('aborted:', abort)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
