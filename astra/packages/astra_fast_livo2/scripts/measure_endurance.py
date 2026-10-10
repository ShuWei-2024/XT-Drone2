#!/usr/bin/env python3
"""Measure long-running FAST-LIVO2 outputs during bounded Gazebo motion."""

import argparse
import csv
import math
import threading
import time
from collections import Counter

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GazeboNode
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


MOTION = (
    (8, 0.06, 0.0, 0.0), (4, 0.0, 0.0, 0.0),
    (8, 0.0, 0.06, 0.0), (4, 0.0, 0.0, 0.0),
    (8, 0.0, 0.0, 0.015), (4, 0.0, 0.0, 0.0),
    (8, -0.06, 0.0, 0.0), (4, 0.0, 0.0, 0.0),
    (8, 0.0, -0.06, 0.0), (4, 0.0, 0.0, 0.0),
    (8, 0.0, 0.0, -0.015), (4, 0.0, 0.0, 0.0),
)

WIDE_MOTION = (
    (50, 0.08, 0.0, 0.0), (4, 0.0, 0.0, 0.0),
    (50, 0.0, 0.08, 0.0), (4, 0.0, 0.0, 0.0),
    (50, -0.08, 0.0, 0.0), (4, 0.0, 0.0, 0.0),
    (50, 0.0, -0.08, 0.0), (4, 0.0, 0.0, 0.0),
    (10, 0.0, 0.0, 0.02), (4, 0.0, 0.0, 0.0),
    (10, 0.0, 0.0, -0.02), (4, 0.0, 0.0, 0.0),
)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def rss_kib(pid):
    if not pid:
        return 0
    try:
        with open(f'/proc/{pid}/status', encoding='ascii') as stream:
            return next(int(line.split()[1]) for line in stream if line.startswith('VmRSS:'))
    except (OSError, StopIteration):
        return 0


def command_velocity(publisher, x=0.0, y=0.0, yaw=0.0):
    message = Twist()
    message.linear.x, message.linear.y = x, y
    message.angular.z = yaw
    return publisher.publish(message)


def drive(stop, start, motion, failed, speed_scale):
    gazebo = GazeboNode()
    publisher = gazebo.advertise('/model/astra_x500_motion/cmd_vel', Twist)
    if stop.wait(max(0.0, start - time.monotonic())):
        return
    try:
        while not stop.is_set():
            for duration, x, y, yaw in motion:
                if stop.is_set():
                    return
                deadline = time.monotonic() + duration / speed_scale
                while not stop.is_set() and time.monotonic() < deadline:
                    if not publisher.has_connections() or not command_velocity(
                            publisher, x * speed_scale, y * speed_scale,
                            yaw * speed_scale):
                        failed.set()
                        return
                    stop.wait(0.2)
    finally:
        command_velocity(publisher)


class Probe(Node):
    def __init__(self):
        super().__init__('fast_livo2_endurance_probe')
        self.counts = Counter()
        self.last_stamp = {}
        self.last_received = {}
        self.max_gap = Counter()
        self.sync = {}
        self.origin = None
        self.last_position = None
        self.max_displacement_m = 0.0
        self.max_step_m = 0.0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.odom, qos)
        self.create_subscription(PointCloud2, '/cloud_registered', self.cloud, qos)
        self.create_subscription(DiagnosticArray, '/fast_livo2/diagnostics', self.diagnostics, 10)

    def record(self, name, header):
        now = time.monotonic()
        stamp = stamp_ns(header)
        if name in self.last_received:
            self.max_gap[name] = max(self.max_gap[name], now - self.last_received[name])
        if name in self.last_stamp and stamp < self.last_stamp[name]:
            self.counts[name + '_backward_stamp'] += 1
        self.last_received[name] = now
        self.last_stamp[name] = stamp
        self.counts[name] += 1

    def odom(self, msg):
        self.record('odom', msg.header)
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            self.counts['odom_invalid'] += 1
            return
        position = values[:3]
        if self.origin is None:
            self.origin = position
        if self.last_position is not None:
            self.max_step_m = max(self.max_step_m, math.dist(position, self.last_position))
        self.max_displacement_m = max(self.max_displacement_m, math.dist(position, self.origin))
        self.last_position = position
        if abs(sum(value * value for value in values[3:]) - 1.0) > 0.01:
            self.counts['odom_bad_quaternion'] += 1

    def cloud(self, msg):
        self.record('cloud', msg.header)
        if msg.width * msg.height:
            self.counts['cloud_nonempty'] += 1
            if not {'x', 'y', 'z'}.issubset(field.name for field in msg.fields):
                self.counts['cloud_missing_xyz'] += 1

    def diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'astra_fast_livo2/input_adapter':
                self.sync = {item.key: item.value for item in status.values}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path')
    parser.add_argument('--seconds', type=float, default=600.0)
    parser.add_argument('--pid', type=int)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--wide', action='store_true',
                        help='Command the nominal 4 m route; a landmark blocks its lateral leg.')
    parser.add_argument('--speed-scale', type=float, default=1.0,
                        help='Scale velocity and inverse-scale segment duration to preserve the route.')
    parser.add_argument('--max-odom-displacement-m', type=float, default=0.0)
    parser.add_argument('--max-odom-step-m', type=float, default=0.0)
    parser.add_argument('--max-rss-kib', type=int, default=0)
    args = parser.parse_args()
    if not 0 < args.speed_scale <= 1:
        parser.error('--speed-scale must be in (0, 1]')
    rclpy.init()
    probe = Probe()
    stop = threading.Event()
    driver_failed = threading.Event()
    started = time.monotonic()
    driver = None
    if args.drive:
        motion = WIDE_MOTION if args.wide else MOTION
        driver = threading.Thread(target=drive, args=(stop, started + 10, motion,
                                                        driver_failed, args.speed_scale), daemon=True)
        driver.start()
    next_report = started
    rows = []
    abort_reason = ''

    def snapshot(elapsed):
        position = probe.last_position or ('', '', '')
        return (round(elapsed, 1), probe.counts['odom'], probe.counts['cloud'],
                probe.counts['cloud_nonempty'], probe.counts['odom_invalid'],
                probe.max_gap['odom'], probe.max_gap['cloud'], rss_kib(args.pid),
                probe.sync.get('image_rate_hz', ''),
                probe.sync.get('sync_total_lidar', ''),
                probe.sync.get('sync_paired', ''),
                probe.sync.get('sync_image_delta_exceeded', ''),
                probe.sync.get('sync_missing_imu', ''),
                *position, probe.max_displacement_m, probe.max_step_m)

    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(probe, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_report:
                rows.append(snapshot(now - started))
                next_report += 30.0
            if driver_failed.is_set():
                abort_reason = 'Gazebo velocity command failed'
                break
            if probe.counts['odom_invalid']:
                abort_reason = 'non-finite odometry pose'
                break
            if args.max_odom_displacement_m > 0 and probe.max_displacement_m > args.max_odom_displacement_m:
                abort_reason = f'odometry displacement exceeded {args.max_odom_displacement_m} m'
                break
            if args.max_odom_step_m > 0 and probe.max_step_m > args.max_odom_step_m:
                abort_reason = f'adjacent odometry step exceeded {args.max_odom_step_m} m'
                break
            if args.max_rss_kib > 0 and rss_kib(args.pid) > args.max_rss_kib:
                abort_reason = f'core RSS exceeded {args.max_rss_kib} KiB'
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if driver:
            driver.join(timeout=12)
        probe.destroy_node()
        rclpy.try_shutdown()
    rows.append(snapshot(time.monotonic() - started))
    with open(args.csv_path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('seconds', 'odom', 'cloud', 'cloud_nonempty',
                         'odom_invalid', 'odom_max_gap_s', 'cloud_max_gap_s',
                         'core_rss_kib', 'adapter_rgb_hz', 'sync_lidar',
                         'sync_paired', 'sync_rgb_rejected', 'sync_imu_missing',
                         'odom_x', 'odom_y', 'odom_z',
                         'odom_max_displacement_m', 'odom_max_step_m'))
        writer.writerows(rows)
    print('counts:', dict(probe.counts))
    print('max_gap_s:', dict(probe.max_gap))
    print('final_sync:', probe.sync)
    print('rss_kib_range:', min((row[7] for row in rows), default=0),
          max((row[7] for row in rows), default=0))
    if abort_reason:
        print('aborted:', abort_reason)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
