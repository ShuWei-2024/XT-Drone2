#!/usr/bin/env python3
"""Passive wall-clock freshness audit; publishes no control messages."""
import argparse
import csv
import json
import time
from collections import defaultdict
from pathlib import Path

import os
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_topic
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return dict(count=len(values), p50=values[int((len(values)-1)*.5)],
                p95=values[int((len(values)-1)*.95)], max=values[-1])


class Monitor(Node):
    def __init__(self, output):
        super().__init__('gpu_joint_monitor')
        self.output = output
        self.stream = (output / 'receipt.csv').open('w')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['kind', 'wall_ns', 'clock_ns', 'stamp_ns', 'x', 'y', 'z', 'value'])
        self.clock_ns = 0
        self.started = time.monotonic_ns()
        self.previous = {}
        self.gaps = defaultdict(list)
        self.ages = []
        self.clock_samples = []
        self.receipt_ages = defaultdict(list)
        self.ready_values = []
        self.reset_events = []
        self.last_reset = None
        self.last_ready = False
        self.resource_stream = (output / 'process_resources.csv').open('w')
        self.resource_writer = csv.writer(self.resource_stream)
        self.resource_writer.writerow(['wall_ns', 'clock_ns', 'role', 'pid', 'rss_kib'])
        self.resource_samples = defaultdict(list)
        self.resource_next = 0.
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, px4_topic('out', 'vehicle_local_position', VehicleLocalPosition, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')),
                                 self.on_local, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, px4_topic('out', 'vehicle_status', VehicleStatus, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')),
                                 self.on_status, qos_profile_sensor_data)
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def record(self, kind, stamp=0, xyz=('', '', ''), value=''):
        wall = time.monotonic_ns()
        if kind in self.previous:
            self.gaps[kind].append((wall-self.previous[kind])/1e6)
        self.previous[kind] = wall
        self.writer.writerow([kind, wall, self.clock_ns, stamp, *xyz, value])

    def on_clock(self, msg):
        self.clock_ns = msg.clock.sec*10**9 + msg.clock.nanosec
        if time.monotonic_ns()-self.started > 10*10**9:
            self.clock_samples.append((time.monotonic_ns(), self.clock_ns))

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        self.record('odom', stamp, (p.x, p.y, p.z))
        if self.clock_ns:
            self.ages.append((self.clock_ns-stamp)/1e6)

    def on_local(self, msg):
        resets = [msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter]
        if self.last_reset is not None and resets != self.last_reset:
            self.reset_events.append(dict(wall_ns=time.monotonic_ns(), before=self.last_reset, after=resets))
        self.last_reset = resets
        self.record('px4_local', msg.timestamp_sample*1000, (msg.x, msg.y, msg.z),
                    json.dumps(dict(xy_valid=msg.xy_valid, z_valid=msg.z_valid,
                                    heading_good=msg.heading_good_for_control, resets=resets)))

    def on_status(self, msg):
        self.record('px4_status', msg.timestamp*1000, value=json.dumps(
            dict(arming_state=msg.arming_state, nav_state=msg.nav_state, failsafe=msg.failsafe)))

    def on_ready(self, msg):
        self.last_ready = msg.data
        self.record('ready', value=int(msg.data))

    def sample_watchdog(self):
        now = time.monotonic_ns()
        if now / 1e9 >= self.resource_next:
            self.resource_next = now / 1e9 + 1.
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    args = (proc / 'cmdline').read_bytes().split(b'\0')
                    role = next((name for name, token in (
                        ('livo', b'fastlivo_mapping'),
                        ('local_map', b'local_mapping_guard.py'),
                        ('planner', b'ego_planner_node'),
                        ('frame_audit', b'audit_frame_alignment.py'),
                        ('external_vision', b'external_vision_sitl.py'),
                        ('health_audit', b'health_probe_117.py'),
                        # The flight driver is a separate Python child. Track
                        # it independently so the audit can distinguish its
                        # diagnostic-memory cost from LIVO/map/planner RSS.
                        ('motion', b'ego_sitl_flight.py'))
                        if any(Path(arg.decode(errors='replace')).name == token.decode()
                               for arg in args[:2] if arg)), None)
                    if role is None:
                        continue
                    status = (proc / 'status').read_text()
                    rss = next(int(line.split()[1]) for line in status.splitlines()
                               if line.startswith('VmRSS:'))
                except (OSError, StopIteration, ValueError):
                    continue
                self.resource_writer.writerow([now, self.clock_ns, role, proc.name, rss])
                self.resource_samples[role].append((now, rss))
        if now-self.started < 10*10**9:
            return
        for kind in ('odom', 'px4_local', 'px4_status', 'ready'):
            if kind in self.previous:
                self.receipt_ages[kind].append((now-self.previous[kind])/1e6)
        self.ready_values.append(self.last_ready)

    def finish(self):
        self.stream.close()
        self.resource_stream.close()
        rtf = None
        if len(self.clock_samples) > 1:
            first, last = self.clock_samples[0], self.clock_samples[-1]
            rtf = (last[1]-first[1])/(last[0]-first[0])
        result = dict(real_time_factor=rtf, gap_ms={k: stats(v) for k,v in self.gaps.items()},
                      odom_clock_age_ms=stats(self.ages),
                      watchdog_receipt_age_ms={k: stats(v) for k,v in self.receipt_ages.items()},
                      ready_fraction=sum(self.ready_values)/len(self.ready_values) if self.ready_values else None,
                      resets=self.reset_events,
                      rss_kib={role: dict(count=len(rows), first=rows[0][1], last=rows[-1][1],
                                         maximum=max(v for _, v in rows))
                               for role, rows in self.resource_samples.items()})
        (self.output/'receipt_summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--stop-file', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.output)
    deadline = time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if args.stop_file and args.stop_file.exists():
                break
            rclpy.spin_once(node, timeout_sec=.01)
            node.sample_watchdog()
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
