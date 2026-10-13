#!/usr/bin/env python3
"""Record FAST-LIVO2 output continuity without using simulator truth."""

import argparse
import csv
import math
import os
import statistics
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__('fast_livo2_trajectory_probe')
        self.rows = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subs = [
            self.create_subscription(Odometry, '/fast_livo2/odom', self.odom, qos),
            self.create_subscription(Path, '/fast_livo2/path', self.path, qos),
            self.create_subscription(PoseStamped, '/fast_livo2/vision_pose', self.vision, qos),
            self.create_subscription(PointCloud2, '/cloud_registered', self.cloud, qos),
        ]

    def add(self, name, header, pose=None, detail=''):
        xyz = (pose.position.x, pose.position.y, pose.position.z) if pose else ('', '', '')
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w) if pose else ('', '', '', '')
        self.rows.append((name, stamp_ns(header), time.monotonic_ns(), header.frame_id,
                          *xyz, *quaternion, detail))

    def odom(self, msg):
        self.add('odom', msg.header, msg.pose.pose)

    def path(self, msg):
        self.add('path', msg.header, detail=str(len(msg.poses)))

    def vision(self, msg):
        self.add('vision_pose', msg.header, msg.pose)

    def cloud(self, msg):
        fields = ','.join(field.name for field in msg.fields)
        self.add('cloud_registered', msg.header, detail=f'{msg.width * msg.height}:{fields}')


def process_stats(pid):
    try:
        with open(f'/proc/{pid}/stat') as stream:
            stat = stream.read().rsplit(')', 1)[1].split()
        with open(f'/proc/{pid}/status') as stream:
            rss = next(int(line.split()[1]) for line in stream if line.startswith('VmRSS:'))
        return int(stat[11]) + int(stat[12]), rss
    except (OSError, StopIteration, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path')
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--pid', type=int)
    args = parser.parse_args()
    rclpy.init()
    probe = Probe()
    samples = []
    end = time.monotonic() + args.seconds
    next_sample = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if args.pid and time.monotonic() >= next_sample:
                sample = process_stats(args.pid)
                if sample:
                    samples.append((time.monotonic(), *sample))
                next_sample += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()

    with open(args.csv_path, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('topic', 'stamp_ns', 'receive_monotonic_ns', 'frame_id',
                         'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'detail'))
        writer.writerows(probe.rows)

    counts = Counter(row[0] for row in probe.rows)
    print('messages:', dict(counts))
    for topic in ('odom', 'vision_pose'):
        poses = [row for row in probe.rows if row[0] == topic]
        if not poses:
            continue
        invalid = sum(not all(math.isfinite(v) for v in row[4:11]) for row in poses)
        jumps = [math.dist(a[4:7], b[4:7]) for a, b in zip(poses, poses[1:])
                 if all(math.isfinite(v) for v in (*a[4:7], *b[4:7]))]
        elapsed = (poses[-1][2] - poses[0][2]) / 1e9 if len(poses) > 1 else 0
        print(f'{topic}: wall_hz={(len(poses)-1)/elapsed if elapsed else 0:.2f} '
              f'invalid={invalid} max_step_m={max(jumps, default=0):.4f} '
              f'mean_step_m={statistics.mean(jumps) if jumps else 0:.4f}')
    clouds = [row[11] for row in probe.rows if row[0] == 'cloud_registered']
    if clouds:
        print('cloud sample:', clouds[-1])
    if len(samples) > 1:
        clock_ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        cpu = [(b[1] - a[1]) / clock_ticks / (b[0] - a[0]) * 100
               for a, b in zip(samples, samples[1:])]
        print(f'process_cpu_percent mean={statistics.mean(cpu):.1f} max={max(cpu):.1f} '
              f'rss_kib_max={max(sample[2] for sample in samples)}')


if __name__ == '__main__':
    main()
