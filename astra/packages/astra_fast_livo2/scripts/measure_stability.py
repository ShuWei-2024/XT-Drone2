#!/usr/bin/env python3
"""Record sensor delivery and timestamp quality without reading Gazebo truth."""

import argparse
import csv
import statistics
import time
from collections import defaultdict

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2


TOPICS = {
    'raw_lidar': ('/sensors/lidar/points', PointCloud2),
    'raw_rgb': ('/sensors/camera/color/image_raw', Image),
    'raw_imu': ('/sensors/imu/data', Imu),
    'input_lidar': ('/fast_livo2/input/lidar', PointCloud2),
    'input_rgb': ('/fast_livo2/input/image', Image),
    'input_imu': ('/fast_livo2/input/imu', Imu),
    'sync_lidar': ('/fast_livo2/sync/lidar', PointCloud2),
    'sync_rgb': ('/fast_livo2/sync/image', Image),
    'sync_imu': ('/fast_livo2/sync/imu', Imu),
}


class Probe(Node):
    def __init__(self, topics):
        super().__init__('fast_livo2_stability_probe')
        self.events = defaultdict(list)
        self.subs = []
        for name in topics:
            topic, msg_type = TOPICS[name]
            self.subs.append(self.create_subscription(
                msg_type, topic, lambda msg, key=name: self.record(key, msg),
                qos_profile_sensor_data))

    def record(self, name, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.events[name].append((stamp, time.monotonic_ns(), msg.header.frame_id))


def describe(values):
    if not values:
        return 'n=0'
    ordered = sorted(values)
    return (f'n={len(values)} mean={statistics.mean(values):.3f} '
            f'max={ordered[-1]:.3f} p95={ordered[int(0.95 * (len(ordered) - 1))]:.3f}')


def nearest_ms(source, target):
    times = sorted(entry[0] for entry in target)
    if not times:
        return []
    import bisect
    result = []
    for stamp, _, _ in source:
        i = bisect.bisect_left(times, stamp)
        result.append(min(abs(stamp - times[j]) for j in (i - 1, i) if 0 <= j < len(times)) / 1e6)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path')
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--topics', nargs='+', choices=tuple(TOPICS), default=list(TOPICS))
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.topics)
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()

    with open(args.csv_path, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('topic', 'seq', 'stamp_ns', 'receive_monotonic_ns', 'frame_id', 'stamp_step_ms', 'receive_step_ms'))
        for name, events in probe.events.items():
            for i, (stamp, received, frame) in enumerate(events):
                previous = events[i - 1] if i else None
                writer.writerow((name, i, stamp, received, frame,
                                 (stamp - previous[0]) / 1e6 if previous else '',
                                 (received - previous[1]) / 1e6 if previous else ''))

    for name in args.topics:
        events = probe.events[name]
        if len(events) < 2:
            print(f'{name}: n={len(events)}')
            continue
        span = (events[-1][1] - events[0][1]) / 1e9
        steps = [(events[i][0] - events[i - 1][0]) / 1e6 for i in range(1, len(events))]
        receive_steps = [(events[i][1] - events[i - 1][1]) / 1e6 for i in range(1, len(events))]
        bursts = sum(received < stamped * 0.5 for stamped, received in zip(steps, receive_steps))
        print(f'{name}: n={len(events)} wall_hz={(len(events)-1)/span:.2f} '
              f'stamp_step_ms {describe(steps)} receive_step_ms {describe(receive_steps)} '
              f'burst_after_backlog={bursts}')
    for lidar, rgb, imu in (('raw_lidar', 'raw_rgb', 'raw_imu'),
                            ('input_lidar', 'input_rgb', 'input_imu'),
                            ('sync_lidar', 'sync_rgb', 'sync_imu')):
        if not {lidar, rgb, imu}.issubset(args.topics):
            continue
        print(f'{lidar}/{rgb} nearest_ms {describe(nearest_ms(probe.events[lidar], probe.events[rgb]))}')
        print(f'{rgb}/{imu} nearest_ms {describe(nearest_ms(probe.events[rgb], probe.events[imu]))}')
        print(f'{lidar}/{imu} nearest_ms {describe(nearest_ms(probe.events[lidar], probe.events[imu]))}')


if __name__ == '__main__':
    main()
