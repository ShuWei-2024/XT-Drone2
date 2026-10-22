"""Passive sensor and DDS audit; no PX4 input publishers."""
import argparse
import bisect
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import sys
import time
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, Imu
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus, SensorGps, SensorCombined
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_topic


def stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec)*10**9 + int(stamp.nanosec)


class Probe(Node):
    def __init__(self):
        super().__init__('astra_117_passive_health')
        self.rows = defaultdict(list)
        self.rows_lock = threading.Lock()
        self.callback_groups = {key: MutuallyExclusiveCallbackGroup() for key in (
            'clock', 'lidar', 'camera', 'imu', 'odom', 'ready', 'px4_local',
            'px4_status', 'gps', 'battery', 'px4_imu')}
        self.clock_ns = 0
        self.clock_receipt_ns = 0
        self.odom_ages = []
        self.px4_ages = []
        self.px4_timing = []
        self.truth_times = []
        self.truth_lock = threading.Lock()
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(Pose_V, '/world/astra_px4_open/dynamic_pose/info', self.on_truth)
        self.ready = []
        self.local = self.status = None
        namespace = os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data,
                                 callback_group=self.callback_groups['clock'])
        for key, cls, topic in [('lidar', PointCloud2, '/sensors/lidar/points'),
                                ('camera', Image, '/sensors/camera/color/image_raw'),
                                ('imu', Imu, '/sensors/imu/data')]:
            self.create_subscription(cls, topic, lambda msg, k=key: self.record(k, stamp_ns(msg)),
                qos_profile_sensor_data, callback_group=self.callback_groups[key])
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom,
                                 qos_profile_sensor_data, callback_group=self.callback_groups['odom'])
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.callback_groups['ready'])
        for key, cls, name in [('px4_local', VehicleLocalPosition, 'vehicle_local_position'),
                              ('px4_status', VehicleStatus, 'vehicle_status'),
                              ('gps', SensorGps, 'vehicle_gps_position'),
                              ('battery', BatteryStatus, 'battery_status'),
                              ('px4_imu', SensorCombined, 'sensor_combined')]:
            self.create_subscription(cls, px4_topic('out', name, cls, namespace),
                lambda msg, k=key: self.on_px4(k, msg), qos_profile_sensor_data,
                callback_group=self.callback_groups[key])

    def record(self, key, measurement_ns=None):
        receipt_ns = time.monotonic_ns()
        with self.rows_lock:
            self.rows[key].append((receipt_ns, measurement_ns))

    def on_clock(self, msg):
        with self.rows_lock:
            self.clock_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
            self.clock_receipt_ns = time.monotonic_ns()

    def latest_clock(self):
        with self.rows_lock:
            return self.clock_ns, self.clock_receipt_ns

    def on_truth(self, msg):
        if any(p.name == 'astra_x500_px4' for p in msg.pose):
            with self.truth_lock:
                self.truth_times.append(msg.header.stamp.sec*10**9 + msg.header.stamp.nsec)

    def on_odom(self, msg):
        sample_ns = stamp_ns(msg)
        self.record('odom', sample_ns)
        clock_ns, _ = self.latest_clock()
        if clock_ns:
            with self.rows_lock:
                self.odom_ages.append((clock_ns-sample_ns)/1e6)

    def on_ready(self, msg):
        self.record('ready')
        with self.rows_lock:
            self.ready.append((time.monotonic(), bool(msg.data)))

    def on_px4(self, key, msg):
        sample_us = getattr(msg, 'timestamp_sample', 0) or getattr(msg, 'timestamp', 0)
        self.record(key, int(sample_us)*1000 if sample_us else None)
        if key == 'px4_local':
            clock_ns, _ = self.latest_clock()
            with self.rows_lock:
                self.local = msg
                if clock_ns:
                    self.px4_ages.append((clock_ns-msg.timestamp_sample*1000)/1e6)
                    self.px4_timing.append((clock_ns, msg.timestamp_sample*1000, msg.timestamp*1000))
        elif key == 'px4_status':
            with self.rows_lock:
                self.status = msg

    @staticmethod
    def summarize_channel(rows, now_ns):
        rows = sorted(rows, key=lambda item: item[0])
        receipts = [item[0] for item in rows]
        receipt_gaps = np.diff(receipts)/1e6
        stamped = [(receipt, int(measurement)) for receipt, measurement in rows
                   if measurement is not None and int(measurement) > 0]
        measurement_gaps = np.diff([item[1] for item in stamped])/1e6
        max_gap = None
        if len(receipt_gaps):
            i = int(np.argmax(receipt_gaps))
            max_gap = dict(duration_ms=float(receipt_gaps[i]),
                           receipt_before_monotonic_ns=int(receipts[i]),
                           receipt_after_monotonic_ns=int(receipts[i+1]),
                           measurement_before_ns=(int(rows[i][1]) if rows[i][1] is not None else None),
                           measurement_after_ns=(int(rows[i+1][1]) if rows[i+1][1] is not None else None))
        return dict(count=len(rows),
            rate_hz=(len(rows)-1)/((receipts[-1]-receipts[0])/1e9) if len(rows)>1 and receipts[-1]>receipts[0] else 0.,
            last_age_ms=(now_ns-receipts[-1])/1e6 if rows else None,
            max_gap_ms=float(receipt_gaps.max()) if len(receipt_gaps) else None,
            max_gap_interval=max_gap,
            measurement_gap_ms=(dict(count=len(measurement_gaps),
                p50=float(np.percentile(measurement_gaps,50)),
                p95=float(np.percentile(measurement_gaps,95)),
                max=float(measurement_gaps.max()),
                nonpositive_count=int(np.count_nonzero(measurement_gaps<=0)))
                if len(measurement_gaps) else None))

    def summary(self):
        now_ns = time.monotonic_ns()
        now = now_ns/1e9
        channels = {}
        with self.rows_lock:
            rows_snapshot = {key: list(rows) for key, rows in self.rows.items()}
            ready_snapshot = list(self.ready)
            local, status = self.local, self.status
            odom_ages, px4_ages = list(self.odom_ages), list(self.px4_ages)
            px4_timing = list(self.px4_timing)
            clock_ns, clock_receipt_ns = self.clock_ns, self.clock_receipt_ns
        for key, rows in rows_snapshot.items():
            channels[key] = self.summarize_channel(rows, now_ns)
        ready = [v for t,v in ready_snapshot if t >= now-10.]
        ages = lambda a: ({'p95_ms':float(np.percentile(a,95)), 'max_ms':float(np.max(a)),
                          'min_ms':float(np.min(a))} if a else {})
        checks = {f'{key}_received': key in channels and channels[key]['count'] >= 10
                  for key in ('lidar','camera','imu','odom','ready','px4_local','px4_status','gps','battery','px4_imu')}
        checks['ready_last_10s'] = bool(ready) and np.mean(ready) >= .95
        checks['ready_currently_true'] = bool(ready_snapshot) and ready_snapshot[-1][1]
        checks['px4_position_valid'] = local is not None and local.xy_valid and local.z_valid
        checks['odom_measurement_age_p95_below_250ms'] = bool(odom_ages) and np.percentile(odom_ages,95) <= 250.
        # Receipt age includes computation and transport; it is not a clock offset.
        checks['px4_measurement_age_p95_below_250ms'] = bool(px4_ages) and np.percentile(px4_ages,95) <= 250.
        # /clock and PX4 callbacks can be delivered in different orders after a
        # scheduler pause. Compare timestamps within the same PX4 message instead.
        checks['px4_sample_not_after_its_publication'] = bool(px4_timing) and all(
            0 < sample <= pub for _,sample,pub in px4_timing)
        with self.truth_lock:
            truth = sorted(self.truth_times)
        pair_dt = []
        for _, sample, _ in px4_timing:
            i = bisect.bisect_left(truth, sample)
            candidates = truth[max(0,i-1):i+1]
            if candidates:
                pair_dt.append(min(abs(t-sample) for t in candidates)/1e6)
        pair_fraction = sum(dt <= 20. for dt in pair_dt)/len(px4_timing) if px4_timing else 0.
        checks['px4_truth_measurement_pairing_within_20ms_at_least_95pct'] = pair_fraction >= .95
        for key in ('odom','ready','px4_local'):
            checks[f'{key}_currently_fresh'] = key in channels and channels[key]['last_age_ms'] <= 250.
        return {'channels':channels, 'odom_measurement_age':ages(odom_ages),
                'odom_measurement_age_samples':ages(odom_ages),
                'odom_age_scope':'Age against most recently received /clock; callback-order upper proxy, not exact DDS latency',
                'px4_measurement_receipt_age':ages(px4_ages),
                'px4_sample_to_publish':ages([(pub-sample)/1e6 for _,sample,pub in px4_timing]),
                'px4_publish_to_last_received_clock':ages([(clock-pub)/1e6 for clock,_,pub in px4_timing]),
                'px4_truth_pair_dt':ages(pair_dt), 'px4_truth_pair_fraction_within_20ms':pair_fraction,
                'clock_callback_age_ms':(now_ns-clock_receipt_ns)/1e6 if clock_receipt_ns else None,
                'timing_note':'Channel max_gap_ms is this probe process callback cadence. measurement_gap_ms uses message timestamps; receipt ages include processing, transport, and callback order. Nearest pairing is a coverage check, not independent clock-offset certification.',
                'ready_last_10s_fraction':float(np.mean(ready)) if ready else None,
                'checks':{k:bool(v) for k,v in checks.items()}, 'passed':all(checks.values())}


def wait_for_health(probe, minimum_seconds, timeout_seconds, spin_once,
                    monotonic=time.monotonic, report=print):
    """Observe continuously; a failed snapshot never authorizes flight."""
    start = monotonic()
    next_check = start + minimum_seconds
    attempts = []
    while monotonic() < start + timeout_seconds:
        spin_once()
        now = monotonic()
        if now < next_check:
            continue
        result = probe.summary()
        failed = [key for key, passed in result['checks'].items() if not passed]
        attempts.append(dict(elapsed_s=now-start, failed_checks=failed,
                             ready_fraction=result['ready_last_10s_fraction']))
        report('Preflight health: ' + ('PASS' if result['passed'] else
               'waiting for ' + ', '.join(failed)), flush=True)
        if result['passed']:
            break
        next_check = now + 1.
    else:
        result = probe.summary()
    result['health_wait'] = dict(elapsed_s=monotonic()-start,
                                timeout_s=timeout_seconds, attempts=attempts)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=60.);p.add_argument('--stop-file',type=Path)
    p.add_argument('--healthy-timeout-sec',type=float,
                   help='Preflight only: recheck unchanged gates until healthy or timeout')
    a=p.parse_args()
    if a.healthy_timeout_sec is not None and (
            not math.isfinite(a.healthy_timeout_sec) or not 12. <= a.seconds <= a.healthy_timeout_sec <= 60.
            or a.stop_file is not None):
        p.error('Health wait requires 12 <= seconds <= timeout <= 60 and no stop file')
    rclpy.init();node=Probe();deadline=time.monotonic()+a.seconds
    executor=MultiThreadedExecutor(num_threads=max(2,min(4,os.cpu_count() or 2)))
    executor.add_node(node)
    try:
        if a.healthy_timeout_sec is not None:
            result=wait_for_health(node,a.seconds,a.healthy_timeout_sec,
                                   lambda:executor.spin_once(timeout_sec=.005))
        else:
            while rclpy.ok() and time.monotonic()<deadline:
                if a.stop_file and a.stop_file.exists():break
                executor.spin_once(timeout_sec=.005)
            result=node.summary()
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
        a.output.with_suffix('.timing.json').write_text(json.dumps(node.px4_timing)+'\n')
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
