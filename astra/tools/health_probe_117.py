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


class Probe(Node):
    def __init__(self):
        super().__init__('astra_117_passive_health')
        self.rows = defaultdict(list)
        self.clock_ns = 0
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
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        for key, cls, topic in [('lidar', PointCloud2, '/sensors/lidar/points'),
                                ('camera', Image, '/sensors/camera/color/image_raw'),
                                ('imu', Imu, '/sensors/imu/data')]:
            self.create_subscription(cls, topic, lambda msg, k=key: self.record(k), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for key, cls, name in [('px4_local', VehicleLocalPosition, 'vehicle_local_position'),
                              ('px4_status', VehicleStatus, 'vehicle_status'),
                              ('gps', SensorGps, 'vehicle_gps_position'),
                              ('battery', BatteryStatus, 'battery_status'),
                              ('px4_imu', SensorCombined, 'sensor_combined')]:
            self.create_subscription(cls, px4_topic('out', name, cls, namespace),
                                     lambda msg, k=key: self.on_px4(k, msg), qos_profile_sensor_data)

    def record(self, key):
        self.rows[key].append(time.monotonic())

    def on_clock(self, msg):
        self.clock_ns = msg.clock.sec * 10**9 + msg.clock.nanosec

    def on_truth(self, msg):
        if any(p.name == 'astra_x500_px4' for p in msg.pose):
            with self.truth_lock:
                self.truth_times.append(msg.header.stamp.sec*10**9 + msg.header.stamp.nsec)

    def on_odom(self, msg):
        self.record('odom')
        if self.clock_ns:
            self.odom_ages.append((self.clock_ns-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e6)

    def on_ready(self, msg):
        self.record('ready')
        self.ready.append((time.monotonic(), bool(msg.data)))

    def on_px4(self, key, msg):
        self.record(key)
        if key == 'px4_local':
            self.local = msg
            if self.clock_ns:
                self.px4_ages.append((self.clock_ns-msg.timestamp_sample*1000)/1e6)
                self.px4_timing.append((self.clock_ns, msg.timestamp_sample*1000, msg.timestamp*1000))
        elif key == 'px4_status':
            self.status = msg

    def summary(self):
        now = time.monotonic()
        channels = {}
        for key, rows in self.rows.items():
            gaps = np.diff(rows)*1000
            channels[key] = dict(count=len(rows), rate_hz=(len(rows)-1)/(rows[-1]-rows[0]) if len(rows)>1 else 0.,
                last_age_ms=(now-rows[-1])*1000,
                max_gap_ms=float(gaps.max()) if len(gaps) else None)
        ready = [v for t,v in self.ready if t >= now-10.]
        ages = lambda a: ({'p95_ms':float(np.percentile(a,95)), 'max_ms':float(np.max(a)),
                          'min_ms':float(np.min(a))} if a else {})
        checks = {f'{key}_received': key in channels and channels[key]['count'] >= 10
                  for key in ('lidar','camera','imu','odom','ready','px4_local','px4_status','gps','battery','px4_imu')}
        checks['ready_last_10s'] = bool(ready) and np.mean(ready) >= .95
        checks['ready_currently_true'] = bool(self.ready) and self.ready[-1][1]
        checks['px4_position_valid'] = self.local is not None and self.local.xy_valid and self.local.z_valid
        checks['odom_measurement_age_p95_below_250ms'] = bool(self.odom_ages) and np.percentile(self.odom_ages,95) <= 250.
        # Receipt age includes computation and transport; it is not a clock offset.
        checks['px4_measurement_age_p95_below_250ms'] = bool(self.px4_ages) and np.percentile(self.px4_ages,95) <= 250.
        # /clock and PX4 callbacks can be delivered in different orders after a
        # scheduler pause. Compare timestamps within the same PX4 message instead.
        checks['px4_sample_not_after_its_publication'] = bool(self.px4_timing) and all(
            0 < sample <= pub for _,sample,pub in self.px4_timing)
        with self.truth_lock:
            truth = sorted(self.truth_times)
        pair_dt = []
        for _, sample, _ in self.px4_timing:
            i = bisect.bisect_left(truth, sample)
            candidates = truth[max(0,i-1):i+1]
            if candidates:
                pair_dt.append(min(abs(t-sample) for t in candidates)/1e6)
        pair_fraction = sum(dt <= 20. for dt in pair_dt)/len(self.px4_timing) if self.px4_timing else 0.
        checks['px4_truth_measurement_pairing_within_20ms_at_least_95pct'] = pair_fraction >= .95
        for key in ('odom','ready','px4_local'):
            checks[f'{key}_currently_fresh'] = key in channels and channels[key]['last_age_ms'] <= 250.
        return {'channels':channels, 'odom_measurement_age':ages(self.odom_ages),
                'px4_measurement_receipt_age':ages(self.px4_ages),
                'px4_sample_to_publish':ages([(pub-sample)/1e6 for _,sample,pub in self.px4_timing]),
                'px4_publish_to_last_received_clock':ages([(clock-pub)/1e6 for clock,_,pub in self.px4_timing]),
                'px4_truth_pair_dt':ages(pair_dt), 'px4_truth_pair_fraction_within_20ms':pair_fraction,
                'timing_note':'Receipt ages include processing, transport, and callback order. Nearest pairing is a coverage check, not independent clock-offset certification.',
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
    try:
        if a.healthy_timeout_sec is not None:
            result=wait_for_health(node,a.seconds,a.healthy_timeout_sec,
                                   lambda:rclpy.spin_once(node,timeout_sec=.005))
        else:
            while rclpy.ok() and time.monotonic()<deadline:
                if a.stop_file and a.stop_file.exists():break
                rclpy.spin_once(node,timeout_sec=.005)
            result=node.summary()
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
        a.output.with_suffix('.timing.json').write_text(json.dumps(node.px4_timing)+'\n')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
