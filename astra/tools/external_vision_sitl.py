#!/usr/bin/env python3
"""Opt-in isolated PX4 1.17 EV experiment. No arming or setpoint publishers.

Position only (EV_CTRL=3): odom currently has no validated velocity covariance.
Never extrapolate or retimestamp measurements to imitate a higher sensor rate.
"""
import argparse
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus, EstimatorStatusFlags
from external_vision_geometry import ENU_TO_NED, convert_pose, valid_measurement

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_sensor_history_qos, px4_topic


class ExternalVision(Node):
    def __init__(self, output):
        super().__init__('astra_external_vision_sitl', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.history = deque(maxlen=100); self.pairs = deque(maxlen=80)
        self.status = self.flags = None
        self.flags_wall = self.local_wall = self.ready_wall = self.last_publish_wall = 0.
        self.map_ready = False; self.translation = None; self.last_stamp = 0
        self.last_source = None; self.source_fault = ''; self.blocked = False
        self.pending = None; self.pending_wall = 0.
        self.published = 0; self.rejected = 0; self.flag_rows = []; self.reset_rows = []
        self.last_resets = None; self.good_flag_count = self.flag_count = 0
        self.health_events = []; self.last_health_reason = None
        self.stream = (output/'external_vision_sent.csv').open('w')
        self.csv = csv.writer(self.stream)
        self.csv.writerow(['timestamp_us', 'timestamp_sample_us', 'measurement_age_ms', 'x', 'y', 'z',
                           'qw', 'qx', 'qy', 'qz', 'position_sigma_m'])
        ns = 'x500_0'
        self.pub = self.create_publisher(VehicleOdometry,
            px4_topic('in', 'vehicle_visual_odometry', VehicleOdometry, ns), qos_profile_sensor_data)
        gate = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(Bool, '/astra/external_vision/ready', gate)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.odom, 10)
        self.create_subscription(Bool, '/local_mapping/ready', self.ready, gate)
        self.create_subscription(VehicleLocalPosition, px4_topic('out', 'vehicle_local_position', VehicleLocalPosition, ns),
                                 self.local, px4_sensor_history_qos())
        self.create_subscription(VehicleStatus, px4_topic('out', 'vehicle_status', VehicleStatus, ns),
                                 self.on_status, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatusFlags, px4_topic('out', 'estimator_status_flags', EstimatorStatusFlags, ns),
                                 self.on_flags, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1., self.save)

    def local(self, msg):
        self.history.append(msg); self.local_wall = time.monotonic()
        resets = [msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter]
        if self.last_resets is not None and resets != self.last_resets:
            self.reset_rows.append(dict(timestamp_sample_us=msg.timestamp_sample,
                before=self.last_resets, after=resets,
                armed=bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)))
        self.last_resets = resets

    def ready(self, msg): self.map_ready = msg.data; self.ready_wall = time.monotonic()
    def on_status(self, msg): self.status = msg
    def on_flags(self, msg):
        self.flags = msg; self.flags_wall = time.monotonic(); self.flag_count += 1
        good = msg.cs_ev_pos and msg.cs_ev_hgt
        self.good_flag_count += int(good)
        self.flag_rows.append(dict(timestamp_us=msg.timestamp, ev_pos=msg.cs_ev_pos, ev_hgt=msg.cs_ev_hgt,
                                   ev_yaw=msg.cs_ev_yaw, ev_vel=msg.cs_ev_vel,
                                   armed=bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)))

    def odom(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if 0 < stamp-now <= 20_000_000:
            self.pending, self.pending_wall = msg, time.monotonic()
            return
        if (msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'base_link'
                or not valid_measurement(stamp, now, self.last_stamp) or self.blocked):
            self.rejected += 1; return
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        position = np.array([p.x, p.y, p.z]); quat = [q.x, q.y, q.z, q.w]
        try: ned, orientation = convert_pose(position, quat, np.zeros(3))
        except ValueError: self.rejected += 1; return
        if self.last_source is not None:
            dt = (stamp-self.last_source[0])/1e9
            if dt <= 0: self.rejected += 1; return
            if np.linalg.norm(position-self.last_source[1]) > .12+.6*dt:
                self.blocked = True; self.source_fault = 'source pose jump; restart on ground required'; return
        self.last_source = (stamp, position.copy())
        wall = time.monotonic()
        if not self.map_ready or wall-self.ready_wall > .25 or wall-self.local_wall > .25: return
        if self.translation is None:
            if not self.history or not self.status or self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED: return
            local = min(self.history, key=lambda v: abs(v.timestamp_sample*1000-stamp))
            if not local.xy_valid or not local.z_valid or abs(local.timestamp_sample*1000-stamp) > 20_000_000: return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            error = math.atan2(math.sin(math.pi/2-local.heading-yaw), math.cos(math.pi/2-local.heading-yaw))
            if abs(error) > math.radians(5): self.pairs.clear(); return
            self.pairs.append((stamp, np.array([local.x, local.y, local.z])-ned))
            if len(self.pairs) < 25 or stamp-self.pairs[0][0] < 4_000_000_000: return
            values = np.array([v for _, v in self.pairs])
            translation = np.median(values, axis=0)
            if np.max(np.linalg.norm(values-translation, axis=1)) > .1:
                self.pairs.popleft(); return
            self.translation = translation
            (self.output/'external_vision_alignment.json').write_text(json.dumps(dict(
                translation_ned_m=translation.tolist(), paired_samples=len(values),
                frozen_measurement_us=stamp//1000, position_reference='base_link',
                rotation='audited ENU/NED and FLU/FRD; zero-yaw SITL only',
                ground_only=True, ground_truth_used=False,
                velocity_fusion=False, yaw_fusion=False, position_sigma_m=.1), indent=2)+'\n')
            self.get_logger().info('EV ground reference frozen; publishing measurement-stamped positions')
        ned += self.translation
        out = VehicleOdometry()
        out.timestamp = now//1000; out.timestamp_sample = stamp//1000
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = ned.astype(float).tolist(); out.q = orientation.astype(float).tolist()
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = out.angular_velocity = [math.nan]*3
        out.position_variance = [.01]*3
        out.orientation_variance = out.velocity_variance = [math.nan]*3
        out.reset_counter = 0; out.quality = 0
        self.pub.publish(out); self.last_stamp = stamp
        self.last_publish_wall = wall; self.published += 1
        self.csv.writerow([out.timestamp, out.timestamp_sample, (now-stamp)/1e6,
                           *out.position, *out.q, .1])

    def health_reason(self):
        wall = time.monotonic()
        if self.blocked: return self.source_fault
        if self.translation is None: return 'ground reference not frozen'
        if wall-self.last_publish_wall > .25: return 'EV measurement publication stale'
        # PX4 publishes control-status flags on change or about once per second.
        # The 1.5 s telemetry watchdog is separate from 250 ms source freshness.
        if wall-self.flags_wall > 1.5: return 'PX4 fusion status report stale'
        if not self.flags or not (self.flags.cs_ev_pos and self.flags.cs_ev_hgt):
            return 'PX4 EV position/height fusion inactive'
        return ''

    def healthy(self): return not self.health_reason()

    def tick(self):
        if self.pending is not None:
            msg = self.pending
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp <= self.get_clock().now().nanoseconds:
                self.pending = None; self.odom(msg)
            elif time.monotonic()-self.pending_wall > .02:
                self.pending = None; self.rejected += 1
        reason = self.health_reason()
        if reason != self.last_health_reason:
            self.last_health_reason = reason
            self.health_events.append(dict(wall_ns=time.monotonic_ns(),
                sim_ns=self.get_clock().now().nanoseconds, reason=reason or 'ready',
                publication_age_s=time.monotonic()-self.last_publish_wall,
                fusion_status_age_s=time.monotonic()-self.flags_wall))
            self.get_logger().info('EV gate: '+(reason or 'ready'))
        self.health_pub.publish(Bool(data=not reason))
    def save(self):
        self.stream.flush()
        armed = [r for r in self.flag_rows if r['armed']]
        result = dict(published_measurements=self.published, rejected_measurements=self.rejected,
            source_fault=self.source_fault, healthy=self.healthy(),
            health_events=self.health_events,
            fusion_flags_count=self.flag_count, intended_ev_pos_hgt_count=self.good_flag_count,
            armed_fusion_flags_count=len(armed),
            armed_ev_pos_hgt_fraction=sum(r['ev_pos'] and r['ev_hgt'] for r in armed)/len(armed) if armed else None,
            flags=self.flag_rows, resets=self.reset_rows,
            note='Fusion control flags need ULog aid-source timestamps/innovations for acceptance evidence')
        (self.output/'external_vision.json').write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (os.environ.get('ASTRA_117_ENV_LOADED') != '1' or os.environ.get('ROS_DOMAIN_ID') != '94'
            or os.environ.get('GZ_PARTITION') != 'astra_px4_117'):
        raise RuntimeError('EV publisher requires the isolated Astra 1.17 SITL runner')
    rclpy.init(); node = ExternalVision(args.output)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.save(); node.stream.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': main()
