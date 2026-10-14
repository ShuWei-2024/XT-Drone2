#!/usr/bin/env python3
"""Guarded SITL-only bridge from sampled planner previews to PX4 position mode."""

import math
import signal
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from px4_topics import px4_topic
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class GuardedBsplineBridge(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('guarded_bspline_bridge', parameter_overrides=parameter_overrides)
        defaults = {
            'enable_px4_output': False,
            'px4_namespace': '',
            'sitl_only': True,
            'velocity_feedforward': False,
            'freeze_tracking_yaw': False,
            'frame_id': 'camera_init',
            'xy_matrix': [-0.0233561327, 0.9997272083,
                          0.9997272083, 0.0233561327],
            'xy_translation_m': [0.0744246373, 0.1474840015],
            'z_translation_m': -0.0094483472,
            'max_estimator_xy_disagreement_m': 0.30,
            'max_estimator_z_disagreement_m': 0.20,
            'max_estimator_yaw_disagreement_rad': 0.35,
            'max_tracking_error_m': 0.30,
            'max_speed_mps': 0.5,
            'max_preview_age_sec': 0.20,
            'max_odom_age_sec': 0.25,
            'max_px4_age_sec': 0.25,
            'max_status_age_sec': 1.0,
            'max_ready_age_sec': 0.25,
            'max_planner_age_sec': 0.25,
            'max_time_pair_sec': 0.02,
            'max_hold_sec': 1.0,
            'max_replan_gap_sec': 0.40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enabled = bool(self.get_parameter('enable_px4_output').value)
        self.px4_namespace = str(self.get_parameter('px4_namespace').value)
        self.sitl_only = bool(self.get_parameter('sitl_only').value)
        self.velocity_feedforward = bool(self.get_parameter('velocity_feedforward').value)
        self.freeze_tracking_yaw = bool(self.get_parameter('freeze_tracking_yaw').value)
        if self.enabled and not self.sitl_only:
            raise ValueError('PX4 output is restricted to isolated SITL')
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.matrix = np.asarray(self.get_parameter('xy_matrix').value,
                                 dtype=float).reshape(2, 2)
        self.translation = np.asarray(self.get_parameter('xy_translation_m').value,
                                      dtype=float)
        self.z_translation = float(self.get_parameter('z_translation_m').value)
        if (self.translation.shape != (2,) or
                not np.isfinite(self.matrix).all() or
                not np.isfinite(self.translation).all() or
                not np.allclose(self.matrix.T @ self.matrix, np.eye(2), atol=0.01) or
                abs(np.linalg.det(self.matrix) + 1.0) > 0.01):
            raise ValueError('camera_init to NED horizontal transform must be orthogonal/reflected')
        self.limits = {name: float(self.get_parameter(name).value) for name in defaults
                       if name.startswith('max_')}
        if any(not math.isfinite(value) or value <= 0 for value in self.limits.values()):
            raise ValueError('all bridge limits must be positive and finite')

        self.pose = self.velocity = self.odom = self.px4 = self.status = None
        self.pose_wall = self.velocity_wall = self.odom_wall = None
        self.px4_wall = self.status_wall = self.mode_wall = None
        self.preview_mode = None
        self.mode = 'WAITING'
        self.tracked = False
        self.offboard_seen = False
        self.fault_since = None
        self.frozen_hold = None
        self.last_land_request = None
        self.fault_reason = ''
        self.preview_gap_since = None
        self.replan_hold = None
        self.tracking_yaw = None
        self.planner_heartbeat_wall = None
        self.ready = False
        self.ready_wall = None
        self.reset_counters = None
        self.px4_history = deque(maxlen=100)
        self.odom_history = deque(maxlen=10)
        self.estimator_pair_age_sec = self.estimator_pair_dt_sec = None
        self.pending_poses = {}
        self.pending_velocities = {}
        self.pose_frame_valid = self.velocity_frame_valid = True

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, '/planning/position_preview',
                                 self.on_pose, 10)
        self.create_subscription(TwistStamped, '/planning/velocity_preview',
                                 self.on_velocity, 10)
        self.create_subscription(String, '/planning/preview_mode', self.on_mode, 10)
        self.create_subscription(Empty, '/planning/heartbeat', self.on_planner_heartbeat, 10)
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, 10)
        self.create_subscription(VehicleLocalPosition, px4_topic(
            'out', 'vehicle_local_position', VehicleLocalPosition, self.px4_namespace),
                                 self.on_px4, sensor_qos)
        self.create_subscription(VehicleStatus, px4_topic(
            'out', 'vehicle_status', VehicleStatus, self.px4_namespace),
                                 self.on_status, sensor_qos)
        self.preview_pub = self.create_publisher(TrajectorySetpoint,
                                                 '/planning/px4_setpoint_preview', 10)
        self.state_pub = self.create_publisher(String, '/planning/bridge_state', 10)
        if self.enabled:
            self.mode_pub = self.create_publisher(OffboardControlMode,
                px4_topic('in', 'offboard_control_mode', OffboardControlMode, self.px4_namespace), 10)
            self.setpoint_pub = self.create_publisher(TrajectorySetpoint,
                px4_topic('in', 'trajectory_setpoint', TrajectorySetpoint, self.px4_namespace), 10)
            self.command_pub = self.create_publisher(VehicleCommand,
                px4_topic('in', 'vehicle_command', VehicleCommand, self.px4_namespace), 10)
        self.output_timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('PX4 output ' + ('ENABLED for SITL' if self.enabled else 'disabled'))

    def on_pose(self, msg):
        self.pose_frame_valid = msg.header.frame_id == self.frame_id
        self.pending_poses[stamp_ns(msg.header.stamp)] = (msg, time.monotonic())
        self.pair_preview()

    def on_velocity(self, msg):
        self.velocity_frame_valid = msg.header.frame_id == self.frame_id
        self.pending_velocities[stamp_ns(msg.header.stamp)] = (msg, time.monotonic())
        self.pair_preview()

    def pair_preview(self):
        # DDS callbacks arrive separately. Commit only complete samples and retain
        # the preceding pair with its original timestamps and receipt age.
        common = self.pending_poses.keys() & self.pending_velocities.keys()
        if common:
            stamp = max(common)
            if self.pose is None or stamp >= stamp_ns(self.pose.header.stamp):
                self.pose, self.pose_wall = self.pending_poses[stamp]
                self.velocity, self.velocity_wall = self.pending_velocities[stamp]
            for cache in (self.pending_poses, self.pending_velocities):
                for key in list(cache):
                    if key <= stamp:
                        del cache[key]
        for cache in (self.pending_poses, self.pending_velocities):
            while len(cache) > 8:
                del cache[min(cache)]

    def on_mode(self, msg):
        self.preview_mode, self.mode_wall = msg.data, time.monotonic()

    def on_planner_heartbeat(self, _msg):
        self.planner_heartbeat_wall = time.monotonic()

    def on_ready(self, msg):
        self.ready, self.ready_wall = msg.data, time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_wall = msg, time.monotonic()
        self.odom_history.append((msg,self.odom_wall))

    def on_status(self, msg):
        self.status, self.status_wall = msg, time.monotonic()
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard_seen = True

    def on_px4(self, msg):
        self.px4, self.px4_wall = msg, time.monotonic()
        self.px4_history.append(msg)

    def fresh(self, seen, limit):
        return seen is not None and time.monotonic() - seen <= limit

    def transform(self, position):
        xy = self.matrix @ np.asarray(position[:2], dtype=float) + self.translation
        return np.array((xy[0], xy[1], -position[2] + self.z_translation))

    def source_yaw(self, odom=None):
        q = (self.odom if odom is None else odom).pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        forward = self.matrix @ np.array((math.cos(yaw), math.sin(yaw)))
        return math.atan2(forward[1], forward[0])

    def state_health(self):
        # These checks also apply while a replacement trajectory is pending.
        if not self.fresh(self.odom_wall, self.limits['max_odom_age_sec']):
            return 'stale FAST-LIVO2 odometry'
        if not self.fresh(self.px4_wall, self.limits['max_px4_age_sec']):
            return 'stale PX4 position'
        if not self.fresh(self.status_wall, self.limits['max_status_age_sec']):
            return 'stale PX4 status'
        if not self.ready:
            return 'local map not ready'
        if not self.fresh(self.ready_wall, self.limits['max_ready_age_sec']):
            return 'stale local map heartbeat'
        if not self.fresh(self.planner_heartbeat_wall, self.limits['max_planner_age_sec']):
            return 'stale planner heartbeat'
        if (self.offboard_seen and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            return 'PX4 left Offboard'
        if self.odom.header.frame_id != self.frame_id:
            return 'odometry frame mismatch'
        now_ns = self.get_clock().now().nanoseconds
        if not 0 <= now_ns - stamp_ns(self.odom.header.stamp) <= self.limits['max_odom_age_sec'] * 1e9:
            return 'stale measurement timestamp'
        if not self.px4.xy_valid or not self.px4.z_valid or not self.px4.heading_good_for_control:
            return 'invalid PX4 estimate'
        if not self.px4_history:
            return 'missing PX4 measurement history'
        reset = (self.px4.xy_reset_counter, self.px4.z_reset_counter,
                 self.px4.heading_reset_counter)
        if self.reset_counters is not None and reset != self.reset_counters:
            return 'PX4 estimator reset'
        self.reset_counters = reset
        latest = self.odom.pose.pose.position
        if not np.isfinite((latest.x,latest.y,latest.z)).all():
            return 'non-finite estimator position'
        if not math.isfinite(self.source_yaw()):
            return 'non-finite estimator heading'
        compatible = [row for row in self.px4_history if
            (row.xy_reset_counter,row.z_reset_counter,row.heading_reset_counter)==reset]
        pair = None
        for source, received in reversed(self.odom_history):
            epoch=stamp_ns(source.header.stamp)
            if (source.header.frame_id!=self.frame_id or
                    not self.fresh(received,self.limits['max_odom_age_sec']) or
                    not 0<=now_ns-epoch<=self.limits['max_odom_age_sec']*1e9 or not compatible):
                continue
            candidate=min(compatible,key=lambda row:abs(epoch-row.timestamp_sample*1000))
            if (candidate.xy_valid and candidate.z_valid and candidate.heading_good_for_control and
                    0<=now_ns-candidate.timestamp_sample*1000<=self.limits['max_px4_age_sec']*1e9 and
                    abs(epoch-candidate.timestamp_sample*1000)<=self.limits['max_time_pair_sec']*1e9):
                pair=(source,candidate);break
        if pair is None:
            return 'estimator measurement times differ'
        source, matched=pair
        self.estimator_pair_age_sec=(now_ns-stamp_ns(source.header.stamp))/1e9
        self.estimator_pair_dt_sec=abs(stamp_ns(source.header.stamp)-matched.timestamp_sample*1000)/1e9
        p = source.pose.pose.position
        odom_ned = self.transform((p.x, p.y, p.z))
        matched_ned = np.array((matched.x, matched.y, matched.z))
        px4_ned = np.array((self.px4.x, self.px4.y, self.px4.z))
        if (not np.isfinite(odom_ned).all() or not np.isfinite(matched_ned).all() or
                not np.isfinite(px4_ned).all()):
            return 'non-finite estimator position'
        delta = matched_ned - odom_ned
        xy_disagreement = float(np.linalg.norm(delta[:2]))
        z_disagreement = float(abs(delta[2]))
        yaw_disagreement = float(abs(wrap(matched.heading - self.source_yaw(source))))
        if not math.isfinite(yaw_disagreement):
            return 'non-finite estimator heading'
        if (xy_disagreement > self.limits['max_estimator_xy_disagreement_m'] or
                z_disagreement > self.limits['max_estimator_z_disagreement_m'] or
                yaw_disagreement > self.limits['max_estimator_yaw_disagreement_rad']):
            return (f'FAST-LIVO2 and PX4 disagree (xy={xy_disagreement:.3f}m '
                    f'z={z_disagreement:.3f}m yaw={math.degrees(yaw_disagreement):.1f}deg)')
        return ''

    def health(self):
        fault = self.state_health()
        if fault:
            return fault
        if self.preview_mode != 'TRACK_PREVIEW' or not self.fresh(
                self.mode_wall, self.limits['max_preview_age_sec']):
            return 'preview not tracking'
        if not all(self.fresh(seen, self.limits['max_preview_age_sec'])
                   for seen in (self.pose_wall, self.velocity_wall)):
            return 'stale trajectory preview'
        if (not self.pose_frame_valid or not self.velocity_frame_valid or
                self.pose.header.frame_id != self.frame_id or
                self.velocity.header.frame_id != self.frame_id or
                stamp_ns(self.pose.header.stamp) != stamp_ns(self.velocity.header.stamp)):
            return 'preview frame or stamp mismatch'
        if not 0 <= self.get_clock().now().nanoseconds - stamp_ns(self.pose.header.stamp) <= self.limits['max_odom_age_sec'] * 1e9:
            return 'stale measurement timestamp'
        px4_ned = np.array((self.px4.x, self.px4.y, self.px4.z))
        q = self.pose.pose.position
        target = self.transform((q.x, q.y, q.z))
        v = self.velocity.twist.linear
        velocity = self.matrix @ np.array((v.x, v.y))
        velocity = np.array((velocity[0], velocity[1], -v.z))
        if (not np.isfinite(target).all() or not np.isfinite(velocity).all() or
                np.linalg.norm(velocity) > self.limits['max_speed_mps'] or
                np.linalg.norm(target - px4_ned) > self.limits['max_tracking_error_m']):
            return 'trajectory speed or tracking error exceeded'
        return ''

    def make_setpoint(self, position, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = list(map(float, position))
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        return msg

    def send_land(self):
        now = time.monotonic()
        if (not self.enabled or self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or
                (self.last_land_request is not None and now - self.last_land_request < 0.5)):
            return
        command = VehicleCommand()
        command.timestamp = self.get_clock().now().nanoseconds // 1000
        command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        command.target_system = command.source_system = 1
        command.target_component = command.source_component = 1
        command.from_external = True
        self.command_pub.publish(command)
        self.last_land_request = now

    def tick(self):
        health_fault = self.health() if self.fault_since is None else self.fault_reason
        waiting_replan = False
        # EGO-Planner briefly invalidates the old spline while publishing a
        # replacement. Keep the current PX4 position during that bounded gap;
        # a persistent gap still enters the normal land-protection path.
        if (self.fault_since is None and self.tracked and
                health_fault == 'preview not tracking' and
                self.fresh(self.mode_wall, self.limits['max_preview_age_sec'])):
            self.preview_gap_since = self.preview_gap_since or time.monotonic()
            if time.monotonic() - self.preview_gap_since <= self.limits['max_replan_gap_sec']:
                health_fault = ''
                waiting_replan = True
                self.mode = 'WAITING_REPLAN'
                if self.px4 is not None and self.fresh(self.px4_wall, self.limits['max_px4_age_sec']):
                    if self.replan_hold is None:
                        yaw = (self.tracking_yaw if self.freeze_tracking_yaw and
                               self.tracking_yaw is not None else self.px4.heading)
                        self.replan_hold = (self.px4.x, self.px4.y, self.px4.z, yaw)
                    hold = self.make_setpoint(self.replan_hold[:3], self.replan_hold[3])
                    self.preview_pub.publish(hold)
                    if self.enabled:
                        self.setpoint_pub.publish(hold)
        elif health_fault != 'preview not tracking':
            self.preview_gap_since = None
            self.replan_hold = None
        fault = health_fault
        if not fault and not waiting_replan:
            q = self.pose.pose.position
            target = self.transform((q.x, q.y, q.z))
            if self.tracking_yaw is None:
                self.tracking_yaw = float(self.px4.heading)
            yaw = self.tracking_yaw if self.freeze_tracking_yaw else self.px4.heading
            setpoint = self.make_setpoint(target, yaw)
            if self.velocity_feedforward:
                v = self.velocity.twist.linear
                xy = self.matrix @ np.array((v.x, v.y))
                setpoint.velocity = [float(xy[0]), float(xy[1]), float(-v.z)]
            self.preview_pub.publish(setpoint)
            self.mode = 'TRACK_DRY_RUN' if not self.enabled else 'TRACK_SITL'
            if self.enabled:
                self.tracked = True
                self.setpoint_pub.publish(setpoint)
        elif not waiting_replan:
            if self.tracked and self.fault_since is None:
                self.fault_since = time.monotonic()
                self.fault_reason = fault
                if (self.px4 is not None and
                        self.fresh(self.px4_wall, self.limits['max_px4_age_sec']) and
                        self.px4.xy_valid and self.px4.z_valid and
                        self.px4.heading_good_for_control and
                        np.isfinite((self.px4.x, self.px4.y, self.px4.z,
                                     self.px4.heading)).all()):
                    self.frozen_hold = (self.px4.x, self.px4.y, self.px4.z,
                                        self.px4.heading)
            if self.enabled and self.tracked and self.frozen_hold is not None:
                hold = self.make_setpoint(self.frozen_hold[:3], self.frozen_hold[3])
                self.setpoint_pub.publish(hold)
                self.preview_pub.publish(hold)
            self.mode = ('FAULT_LATCHED: ' + self.fault_reason if self.fault_since is not None
                         else 'WAITING: ' + fault)
            if (self.enabled and self.tracked and self.fault_since is not None and
                    (self.frozen_hold is None or
                     time.monotonic() - self.fault_since >= self.limits['max_hold_sec']) and
                    self.status is not None and
                    self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.send_land()
        if (self.enabled and self.tracked and
                (self.last_land_request is None or
                 (self.status is not None and
                  self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD))):
            mode = OffboardControlMode()
            mode.timestamp = self.get_clock().now().nanoseconds // 1000
            mode.position = True
            self.mode_pub.publish(mode)
        self.state_pub.publish(String(data=self.mode))


def main():
    rclpy.init()
    node = GuardedBsplineBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
