#!/usr/bin/env python3
"""Preview guarded EGO-Planner samples without sending commands to PX4."""

import math
import signal
import time
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from px4_topics import px4_sensor_history_qos, px4_topic
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.interpolate import BSpline
from std_msgs.msg import Bool, Empty, String, Int64
from traj_utils.msg import Bspline as BsplineMsg


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def parse_spline(msg, max_speed, max_accel):
    degree = msg.order
    knots = np.asarray(msg.knots, dtype=np.float64)
    controls = np.asarray([(p.x, p.y, p.z) for p in msg.pos_pts], dtype=np.float64)
    if (degree != 3 or controls.ndim != 2 or len(controls) < degree + 1 or
            len(knots) != len(controls) + degree + 1 or
            not np.isfinite(knots).all() or not np.isfinite(controls).all() or
            np.any(np.diff(knots) <= 0)):
        raise ValueError('invalid cubic B-spline')
    start, end = knots[degree], knots[-degree - 1]
    if end <= start or stamp_ns(msg.start_time) <= 0:
        raise ValueError('invalid trajectory time')
    spline = BSpline(knots, controls, degree, extrapolate=False)
    times = np.linspace(start, end, max(2, math.ceil((end - start) / 0.02) + 1))
    speed = np.linalg.norm(spline.derivative(1)(times), axis=1)
    accel = np.linalg.norm(spline.derivative(2)(times), axis=1)
    if (not np.isfinite(speed).all() or not np.isfinite(accel).all() or
            np.max(speed) > max_speed or np.max(accel) > max_accel):
        raise ValueError('trajectory exceeds preview motion limits')
    return spline, start, end


class PreviewBspline(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('preview_bspline', parameter_overrides=parameter_overrides)
        self.control_clock = self.get_clock()
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('max_speed_mps', 0.5)
        self.declare_parameter('max_accel_mps2', 0.5)
        self.declare_parameter('max_odom_age_sec', 0.25)
        self.declare_parameter('max_ready_age_sec', 0.25)
        self.declare_parameter('max_px4_position_age_sec', 0.25)
        self.frame_id = self.get_parameter('frame_id').value
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_accel = float(self.get_parameter('max_accel_mps2').value)
        self.max_odom_age = float(self.get_parameter('max_odom_age_sec').value)
        self.max_ready_age = float(self.get_parameter('max_ready_age_sec').value)
        self.max_px4_age = float(self.get_parameter('max_px4_position_age_sec').value)
        if min(self.max_speed, self.max_accel, self.max_odom_age,
               self.max_ready_age, self.max_px4_age) <= 0:
            raise ValueError('preview limits and timeouts must be positive')

        self.ready = False
        self.valid = False
        self.ready_wall = None
        self.planner_wall = None
        self.odom_wall = None
        self.odom = None
        self.px4_wall = None
        self.px4_position = None
        self.hold_px4_position = None
        self.hold_pose = None
        self.spline = None
        self.trajectory_validator = None
        self.trajectory_start_ns = None
        self.trajectory_id = None
        self.completed_trajectory_id = None
        self.span = None
        self.mode = None
        self.reason = 'preview not initialized'
        self.sample_callback = None  # Optional serial handoff in a composed process.

        gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        px4_qos = px4_sensor_history_qos()
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready, gate_qos)
        self.create_subscription(Bool, '/planning/trajectory_valid', self.on_valid, gate_qos)
        self.create_subscription(Empty, '/planning/heartbeat', self.on_planner_heartbeat, 10)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, 10)
        self.create_subscription(BsplineMsg, '/planning/bspline', self.on_trajectory, 10)
        self.create_subscription(Int64, '/planning/trajectory_completed', self.on_completed, 10)
        self.create_subscription(VehicleLocalPosition, px4_topic(
            'out', 'vehicle_local_position', VehicleLocalPosition,
            self.get_parameter('px4_namespace').value),
                                 self.on_px4_position, px4_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/planning/position_preview', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/planning/velocity_preview', 10)
        self.hold_pub = self.create_publisher(TrajectorySetpoint,
                                              '/planning/px4_hold_preview', 10)
        self.mode_pub = self.create_publisher(String, '/planning/preview_mode', 10)
        self.reason_pub = self.create_publisher(String, '/planning/preview_reason', 10)
        self.output_timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()
        if not msg.data:
            self.spline = None

    def on_valid(self, msg):
        self.valid = msg.data
        if not msg.data:
            self.spline = None

    def on_planner_heartbeat(self, _msg):
        self.planner_wall = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if (msg.header.frame_id != self.frame_id or stamp_ns(msg.header.stamp) <= 0 or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z))):
            self.odom = None
            return
        self.odom = msg
        self.odom_wall = time.monotonic()

    def on_px4_position(self, msg):
        if (msg.xy_valid and msg.z_valid and msg.heading_good_for_control and
                all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.heading))):
            self.px4_position = (msg.x, msg.y, msg.z, msg.heading)
            self.px4_wall = time.monotonic()
        else:
            self.px4_position = None

    def on_trajectory(self, msg):
        try:
            spline, start, end = parse_spline(msg, self.max_speed, self.max_accel)
            if self.trajectory_validator is not None:
                self.trajectory_validator.validate_spline(spline)
        except ValueError as exc:
            self.spline = None
            self.get_logger().warning(str(exc))
            return
        self.spline = spline
        self.span = start, end
        self.trajectory_start_ns = stamp_ns(msg.start_time)
        self.trajectory_id = msg.traj_id

    def on_completed(self, msg):
        self.completed_trajectory_id = msg.data

    def fresh(self, last, max_age):
        return last is not None and time.monotonic() - last <= max_age

    def tick(self):
        now_ns = self.control_clock.now().nanoseconds
        odom_age_ns = (now_ns - stamp_ns(self.odom.header.stamp)
                       if self.odom is not None else -1)
        odom_ok = (self.odom is not None and
                   self.fresh(self.odom_wall, self.max_odom_age) and
                   0 <= odom_age_ns <= self.max_odom_age * 1e9)
        elapsed = ((now_ns - self.trajectory_start_ns) / 1e9
                   if self.trajectory_start_ns is not None else -1.0)
        tracking = (self.ready and self.valid and self.spline is not None and odom_ok and
                    self.fresh(self.ready_wall, self.max_ready_age) and
                    self.fresh(self.planner_wall, self.max_ready_age) and
                    0.0 <= elapsed <= self.span[1] - self.span[0])

        if tracking:
            t = self.span[0] + elapsed
            point = self.spline(t)
            velocity = self.spline.derivative(1)(t)
            if not np.isfinite(point).all() or not np.isfinite(velocity).all():
                tracking = False

        if tracking:
            self.hold_pose = None
            self.hold_px4_position = None
            pose = PoseStamped()
            pose.header.stamp = self.control_clock.now().to_msg()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
            pose.pose.orientation.w = 1.0
            twist = TwistStamped()
            twist.header = pose.header
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = map(float, velocity)
            self.pose_pub.publish(pose)
            self.velocity_pub.publish(twist)
            mode = 'TRACK_PREVIEW'
        else:
            mode = 'HOLD_PREVIEW' if odom_ok else 'NO_VALID_HOLD'
            if odom_ok:
                if self.hold_pose is None:
                    self.hold_pose = deepcopy(self.odom.pose.pose)
                pose = PoseStamped()
                pose.header.stamp = self.control_clock.now().to_msg()
                pose.header.frame_id = self.frame_id
                pose.pose = self.hold_pose
                self.pose_pub.publish(pose)
            if self.px4_position is not None and self.fresh(self.px4_wall, self.max_px4_age):
                if self.hold_px4_position is None:
                    self.hold_px4_position = self.px4_position
                hold = TrajectorySetpoint()
                hold.timestamp = now_ns // 1000
                hold.position = list(self.hold_px4_position[:3])
                hold.velocity = [math.nan] * 3
                hold.acceleration = [math.nan] * 3
                hold.jerk = [math.nan] * 3
                hold.yaw = self.hold_px4_position[3]
                hold.yawspeed = math.nan
                self.hold_pub.publish(hold)

        if mode != self.mode:
            self.get_logger().info('Preview mode: ' + mode)
            self.mode = mode
        self.mode_pub.publish(String(data=mode))
        reasons = []
        if not self.ready:
            reasons.append('map not ready')
        if not self.valid:
            reasons.append('trajectory invalid')
        if self.spline is None:
            reasons.append('no accepted spline')
        if not odom_ok:
            reasons.append('odometry stale or invalid')
        if not self.fresh(self.ready_wall, self.max_ready_age):
            reasons.append('map heartbeat stale')
        if not self.fresh(self.planner_wall, self.max_ready_age):
            reasons.append('planner heartbeat stale')
        if self.span is not None and not 0.0 <= elapsed <= self.span[1] - self.span[0]:
            reasons.append('outside trajectory time span')
        if not tracking and not reasons:
            reasons.append('non-finite trajectory sample')
        self.reason = 'tracking' if tracking else '; '.join(reasons)
        self.reason_pub.publish(String(data=self.reason))
        if self.sample_callback is not None:
            self.sample_callback(pose if tracking else None,
                                 twist if tracking else None, mode)


def main():
    rclpy.init()
    node = PreviewBspline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
