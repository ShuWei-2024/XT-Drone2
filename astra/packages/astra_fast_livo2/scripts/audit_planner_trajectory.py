#!/usr/bin/env python3
"""Sample EGO-Planner B-splines against the guarded map and motion limits."""

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from traj_utils.msg import Bspline as BsplineMsg


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def inspect_trajectory(msg, cloud, limits):
    knots = np.asarray(msg.knots, dtype=np.float64)
    controls = np.asarray([(p.x, p.y, p.z) for p in msg.pos_pts], dtype=np.float64)
    order = msg.order
    if (order < 1 or len(controls) < order + 1 or
            len(knots) != len(controls) + order + 1 or
            not np.isfinite(knots).all() or not np.isfinite(controls).all() or
            np.any(np.diff(knots) < 0)):
        raise ValueError('invalid B-spline control points or knot vector')
    start, end = knots[order], knots[-order - 1]
    if end <= start:
        raise ValueError('non-positive B-spline duration')
    times = np.linspace(start, end, max(2, math.ceil((end - start) / 0.01) + 1))
    spline = BSpline(knots, controls, order, extrapolate=False)
    position = spline(times)
    velocity = spline.derivative(1)(times)
    acceleration = spline.derivative(2)(times)
    if not all(np.isfinite(a).all() for a in (position, velocity, acceleration)):
        raise ValueError('non-finite trajectory sample')
    if cloud is None or cloud['frame'] != 'camera_init':
        raise ValueError('missing camera_init obstacle cloud')
    if abs(stamp_ns(msg.start_time) - cloud['stamp_ns']) > 1_000_000_000:
        raise ValueError('obstacle cloud is more than 1 s from trajectory start')
    distances, closest_indices = cKDTree(cloud['points']).query(position, workers=-1)
    closest_sample = int(np.argmin(distances))
    result = {
        'traj_id': msg.traj_id,
        'start_ns': stamp_ns(msg.start_time),
        'duration_s': float(end - start),
        'cloud_points': len(cloud['points']),
        'cloud_age_s': (stamp_ns(msg.start_time) - cloud['stamp_ns']) / 1e9,
        'min_inflated_cloud_distance_m': float(np.min(distances)),
        'closest_trajectory_point': position[closest_sample].tolist(),
        'closest_obstacle_point': cloud['points'][closest_indices[closest_sample]].tolist(),
        'max_speed_mps': float(np.max(np.linalg.norm(velocity, axis=1))),
        'max_accel_mps2': float(np.max(np.linalg.norm(acceleration, axis=1))),
        'start_position': position[0].tolist(),
        'end_position': position[-1].tolist(),
    }
    result['pass'] = (result['min_inflated_cloud_distance_m'] >= limits.margin and
                      result['max_speed_mps'] <= limits.max_speed * 1.05 and
                      result['max_accel_mps2'] <= limits.max_accel * 1.05)
    return result, spline, (start, end)


class PlannerAudit(Node):
    def __init__(self, limits):
        super().__init__('planner_trajectory_audit')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.limits = limits
        self.ready = False
        self.ready_transitions = []
        self.valid = False
        self.odom = None
        self.cloud = None
        self.results = []
        self.errors = []
        self.valid_transitions = []
        self.continuity = []
        self.previous = None
        gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready, gate_qos)
        self.create_subscription(Bool, '/planning/trajectory_valid', self.on_valid, gate_qos)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/local_mapping/obstacles', self.on_cloud, 10)
        self.create_subscription(BsplineMsg, '/planning/bspline', self.on_trajectory, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

    def on_ready(self, msg):
        if msg.data != self.ready:
            self.ready_transitions.append((self.get_clock().now().nanoseconds, msg.data))
        self.ready = msg.data

    def on_valid(self, msg):
        self.valid = msg.data
        self.valid_transitions.append((self.get_clock().now().nanoseconds, msg.data))

    def on_odom(self, msg):
        self.odom = msg

    def on_cloud(self, msg):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        points = np.asarray(points, dtype=np.float64)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points):
            self.cloud = {'points': points, 'frame': msg.header.frame_id,
                          'stamp_ns': stamp_ns(msg.header.stamp)}

    def on_trajectory(self, msg):
        try:
            result, spline, span = inspect_trajectory(msg, self.cloud, self.limits)
            if self.previous is not None:
                prev_spline, prev_span, prev_start_ns = self.previous
                at = (result['start_ns'] - prev_start_ns) / 1e9
                old_time = prev_span[0] + at
                if prev_span[0] <= old_time <= prev_span[1]:
                    old = prev_spline(old_time)
                    new = spline(span[0])
                    self.continuity.append({
                        'position_gap_m': float(np.linalg.norm(old - new)),
                        'velocity_gap_mps': float(np.linalg.norm(
                            prev_spline.derivative(1)(old_time) - spline.derivative(1)(span[0]))),
                    })
            self.previous = spline, span, result['start_ns']
            self.results.append(result)
        except ValueError as exc:
            self.errors.append(str(exc))

    def send_relative_goal(self, dx, dy, dz):
        if self.odom is None or self.odom.header.frame_id != 'camera_init':
            raise ValueError('missing camera_init odometry')
        goal = PoseStamped()
        goal.header.frame_id = 'camera_init'
        goal.header.stamp = self.get_clock().now().to_msg()
        current = self.odom.pose.pose.position
        goal.pose.position.x = current.x + dx
        goal.pose.position.y = current.y + dy
        goal.pose.position.z = current.z + dz
        goal.pose.orientation.w = 1.0
        self.goal_msg = goal
        self.goal_pub.publish(goal)
        return (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-dx', type=float, default=0.5)
    parser.add_argument('--goal-dy', type=float, default=0.0)
    parser.add_argument('--goal-dz', type=float, default=0.0)
    parser.add_argument('--max-speed', type=float, default=0.5)
    parser.add_argument('--max-accel', type=float, default=0.5)
    parser.add_argument('--margin', type=float, default=0.15)
    parser.add_argument('--max-revision-position-gap', type=float, default=0.05)
    parser.add_argument('--max-revision-velocity-gap', type=float, default=0.1)
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--output')
    args = parser.parse_args()
    rclpy.init()
    node = PlannerAudit(args)
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not (node.ready and node.cloud and node.odom):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not (node.ready and node.cloud and node.odom):
            raise RuntimeError('guarded map did not become ready')
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and node.goal_pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.goal_pub.get_subscription_count() == 0:
            raise RuntimeError('planner goal subscriber was not discovered')
        goal = node.send_relative_goal(args.goal_dx, args.goal_dy, args.goal_dz)
        deadline = time.monotonic() + args.seconds
        retry_at = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.results and not node.errors and node.ready and time.monotonic() >= retry_at:
                node.goal_pub.publish(node.goal_msg)
                retry_at = time.monotonic() + 0.5
        continuity_ok = all(
            gap['position_gap_m'] <= args.max_revision_position_gap and
            gap['velocity_gap_mps'] <= args.max_revision_velocity_gap
            for gap in node.continuity)
        summary = {'goal': goal, 'ready_end': node.ready, 'valid_end': node.valid,
                   'trajectories': node.results, 'errors': node.errors,
                   'revision_gaps': node.continuity,
                   'ready_transitions': node.ready_transitions,
                   'valid_transitions': node.valid_transitions,
                   'continuity_pass': continuity_ok,
                   'pass': bool(node.results) and not node.errors and continuity_ok and all(
                       result['pass'] for result in node.results)}
        encoded = json.dumps(summary, indent=2)
        print(encoded)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as output:
                output.write(encoded + '\n')
        if not summary['pass']:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
