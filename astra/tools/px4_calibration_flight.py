#!/usr/bin/env python3
"""One-off bounded PX4 SITL motion for frame calibration; no planner input."""

from pathlib import Path
import math
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
import os
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_sensor_history_qos, px4_topic
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class CalibrationFlight(Node):
    def __init__(self):
        super().__init__('px4_calibration_flight')
        self.local = None
        self.local_wall = 0.0
        self.status = None
        self.status_wall = 0.0
        self.truth = None
        self.truth_wall = 0.0
        self.gz = GazeboNode()
        self.gz.subscribe(Pose_V, '/world/astra_px4_open/dynamic_pose/info', self.on_truth)
        self.create_subscription(VehicleLocalPosition, px4_topic('out', 'vehicle_local_position', VehicleLocalPosition, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')),
                                 self.on_local, px4_sensor_history_qos())
        self.create_subscription(VehicleStatus, px4_topic('out', 'vehicle_status', VehicleStatus, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')),
                                 self.on_status, qos_profile_sensor_data)
        self.mode_pub = self.create_publisher(OffboardControlMode,
                                               px4_topic('in', 'offboard_control_mode', OffboardControlMode, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')), 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint,
                                                   px4_topic('in', 'trajectory_setpoint', TrajectorySetpoint, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')), 10)
        self.command_pub = self.create_publisher(VehicleCommand,
                                                  px4_topic('in', 'vehicle_command', VehicleCommand, os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')), 10)

    def on_local(self, msg):
        self.local = msg
        self.local_wall = time.monotonic()

    def on_status(self, msg):
        self.status = msg
        self.status_wall = time.monotonic()

    def on_truth(self, msg):
        pose = next((item for item in msg.pose if item.name == 'astra_x500_px4'), None)
        if pose:
            p = pose.position
            self.truth = (p.x, p.y, p.z)
            self.truth_wall = time.monotonic()

    def fresh(self):
        now = time.monotonic()
        checks = {
            'PX4 local missing': self.local is not None,
            'PX4 status missing': self.status is not None,
            'Gazebo truth missing': self.truth is not None,
            'PX4 local receipt age >= 0.3 s': now - self.local_wall < 0.3,
            'PX4 status receipt age >= 1.0 s': now - self.status_wall < 1.0,
            'Gazebo truth receipt age >= 0.3 s': now - self.truth_wall < 0.3,
        }
        if self.local is not None:
            checks.update({
                'PX4 XY invalid': self.local.xy_valid,
                'PX4 Z invalid': self.local.z_valid,
                'PX4 dead reckoning': not self.local.dead_reckoning,
                'PX4 eph >= 0.8 m': self.local.eph < 0.8,
            })
        self.fresh_failure_reason = '; '.join(key for key, ok in checks.items() if not ok)
        return all(checks.values())

    def send(self, target):
        stamp = self.local.timestamp
        mode = OffboardControlMode()
        mode.timestamp = stamp
        mode.position = True
        self.mode_pub.publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = stamp
        setpoint.position = list(target[:3])
        setpoint.velocity = [math.nan] * 3
        setpoint.acceleration = [math.nan] * 3
        setpoint.jerk = [math.nan] * 3
        setpoint.yaw = target[3]
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.local.timestamp
        msg.command = command
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def spin_for(self, seconds, start, end):
        deadline = time.monotonic() + seconds
        begin = time.monotonic()
        next_send = begin
        truth_start = self.truth
        local_start = (self.local.x, self.local.y, self.local.z)
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if not self.fresh():
                raise RuntimeError('stale or invalid PX4/Gazebo state: ' + self.fresh_failure_reason)
            if (math.hypot(self.truth[0], self.truth[1]) > 2.0 or
                    self.truth[2] < -0.2 or self.truth[2] > 2.5):
                raise RuntimeError('Gazebo vehicle left calibration bounds')
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                    self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                raise RuntimeError('PX4 left Offboard during calibration')
            if now >= next_send:
                fraction = min(1.0, (now - begin) / seconds)
                self.send(tuple(a + fraction * (b - a) for a, b in zip(start, end)))
                next_send += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        print('segment end', 'truth', self.truth, 'truth_delta',
              tuple(b - a for a, b in zip(truth_start, self.truth)), 'px4_delta',
              tuple(b - a for a, b in zip(local_start,
                                           (self.local.x, self.local.y, self.local.z))),
              'px4',
              None if self.local is None else (self.local.x, self.local.y,
                                               self.local.z, self.local.heading),
              'heading_good', None if self.local is None else
              self.local.heading_good_for_control, flush=True)


def main():
    rclpy.init()
    node = CalibrationFlight()
    armed = False
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not node.fresh():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.fresh() or math.hypot(*node.truth[:2]) > 0.5:
            raise RuntimeError('SITL world or vehicle not ready at origin')
        p = node.local
        origin = (p.x, p.y, p.z, p.heading)
        if not all(math.isfinite(v) for v in origin):
            raise RuntimeError('non-finite PX4 local position or heading')
        if abs(p.z + node.truth[2]) > 0.15:
            raise RuntimeError('PX4 and Gazebo ground height differ by more than 0.15 m')
        print('prestream', origin, flush=True)
        node.spin_for(2.5, origin, origin)
        node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
        node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not node.fresh():
                raise RuntimeError('state lost after arming request')
            node.send(origin)
            rclpy.spin_once(node, timeout_sec=0.05)
            if (node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                    node.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                armed = True
                break
        if not armed:
            raise RuntimeError('PX4 rejected Offboard or arming')
        high = (origin[0], origin[1], origin[2] - 1.7, origin[3])
        yawed = (high[0], high[1], high[2], high[3] + 0.35)
        north = (high[0] + 0.5, high[1], high[2], high[3])
        northeast = (north[0], north[1] + 0.5, high[2], high[3])
        for name, duration, start, end in (
                ('climb', 10.0, origin, high),
                ('hover', 8.0, high, high),
                ('yaw_out', 3.0, high, yawed),
                ('yaw_back', 3.0, yawed, high),
                ('north', 4.0, high, north),
                ('east', 4.0, north, northeast),
                ('return_north', 4.0, northeast, (high[0], northeast[1], high[2], high[3])),
                ('return_east', 4.0, (high[0], northeast[1], high[2], high[3]), high)):
            if name == 'yaw_out' and (node.truth[2] < 1.5 or
                                      abs(node.local.z + node.truth[2]) > 0.15):
                raise RuntimeError('hover height did not settle')
            if name == 'north' and not node.local.heading_good_for_control:
                raise RuntimeError('PX4 in-flight heading alignment is incomplete')
            print(name, 'truth', node.truth, 'heading_good',
                  node.local.heading_good_for_control, flush=True)
            node.spin_for(duration, start, end)
        print('calibration motion complete; requesting land', flush=True)
    except Exception as exc:
        print('ABORT:', exc, flush=True)
    finally:
        if armed or (node.status and
                     node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    break
            if (node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and node.truth is not None and node.truth[2] < 0.05
                    and time.monotonic() - node.truth_wall < 0.3):
                node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 21196)
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                        break
            print('land result', None if node.status is None else
                  (node.status.arming_state, node.status.nav_state), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
