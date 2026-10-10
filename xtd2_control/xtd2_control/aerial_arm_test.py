"""Feedback-based joint test and optional PX4 takeoff/arm-motion/landing test."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLocalPosition, VehicleStatus

from xtd2_control.aerial_arm_control import ArmController, JOINTS


def topic(namespace, direction, name, message_type):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    return f'/{namespace}/fmu/{direction}/{name}' + (f'_v{version}' if version else '')


class TestController(ArmController):
    def __init__(self, namespace, vehicle_id, gcs_heartbeat=False):
        super().__init__(namespace)
        self.position = None
        self.status = None
        self.acknowledgements = {}
        self.vehicle_id = vehicle_id
        self.goal = None
        self.heartbeat = False
        self.maximum_drift = 0.0
        self.start_position = None
        self.gcs = None
        if gcs_heartbeat:
            from pymavlink import mavutil
            # Receive this local SITL instance's MAVLink telemetry and reply
            # with an ordinary GCS heartbeat; no arming checks are disabled.
            self.gcs = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255, source_component=190)
            self.create_timer(.5, self.gcs_keep_alive)
        self.create_subscription(VehicleLocalPosition, topic(namespace, 'out', 'vehicle_local_position', VehicleLocalPosition), self.on_position, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, topic(namespace, 'out', 'vehicle_status', VehicleStatus), self.on_status, qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, topic(namespace, 'out', 'vehicle_command_ack', VehicleCommandAck), self.on_ack, qos_profile_sensor_data)
        self.mode_publisher = self.create_publisher(OffboardControlMode, topic(namespace, 'in', 'offboard_control_mode', OffboardControlMode), 10)
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, topic(namespace, 'in', 'trajectory_setpoint', TrajectorySetpoint), 10)
        self.command_publisher = self.create_publisher(VehicleCommand, topic(namespace, 'in', 'vehicle_command', VehicleCommand), 10)
        self.create_timer(.05, self.keep_alive)

    def gcs_keep_alive(self):
        from pymavlink import mavutil
        for _ in range(100):
            if self.gcs.recv_match(blocking=False) is None:
                break
        self.gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

    def on_position(self, message):
        self.position = message
        if self.start_position and self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            x, y, z = self.start_position
            self.maximum_drift = max(self.maximum_drift, math.hypot(message.x-x, message.y-y))

    def on_status(self, message):
        self.status = message

    def on_ack(self, message):
        self.acknowledgements[message.command] = message.result

    def keep_alive(self):
        if self.heartbeat and self.goal:
            timestamp = self.get_clock().now().nanoseconds // 1000
            self.mode_publisher.publish(OffboardControlMode(timestamp=timestamp, position=True))
            self.setpoint_publisher.publish(TrajectorySetpoint(timestamp=timestamp, position=self.goal[:3], yaw=self.goal[3]))

    def command(self, number, param1=0.0, param2=0.0):
        self.acknowledgements.pop(number, None)
        message = VehicleCommand(timestamp=self.get_clock().now().nanoseconds // 1000, command=number, param1=float(param1), param2=float(param2), param4=math.nan, param5=math.nan, param6=math.nan, param7=math.nan, target_system=self.vehicle_id+1, target_component=1, source_system=255, source_component=190, from_external=True)
        self.command_publisher.publish(message)

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                print('PASS:', description, flush=True)
                return
        raise RuntimeError('Timeout: ' + description)

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)

    def move_arm(self, target, description):
        origin = [self.states[joint] for joint in JOINTS]
        start = time.monotonic()
        stable_since = None
        while time.monotonic() - start < 15:
            t = min(1., (time.monotonic()-start)/3.)
            blend = t*t*(3-2*t)
            self.send([a+(b-a)*blend for a, b in zip(origin, target)])
            rclpy.spin_once(self, timeout_sec=.05)
            if self.heartbeat and self.start_position:
                if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    raise RuntimeError('Vehicle left armed Offboard flight during arm motion')
                if self.start_position[2] - self.position.z < .8:
                    raise RuntimeError('Height below 0.8 m during airborne arm motion')
            if t == 1 and all(abs(self.states[joint]-desired) < tol for joint, desired, tol in zip(JOINTS, target, [.10, .10, .10, .003, .003])):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since > .5:
                    print('PASS:', description, {joint: round(self.states[joint], 4) for joint in JOINTS}, flush=True)
                    return
            else:
                stable_since = None
        raise RuntimeError(f'Arm tracking timeout: {description}, measured={self.states}, target={target}')


def main():
    parser = argparse.ArgumentParser(description='Test feedback from a running aerial_arm_launch.py; --fly additionally arms and flies the simulated aircraft.')
    parser.add_argument('--namespace', default='x500_arm_0')
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--fly', action='store_true')
    parser.add_argument('--gcs-heartbeat', action='store_true', help='Provide a local simulated GCS heartbeat for the flight test when QGroundControl is not running')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    if args.gcs_heartbeat and not args.fly:
        parser.error('--gcs-heartbeat is only used together with --fly')
    node = TestController(args.namespace.strip('/'), args.id, args.gcs_heartbeat)
    result = {'passed': False, 'flight_requested': args.fly}
    try:
        node.wait(lambda: all(joint in node.states for joint in JOINTS) and all(p.get_subscription_count() for p in node.publishers_), 45, 'all five joint states and command subscribers')
        node.move_arm([0., 0., 0., .02, .02], 'stow / gripper open')
        node.move_arm([.35, .55, -.65, 0., 0.], 'yaw + reach / gripper close')
        node.move_arm([0., 1.1, -.35, .02, .02], 'arm down / gripper open')
        node.move_arm([0., 0., 0., .02, .02], 'return to stow')
        if args.fly:
            node.wait(lambda: node.position is not None and node.position.xy_valid and node.position.z_valid and node.status is not None, 45, 'valid PX4 position and vehicle status')
            if node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                raise RuntimeError('Flight test requires a disarmed aircraft on the ground')
            node.start_position = (node.position.x, node.position.y, node.position.z)
            node.goal = [node.position.x, node.position.y, node.position.z-2., node.position.heading]
            node.heartbeat = True
            node.pause(2.)
            node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            node.wait(lambda: node.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD, 8, 'Offboard confirmed by VehicleStatus')
            node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            node.wait(lambda: node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED, 10, 'arming confirmed by VehicleStatus')
            node.wait(lambda: abs(node.position.z-node.goal[2]) < .3, 30, 'takeoff to 2 m')
            node.pause(3.)
            node.move_arm([.25, .55, -.65, .02, .02], 'move arm while hovering')
            node.move_arm([0., 0., 0., 0., 0.], 'close gripper and stow while hovering')
            if node.maximum_drift > 1.:
                raise RuntimeError(f'Horizontal drift exceeded 1 m: {node.maximum_drift}')
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            node.wait(lambda: node.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND, 8, 'landing mode confirmed')
            node.heartbeat = False
            node.wait(lambda: node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED, 45, 'landed and automatically disarmed')
            result['maximum_horizontal_drift_m'] = node.maximum_drift
        result['passed'] = True
        result['joint_positions'] = node.states
    except Exception as error:
        result['error'] = str(error)
        result['command_acknowledgements'] = node.acknowledgements
        print('FAIL:', result, flush=True)
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            node.heartbeat = False
            try:
                node.wait(lambda: node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED, 45, 'cleanup landing')
            except RuntimeError:
                print('Landing did not finish; inspect the running simulation.', flush=True)
        raise
    finally:
        if args.output:
            args.output.write_text(json.dumps(result, indent=2))
        if node.gcs:
            node.gcs.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
