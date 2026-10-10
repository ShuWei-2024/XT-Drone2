"""Move the simulated aerial arm; confirm the result using joint feedback."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

PRESETS = {'stow': (0.0, 0.0, 0.0), 'reach': (0.0, 0.55, -0.65), 'down': (0.0, 1.1, -0.35)}
JOINTS = ['arm_yaw_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'gripper_left_joint', 'gripper_right_joint']
COMMANDS = ['yaw', 'shoulder', 'elbow', 'gripper_left', 'gripper_right']


class ArmController(Node):
    def __init__(self, namespace):
        super().__init__('aerial_arm_control')
        self.states = {}
        self.publishers_ = [self.create_publisher(Float64, f'/{namespace}/arm/{name}/cmd_pos', 10) for name in COMMANDS]
        self.create_subscription(JointState, f'/{namespace}/arm/joint_states', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        self.states = dict(zip(message.name, message.position))

    def send(self, targets):
        for publisher, target in zip(self.publishers_, targets):
            publisher.publish(Float64(data=float(target)))


def main():
    parser = argparse.ArgumentParser(description='Command 3 arm angles (radians) and gripper stroke (metres). Does not arm the aircraft.')
    parser.add_argument('--namespace', default='x500_arm_0')
    parser.add_argument('--pose', choices=PRESETS, default='stow')
    parser.add_argument('--yaw', type=float)
    parser.add_argument('--shoulder', type=float)
    parser.add_argument('--elbow', type=float)
    parser.add_argument('--gripper', type=float, default=0.04, help='Total finger travel: 0 closed, 0.05 fully open; inner gap is 0.01 + travel')
    parser.add_argument('--duration', type=float, default=3.0, help='Ramp duration in seconds')
    args = parser.parse_args()
    targets = list(PRESETS[args.pose])
    for index, supplied in enumerate([args.yaw, args.shoulder, args.elbow]):
        if supplied is not None:
            targets[index] = supplied
    for number, (low, high) in zip(targets + [args.gripper, args.duration], [(-1.57, 1.57), (-.35, 1.45), (-1.6, 1.6), (0, .05), (.2, 30)]):
        if not math.isfinite(number) or not low <= number <= high:
            parser.error(f'{number} outside [{low}, {high}]')
    targets.extend([args.gripper / 2] * 2)
    rclpy.init()
    node = ArmController(args.namespace.strip('/'))
    try:
        deadline = time.monotonic() + 15
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(joint in node.states for joint in JOINTS) and all(p.get_subscription_count() for p in node.publishers_):
                break
        else:
            raise RuntimeError('No complete joint feedback/command subscribers. Start aerial_arm_launch.py first.')
        start = [node.states[joint] for joint in JOINTS]
        begun = time.monotonic()
        reached_since = None
        while rclpy.ok() and time.monotonic() - begun < args.duration + 10:
            fraction = min(1.0, (time.monotonic() - begun) / args.duration)
            blend = fraction * fraction * (3 - 2 * fraction)
            node.send([a + blend * (b - a) for a, b in zip(start, targets)])
            rclpy.spin_once(node, timeout_sec=.05)
            reached = fraction >= 1 and all(abs(node.states[joint] - target) < tolerance for joint, target, tolerance in zip(JOINTS, targets, [.08, .08, .08, .003, .003]))
            if reached:
                reached_since = reached_since or time.monotonic()
                if time.monotonic() - reached_since > .5:
                    print('PASS: arm reached target; measured positions:', {joint: round(node.states[joint], 4) for joint in JOINTS})
                    return
            else:
                reached_since = None
        raise RuntimeError(f'Joint tracking timeout; measured={node.states}, target={targets}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
