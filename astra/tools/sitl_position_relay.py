"""Test-only PX4 position relay: preserve messages and isolate a telemetry outage.

Only the EGO SITL child remaps its input to this output. EV, the independent
frame auditor, PX4 status, and every /fmu/in command retain their direct paths.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_sensor_history_qos, px4_topic

RELAY_TOPIC = '/astra/sitl_fault/vehicle_local_position'


def relay_ros_args(namespace='x500_0'):
    if os.environ.get('ASTRA_FLIGHT_PROCESS_ISOLATED') != '1':
        raise ValueError('Position relay input is restricted to the isolated SITL child')
    original = px4_topic('out', 'vehicle_local_position', VehicleLocalPosition, namespace)
    return ['--ros-args', '-r', original+':='+RELAY_TOPIC]


class PositionRelay(Node):
    def __init__(self, namespace='x500_0'):
        super().__init__('sitl_position_outage_relay', use_global_arguments=False)
        self.source = px4_topic('out', 'vehicle_local_position', VehicleLocalPosition, namespace)
        self.forwarded = 0
        self.first_sample_us = self.last_sample_us = None
        self.last_forward_wall_ns = None
        self.publisher = self.create_publisher(VehicleLocalPosition, RELAY_TOPIC,
                                               px4_sensor_history_qos())
        self.create_subscription(VehicleLocalPosition, self.source, self.forward,
                                 px4_sensor_history_qos())

    def forward(self, msg):
        # Do not republish to PX4 or stamp old samples as fresh.
        self.publisher.publish(msg)
        self.forwarded += 1
        if self.first_sample_us is None:
            self.first_sample_us = msg.timestamp_sample
        self.last_sample_us = msg.timestamp_sample
        self.last_forward_wall_ns = time.monotonic_ns()

    def summary(self):
        return dict(source=self.source, output=RELAY_TOPIC, forwarded=self.forwarded,
                    first_sample_us=self.first_sample_us, last_sample_us=self.last_sample_us,
                    last_forward_wall_ns=self.last_forward_wall_ns,
                    scope='Mission position input only; no command, EV or status remapping')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ASTRA_FLIGHT_PROCESS_ISOLATED') != '1':
        parser.error('Relay is restricted to an isolated SITL run')
    rclpy.init()
    node = PositionRelay(os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        args.output.write_text(json.dumps(node.summary(), indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
