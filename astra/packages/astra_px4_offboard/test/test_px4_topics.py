"""Verify DDS subscriptions with the installed PX4 message version."""

import sys
import unittest
from pathlib import Path

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from guarded_bspline_bridge import GuardedBsplineBridge
from px4_topics import px4_topic


class TopicTests(unittest.TestCase):
    def test_legacy_messages_keep_existing_topic(self):
        class LegacyMessage:
            pass
        self.assertEqual('/fmu/out/vehicle_status',
                         px4_topic('out', 'vehicle_status', LegacyMessage))

    def test_version_zero_has_no_suffix(self):
        self.assertEqual('/x500_0/fmu/in/trajectory_setpoint', px4_topic(
            'in', 'trajectory_setpoint', TrajectorySetpoint, '/x500_0/'))

    def test_real_bridge_subscribes_to_selected_namespace_and_version(self):
        rclpy.init()
        node = GuardedBsplineBridge(parameter_overrides=[
            Parameter('px4_namespace', value='x500_0')])
        try:
            subscriptions = {s.topic_name for s in node.subscriptions}
            for name, cls in [('vehicle_local_position', VehicleLocalPosition),
                              ('vehicle_status', VehicleStatus)]:
                self.assertIn(px4_topic('out', name, cls, 'x500_0'), subscriptions)
            self.assertFalse(hasattr(node, 'setpoint_pub'))
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
