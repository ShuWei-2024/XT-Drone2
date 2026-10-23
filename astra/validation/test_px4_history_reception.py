"""DDS history must bridge a receive stall without accepting stale measurements."""
import sys
import time
from pathlib import Path

import pytest
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'packages/astra_px4_offboard/scripts'))
from measurement_pair_gate import recent_measurement_pair
from px4_topics import px4_sensor_history_qos
from ready_batch_executor import BoundedEventsExecutor, ReadyBatchExecutor


@pytest.mark.parametrize('executor_class', [ReadyBatchExecutor, BoundedEventsExecutor])
def test_actual_dds_history_retains_pair_during_receive_stall(executor_class):
    rclpy.init()
    node = Node('px4_history_reception_test')
    executor = executor_class()
    executor.add_node(node)
    shallow, retained = [], []
    topic = '/test_only/px4_history'  # No PX4 command publishers.
    node.create_subscription(VehicleLocalPosition, topic, shallow.append,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(VehicleLocalPosition, topic, retained.append,
                             px4_sensor_history_qos())
    pub = node.create_publisher(VehicleLocalPosition, topic, 64)
    try:
        deadline = time.monotonic() + 4.
        while (not shallow or not retained) and time.monotonic() < deadline:
            pub.publish(VehicleLocalPosition(timestamp_sample=1))
            executor.spin_once(timeout_sec=.01)
        assert shallow and retained, 'DDS discovery did not complete'
        shallow.clear(); retained.clear()
        # Do not spin while a 100 Hz source publishes for 200 ms. DDS can
        # receive in the background, while Python cannot run callbacks.
        for index in range(20):
            pub.publish(VehicleLocalPosition(
                timestamp_sample=1_000_000 + index * 10_000,
                z=-1., z_valid=True, z_reset_counter=2))
            time.sleep(.01)
        deadline = time.monotonic() + 2.
        while (not retained or retained[-1].timestamp_sample != 1_190_000
               or not shallow or shallow[-1].timestamp_sample != 1_190_000):
            assert time.monotonic() < deadline, 'DDS drain did not complete'
            executor.spin_once(timeout_sec=.01)
        assert any(m.timestamp_sample == 1_000_000 for m in retained)
        assert not any(m.timestamp_sample == 1_000_000 for m in shallow)
        measurement = [(1_000_000_000, 1.)]
        assert recent_measurement_pair(measurement, shallow, 1_200_000_000,
                                       z_reset_counter=2) is None
        assert recent_measurement_pair(measurement, retained, 1_200_000_000,
                                       z_reset_counter=2) is not None
        # Retention is not permission to use an expired or reset-era pair.
        assert recent_measurement_pair(measurement, retained, 1_251_000_000,
                                       z_reset_counter=2) is None
        assert recent_measurement_pair(measurement, retained, 1_200_000_000,
                                       z_reset_counter=3) is None
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
