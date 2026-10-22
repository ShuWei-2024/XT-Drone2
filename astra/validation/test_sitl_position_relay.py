"""Position-only outage must leave EV observers, status and commands untouched."""
import sys
import time
from pathlib import Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from px4_msgs.msg import VehicleLocalPosition
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from sitl_position_relay import PositionRelay, RELAY_TOPIC, relay_ros_args
from px4_topics import px4_sensor_history_qos, px4_topic


def test_relay_input_rejected_outside_isolated_sitl(monkeypatch):
    monkeypatch.delenv('ASTRA_FLIGHT_PROCESS_ISOLATED',raising=False)
    with pytest.raises(ValueError,match='isolated SITL'):relay_ros_args()


def test_actual_dds_position_relay_preserves_timestamps_and_isolates_outage(monkeypatch):
    monkeypatch.setenv('ASTRA_FLIGHT_PROCESS_ISOLATED','1')
    rclpy.init()
    source=px4_topic('out','vehicle_local_position',VehicleLocalPosition,'x500_0')
    direct=Node('position_relay_direct_observer',use_global_arguments=False)
    mission=Node('position_relay_mission',cli_args=relay_ros_args(),use_global_arguments=False)
    relay=PositionRelay()
    executor=SingleThreadedExecutor()
    for node in (direct,mission,relay):executor.add_node(node)
    original,forwarded=[],[]
    direct.create_subscription(VehicleLocalPosition,source,original.append,px4_sensor_history_qos())
    mission.create_subscription(VehicleLocalPosition,source,forwarded.append,px4_sensor_history_qos())
    publisher=direct.create_publisher(VehicleLocalPosition,source,px4_sensor_history_qos())
    msg=VehicleLocalPosition(timestamp=1234567,timestamp_sample=1234000,
        x=1.25,y=-.75,z=-1.5,heading=.5,xy_valid=True,z_valid=True,xy_reset_counter=3)
    relay_destroyed=False
    try:
        assert mission.resolve_topic_name(source)==RELAY_TOPIC
        for topic in ['/x500_0/fmu/in/trajectory_setpoint','/x500_0/fmu/in/vehicle_command',
                      '/x500_0/fmu/out/vehicle_status_v1']:
            assert mission.resolve_topic_name(topic)==topic
        deadline=time.monotonic()+5.
        while not original or not forwarded:
            assert time.monotonic()<deadline,'Relay discovery/forwarding timed out'
            publisher.publish(msg);executor.spin_once(timeout_sec=.01)
        assert serialize_message(forwarded[-1])==serialize_message(msg)
        assert relay.last_sample_us==1234000
        # Remove/destroy the independent relay. Direct PX4 observers still get
        # newer source data; mission position stays at the last real sample.
        executor.remove_node(relay);relay.destroy_node();relay_destroyed=True
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.005)
        before=len(forwarded);direct_before=len(original)
        msg.timestamp=2234567;msg.timestamp_sample=2234000
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:
            publisher.publish(msg);executor.spin_once(timeout_sec=.005)
        assert len(original)>direct_before
        assert original[-1].timestamp_sample==2234000
        assert len(forwarded)==before
        assert forwarded[-1].timestamp_sample==1234000
    finally:
        if not relay_destroyed:executor.remove_node(relay);relay.destroy_node()
        executor.remove_node(mission);executor.remove_node(direct)
        executor.shutdown();mission.destroy_node();direct.destroy_node()
        rclpy.try_shutdown()
