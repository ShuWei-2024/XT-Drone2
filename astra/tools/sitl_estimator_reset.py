"""One real PX4 EKF reset in an explicitly isolated SITL fault experiment.

Uses PX4's external-observation commands, never forged output/reset counters.
The small observation change is a test stimulus, not calibration or EV fusion.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


def input_ages_valid(received, now):
    # PX4 status is event-driven and this SITL publishes it about every 0.5 s.
    # Match the supervisor's separate 1 s status and 250 ms position gates.
    limits = dict(local=.25, global_position=.3, status=1.)
    return all(key in received and 0 <= now-received[key] < limit
               for key, limit in limits.items())


def reset_parameters(axis, local, global_position):
    """Small, bounded observations; NaN altitude prevents an unintended Z reset."""
    if axis == 'yaw':
        return 620, dict(param3=math.degrees(local.heading)+2., param7=1.)
    if axis not in ('xy', 'z'):
        raise ValueError('Reset axis must be xy, z or yaw')
    return 43003, dict(param2=0., param3=.1,
        param5=global_position.lat + (.05/6371000.*180./math.pi if axis == 'xy' else 0.),
        param6=global_position.lon,
        param7=global_position.alt+.05 if axis == 'z' else math.nan)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', choices=('xy', 'z', 'yaw'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trigger-file', type=Path)
    parser.add_argument('--ready-file', type=Path)
    args = parser.parse_args()
    if os.environ.get('ASTRA_FLIGHT_PROCESS_ISOLATED') != '1':
        parser.error('Reset injection is restricted to an isolated SITL run')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import (VehicleLocalPosition, VehicleGlobalPosition,
                              VehicleStatus, VehicleCommand, VehicleCommandAck)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'packages/astra_px4_offboard/scripts'))
    from px4_topics import px4_sensor_history_qos, px4_topic
    rclpy.init()
    node = Node('sitl_real_estimator_reset')
    ns = os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0')
    latest = {}; received = {}; acknowledgements = []
    def receive(key, msg):
        latest[key] = msg
        received[key] = time.monotonic()
    for key, topic, cls in [('local', 'vehicle_local_position', VehicleLocalPosition),
                             ('global_position', 'vehicle_global_position', VehicleGlobalPosition),
                             ('status', 'vehicle_status', VehicleStatus)]:
        node.create_subscription(cls, px4_topic('out', topic, cls, ns),
            lambda msg, key=key: receive(key, msg), px4_sensor_history_qos())
    node.create_subscription(VehicleCommandAck,
        px4_topic('out', 'vehicle_command_ack', VehicleCommandAck, ns),
        lambda msg: acknowledgements.append(dict(command=msg.command, result=msg.result,
            timestamp_us=msg.timestamp, wall_ns=time.monotonic_ns())), qos_profile_sensor_data)
    publisher = node.create_publisher(VehicleCommand,
        px4_topic('in', 'vehicle_command', VehicleCommand, ns), 10)
    result = dict(axis=args.axis, injected=False)
    try:
        deadline = time.monotonic()+4.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if input_ages_valid(received, time.monotonic()):
                break
        if set(latest) != {'local', 'global_position', 'status'}:
            raise RuntimeError('Missing direct PX4 samples before reset')
        if args.trigger_file:
            if args.ready_file:
                args.ready_file.touch()
            deadline = time.monotonic()+300.
            while not args.trigger_file.exists():
                if time.monotonic() >= deadline:
                    raise RuntimeError('No bounded SITL reset trigger received')
                rclpy.spin_once(node, timeout_sec=.01)
            # Drain already queued samples without delaying a stale-state gate.
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.)
        local, status = latest['local'], latest['status']
        result['preinjection_state'] = dict(arming_state=status.arming_state,
            nav_state=status.nav_state, xy_valid=local.xy_valid, z_valid=local.z_valid,
            receipt_age_s={key: time.monotonic()-value for key, value in received.items()})
        if (not input_ages_valid(received, time.monotonic()) or
                status.arming_state != VehicleStatus.ARMING_STATE_ARMED or
                status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                not local.xy_valid or not local.z_valid):
            raise RuntimeError('Reset requires fresh, valid, armed Offboard state')
        counters = lambda m: [m.xy_reset_counter, m.z_reset_counter, m.heading_reset_counter]
        command, params = reset_parameters(args.axis, local, latest['global_position'])
        msg = VehicleCommand()
        msg.timestamp = local.timestamp
        msg.command = command
        msg.target_system = msg.target_component = 1
        msg.source_system = 1; msg.source_component = 191
        msg.from_external = True
        for key, value in params.items():
            setattr(msg, key, float(value))
        result.update(kind='real_px4_external_observation_reset', injected=True,
            command_wall_ns=time.monotonic_ns(), command_sim_us=local.timestamp,
            command=command, before_counters=counters(local),
            before_sample_us=local.timestamp_sample,
            parameters={key: (value if math.isfinite(value) else 'NaN') for key, value in params.items()})
        publisher.publish(msg)
        # Persist the send even if later acknowledgement/observation fails.
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            current = latest['local']
            if counters(current) != result['before_counters'] and 'after_counters' not in result:
                result.update(after_counters=counters(current),
                    after_sample_us=current.timestamp_sample, observed_wall_ns=time.monotonic_ns())
            if 'after_counters' in result and any(a['command'] == command for a in acknowledgements):
                break
        result['acknowledgements'] = acknowledgements
        index = {'xy': 0, 'z': 1, 'yaw': 2}[args.axis]
        result['requested_counter_changed'] = ('after_counters' in result and
            result['after_counters'][index] != result['before_counters'][index])
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()
    return 0 if result.get('requested_counter_changed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
