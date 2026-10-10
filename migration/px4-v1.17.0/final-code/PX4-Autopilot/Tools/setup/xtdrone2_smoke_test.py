#!/usr/bin/env python3
"""Validate PX4, Gazebo, XRCE Agent and the installed XTDrone2 ROS node.

Run after sourcing Tools/setup/xtdrone2_env.sh and ~/ws_xtd2/install/setup.bash.
This starts headless simulation and reads telemetry without arming the vehicle.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rclpy
from xtd2_communication.multirotor_communication import MultirotorCommunication


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xtdrone2-world', help='Validate an installed XTDrone2 world and spawn launch file')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    records = root / 'migration/px4-v1.17.0'
    records.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
        port.bind(('0.0.0.0', 8888))
    env = os.environ.copy()
    env.update(HEADLESS='1', PX4_UXRCE_DDS_NS='x500_0')
    prefix = 'xtdrone2-' + args.xtdrone2_world + '-' if args.xtdrone2_world else ''
    commands = [('agent-runtime', ['MicroXRCEAgent', 'udp4', '-p', '8888'])]
    if args.xtdrone2_world:
        from ament_index_python.packages import get_package_share_directory
        assets = Path(get_package_share_directory('xtd2_gz_sim'))
        env['GZ_SIM_RESOURCE_PATH'] = str(assets / 'models')
        commands.extend([
            ('gazebo-runtime', ['gz', 'sim', '-s', '-r', str(assets / 'worlds' / (args.xtdrone2_world + '.sdf'))]),
            ('launch-runtime', ['ros2', 'launch', 'xtd2_launch', 'xtd2_vehicle_spawn_launch.py',
                                'world_name:=' + args.xtdrone2_world, 'model:=gz_x500',
                                'id:=0', 'namespace:=x500_0']),
        ])
    else:
        commands.append(('px4-gazebo-runtime', ['make', 'px4_sitl', 'gz_x500']))
    processes = []
    logs = []
    node = None
    result = {'passed': False, 'namespace': 'x500_0', 'received_samples': 0}
    try:
        for name, command in commands:
            log = (records / (prefix + name + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, cwd=root, env=env,
                                              stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        rclpy.init()
        node = MultirotorCommunication('gz_x500', 0, debug=True)
        deadline = time.monotonic() + 90
        previous = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            position = node.cur_vehicle_local_position
            if position is not None and position.timestamp != previous:
                previous = position.timestamp
                result['received_samples'] += 1
                result['position'] = {'timestamp': int(position.timestamp),
                                      'x': float(position.x), 'y': float(position.y),
                                      'z': float(position.z),
                                      'xy_valid': bool(position.xy_valid),
                                      'z_valid': bool(position.z_valid)}
                if result['received_samples'] >= 5 and position.xy_valid and position.z_valid:
                    result['passed'] = True
                    break
            if processes[0].poll() is not None:
                raise RuntimeError('XRCE Agent exited before telemetry was received')
        result['topic'] = '/x500_0/fmu/out/vehicle_local_position_v1'
        result['agent_returncode_before_cleanup'] = processes[0].poll()
        result['simulation_returncode_before_cleanup'] = processes[-1].poll()
        result['world'] = args.xtdrone2_world or 'PX4 default'
    except Exception as error:
        result['error'] = repr(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in reversed(processes):
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)
        for process in processes:
            if process.stdin is not None:
                process.stdin.close()
        for log in logs:
            log.close()
        (records / (prefix + 'runtime-validation.json')).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
