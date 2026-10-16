#!/usr/bin/env python3
"""Replay recorded sensor map/odom to EGO only; never publishes PX4 inputs.

Occupied observations are reconstructed with the recorded static-map settings.
This isolates planning failures; it is not a closed-loop flight validation.
Run in a separate ROS_DOMAIN_ID with no simulator running.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header
from traj_utils.msg import Bspline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=28.)
    parser.add_argument('--goal-index', type=int, default=0,
                        help='Recorded goal event and its preceding sensor observations')
    args = parser.parse_args()
    from audit_height_dynamics_117 import active_isolated_simulators
    if active_isolated_simulators(): parser.error('Stop simulation before offline planner replay')
    if os.environ.get('ROS_DOMAIN_ID') in (None, '94'): parser.error('Use a separate explicit ROS_DOMAIN_ID')
    events = json.loads((args.run/'motion_events.json').read_text())
    runtime = json.loads((args.run/'runtime.json').read_text())
    goal_event=next((e for e in events['events'] if e['event']=='ego_goal_sent'
                     and e['goal_index']==args.goal_index),None)
    if goal_event is None:parser.error('No recorded goal at the requested index')
    start = goal_event['sim_ns'] - 500_000_000
    args.output.mkdir(parents=True, exist_ok=False)
    rows = [row for row in csv.DictReader((args.run/'frame_samples.csv').open())
            if row['source'] == 'fast_livo2']
    stamps = np.array([int(row['measurement_time_ns']) for row in rows])
    lio_velocities={int(row['measurement_ns']):[float(row[key]) for key in ('update_vx','update_vy','update_vz')]
        for row in csv.DictReader((args.run/'livo_timing.csv').open()) if row['stage']=='LIO'}
    maps = np.load(args.run/'local_map_samples.npz')
    module_path = Path(__file__).resolve().parents[1]/'packages/astra_fast_livo2/scripts/local_mapping_guard.py'
    spec = importlib.util.spec_from_file_location('mapping_guard', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    voxel = .2
    offsets = np.array([(x,y,z) for x in range(-4,5) for y in range(-4,5)
                        for z in range(-4,5) if math.sqrt(x*x+y*y+z*z)*voxel <= .65+1e-9])
    inflation = runtime['descent_column_inflation_m']
    size = math.ceil(inflation/voxel)
    column_offsets = np.array([(x,y,0) for x in range(-size,size+1)
                              for y in range(-size,size+1) if math.hypot(x,y)*voxel <= inflation+1e-9])
    inflated = []
    for i in range(len(maps['stamps'])):
        cells = np.floor(maps['xyz'][maps['offsets'][i]:maps['offsets'][i+1]]/voxel).astype(np.int64)
        inflated_cells = (module.static_descent_obstacles(cells, offsets, voxel, .15,
            runtime['planner_virtual_ceiling_m'], column_offsets,
            maximum_obstacle_z=runtime.get('column_obstacle_max_z_m',runtime['planner_virtual_ceiling_m']))
            if runtime['static_descent_columns'] else module.inflate_cells(cells, offsets))
        inflated.append(((inflated_cells+.5)*voxel).astype(np.float32))
    rclpy.init()
    node = Node('planner_input_replay')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    odom_pub = node.create_publisher(Odometry, '/fast_livo2/odom', 10)
    map_pub = node.create_publisher(PointCloud2, '/local_mapping/obstacles', 2)
    ready_pub = node.create_publisher(Bool, '/local_mapping/ready',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
    envelope = None
    envelope_pub = None
    if runtime.get('shared_execution_envelope'):
        from planning_envelope import PlanningEnvelope
        from scan_mesh_guard import MeshGuard
        from nav_msgs.msg import OccupancyGrid
        frozen = next(e for e in events['events'] if e['event']=='shared_execution_envelope_frozen')
        guard = MeshGuard(args.run/'mesh_guard.npz')
        envelope = PlanningEnvelope(guard.flight_boxes, guard.bounds, frozen['ground_origin_livo'],
            reserve=frozen['reserve_m'], z_bounds=frozen.get('z_bounds_livo_m'),
            connected_to_origin=guard.metadata.get('whole_map_selection', False))
        envelope_pub = node.create_publisher(OccupancyGrid, '/planning/execution_envelope',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    heartbeats, splines, validity = [], [], []
    envelope_checks = []
    node.create_subscription(Empty, '/planning/heartbeat', lambda m: heartbeats.append(time.monotonic()), 10)
    node.create_subscription(Bool, '/planning/trajectory_valid',
                             lambda m: validity.append([time.monotonic(), m.data]), 10)
    def on_spline(m):
        splines.append([time.monotonic(), m.traj_id])
        if envelope is not None:
            from scipy.interpolate import BSpline
            try:
                envelope.validate_spline(BSpline(m.knots, [[p.x,p.y,p.z] for p in m.pos_pts], m.order))
                envelope_checks.append(True)
            except ValueError: envelope_checks.append(False)
    node.create_subscription(Bspline, '/planning/bspline', on_spline, 10)
    log = (args.output/'planner.log').open('w')
    process = subprocess.Popen(['ros2','launch','astra_fast_livo2','ego_planner_guarded.launch.py',
        'max_vel:=0.20','max_acc:=0.25',
        f'map_size_xy_m:={(runtime.get("prepared_outdoor_envelope") or {}).get("planner_map_size_xy_m",32.)}',
        f'require_execution_envelope:={str(envelope is not None).lower()}',
        f"planning_horizon:={runtime.get('planner_horizon_m',10. if 'planning_horizon:=10.0' in (args.run/'runner.py').read_text() else 6.)}",'max_tracking_error_m:=0.60',
        f"virtual_ceiling_m:={runtime['planner_virtual_ceiling_m']}",
        f"optimization_clearance_m:={runtime['optimization_clearance_m']}"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    begin = time.monotonic()
    last_map = -1
    goal_sent = False
    last_position = None
    try:
        while time.monotonic()-begin < args.seconds+3.:
            elapsed = time.monotonic()-begin
            ns = int(start + max(0.,elapsed-3.)*1e9)
            clock = Clock(); clock.clock.sec,clock.clock.nanosec = divmod(ns,10**9)
            clock_pub.publish(clock)
            if envelope_pub is not None and elapsed < 3.:
                envelope_pub.publish(envelope.message(clock.clock))
            i = int(np.clip(np.searchsorted(stamps, ns, side='right')-1, 0, len(rows)-1))
            row = rows[i]
            odom = Odometry(); odom.header = Header(stamp=clock.clock, frame_id='camera_init')
            p = odom.pose.pose.position
            p.x,p.y,p.z = (float(row[k]) for k in ('x_m','y_m','z_m'))
            odom.pose.pose.orientation.w = 1.
            position = np.array([p.x,p.y,p.z])
            if int(row['measurement_time_ns']) in lio_velocities:
                v=lio_velocities[int(row['measurement_time_ns'])]
                odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=v
            elif last_position is not None:
                v = (position-last_position)/.05
                odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z = v
            last_position = position
            odom_pub.publish(odom)
            j = int(np.clip(np.searchsorted(maps['stamps'],ns,side='right')-1,0,len(inflated)-1))
            if j != last_map or elapsed < 3.:
                map_pub.publish(point_cloud2.create_cloud_xyz32(odom.header,inflated[j]))
                last_map = j
            ready_pub.publish(Bool(data=True))
            if elapsed >= 3. and not goal_sent:
                goal = PoseStamped();goal.header=odom.header;goal.pose.orientation.w=1.
                goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=goal_event['goal']
                goal_pub.publish(goal);goal_sent=True
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5.)
        log.close()
        gaps=np.diff(heartbeats)*1000.
        result=dict(scope='recorded sensor observations, reconstructed inflation, odom replay; no flight',
            goal_index=args.goal_index,goal=goal_event['goal'],
            heartbeat_count=len(heartbeats),heartbeat_max_gap_ms=float(gaps.max()) if len(gaps) else None,
            heartbeat_p95_gap_ms=float(np.percentile(gaps,95)) if len(gaps) else None,
            trajectories=splines,validity=validity,
            shared_envelope_curve_checks=envelope_checks,
            shared_envelope_all_curves_passed=bool(envelope_checks and all(envelope_checks)))
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('trajectories','validity')}))
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
