"""One bounded EGO-planned SITL leg, with exclusive setpoint ownership."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).parent))
from obstacle_world_geometry_20261010 import clearances, load as load_obstacles

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/astra_px4_offboard/scripts'))
from guarded_position_sitl import GuardedPositionTest
from guarded_bspline_bridge import GuardedBsplineBridge
from preview_bspline import PreviewBspline, stamp_ns


def receive_batch(executor):
    # A /clock callback can precede already queued odometry after a scheduling
    # pause. Process a bounded batch before checking freshness, so the guard
    # sees received measurements rather than only the first ready callback.
    executor.spin_once(timeout_sec=.002)
    deadline = time.monotonic() + .004
    for _ in range(32):
        if time.monotonic() >= deadline:
            break
        executor.spin_once(timeout_sec=0.)


def takeoff(node):
    deadline = time.monotonic() + 25.
    good_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
        if not (node.fault() or node.initial_alignment_fault()):
            good_since = good_since or time.monotonic()
            if time.monotonic() - good_since >= 5.:
                break
        else:
            good_since = None
    else:
        raise RuntimeError('preflight not healthy: ' + (node.fault() or node.initial_alignment_fault()))
    p = node.local
    origin = (p.x, p.y, p.z, p.heading)
    node.vertical_reference = p.z + node.truth[2]
    if node.initial_alignment_fault():
        raise RuntimeError('invalid initial world position')
    node.guarded_segment('prestream', 2., origin, origin)
    node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
    node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
    deadline = time.monotonic() + 5.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
        node.check_safe()
        node.send(origin)
        if (node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                node.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            break
    else:
        raise RuntimeError('Offboard/arming rejected')
    node.resets = (node.local.xy_reset_counter, node.local.z_reset_counter,
                   node.local.heading_reset_counter)
    high = (*origin[:2], origin[2] - 1.7, origin[3])
    node.guarded_segment('climb', 16., origin, high)
    node.wait_heading_alignment(high)
    node.guarded_segment('settle', 3., high, high)
    return high


def await_initial_yaw_agreement(node, bridge, executor, high, maximum_deg):
    """Check paired estimator headings before freezing the SITL frame or yaw."""
    node.phase = 'initial_yaw_agreement'
    node.event('initial_yaw_agreement_wait', maximum_deg=maximum_deg,
               required_sec=2., timeout_sec=30.)
    deadline = time.monotonic()+30.
    next_send = next_log = 0.
    good_wall = good_measurement = last_stamp = None
    last_error = None
    while time.monotonic() < deadline:
        receive_batch(executor)
        node.check_safe()
        now = time.monotonic()
        if now >= next_send:
            node.send(high)
            next_send = now+.05
        valid = (bridge.odom is not None and bridge.px4_history and
                 bridge.fresh(bridge.odom_wall, .25) and bridge.fresh(bridge.px4_wall, .25))
        stamp = stamp_ns(bridge.odom.header.stamp) if valid else None
        if valid:
            matched = min(bridge.px4_history, key=lambda p: abs(p.timestamp_sample*1000-stamp))
            q = bridge.odom.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            last_error = abs(math.degrees(math.atan2(
                math.sin(math.pi/2-matched.heading-yaw),
                math.cos(math.pi/2-matched.heading-yaw))))
            valid = (abs(matched.timestamp_sample*1000-stamp) <= 20_000_000 and
                     0 <= node.get_clock().now().nanoseconds-stamp <= 250_000_000 and
                     matched.heading_good_for_control and math.isfinite(last_error) and
                     last_error <= maximum_deg)
        if not valid or (last_stamp is not None and stamp is not None and
                         (stamp < last_stamp or stamp-last_stamp > 250_000_000)):
            good_wall = good_measurement = None
        if valid and stamp != last_stamp:
            if good_wall is None:
                good_wall, good_measurement = now, stamp
            if now-good_wall >= 2. and stamp-good_measurement >= 2_000_000_000:
                node.event('initial_yaw_agreement_complete', disagreement_deg=last_error,
                           continuous_sec=(stamp-good_measurement)/1e9)
                return
        last_stamp = stamp
        if now >= next_log:
            node.event('initial_yaw_agreement_sample', disagreement_deg=last_error,
                       valid=bool(valid), continuous_sec=0. if good_wall is None else now-good_wall)
            next_log = now+.5
    raise RuntimeError(f'initial paired yaw agreement did not remain within {maximum_deg:.1f}deg '
                       f'for 2s; last disagreement={last_error}')


def execute(node, high, result, goal_distance_m=0.5, goal_y_offset_m=0.0,
            obstacle_audit=False, max_tracking_error_m=0.30, prepared_nodes=None, single_goal=False,
            fixed_enu_ned_transform=False, obstacles=None, route_offsets=None,
            max_initial_yaw_disagreement_deg=5.):
    preview, bridge = prepared_nodes
    # Supervisor calls tick only after a clean handoff; no bridge preflight output.
    bridge.output_timer.cancel()
    executor = SingleThreadedExecutor()
    for n in (node, preview, bridge):
        executor.add_node(n)
    goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
    bridge_rows = []
    raw_splines = []
    from traj_utils.msg import Bspline
    def record_spline(msg):
        raw_splines.append(dict(start_ns=stamp_ns(msg.start_time), order=msg.order,
                                knots=list(msg.knots),
                                controls=[[p.x, p.y, p.z] for p in msg.pos_pts]))
    subscription = node.create_subscription(Bspline, '/planning/bspline', record_spline, 10)
    handed = False
    try:
        if fixed_enu_ned_transform:
            await_initial_yaw_agreement(node, bridge, executor, high, max_initial_yaw_disagreement_deg)
        node.phase = 'alignment_capture'
        pairs = []
        deadline, next_send, last_stamp = time.monotonic() + 2., 0., None
        while time.monotonic() < deadline:
            receive_batch(executor)
            node.check_safe()
            if time.monotonic() >= next_send:
                node.send(high)
                next_send = time.monotonic() + .05
            if bridge.odom is None or not bridge.px4_history:
                continue
            stamp = stamp_ns(bridge.odom.header.stamp)
            matched = min(bridge.px4_history, key=lambda p: abs(p.timestamp_sample * 1000 - stamp))
            if stamp == last_stamp or abs(matched.timestamp_sample * 1000 - stamp) > 20_000_000:
                continue
            p, q = bridge.odom.pose.pose.position, bridge.odom.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            pairs.append(([p.x, p.y, p.z], [matched.x, matched.y, matched.z],
                          math.pi/2 - matched.heading - yaw))
            last_stamp = stamp
        if len(pairs) < 8:
            raise RuntimeError('not enough time-paired alignment samples')
        if fixed_enu_ned_transform and any(abs(math.degrees(p[2])) > max_initial_yaw_disagreement_deg
                                           for p in pairs):
            raise RuntimeError('initial yaw agreement lost during alignment capture')
        # Freeze once; never follow estimator drift by continuously changing origin.
        if fixed_enu_ned_transform:
            # The simulated mounts use the audited ENU -> NED axis exchange.
            # Do not fold short-term yaw-estimator noise into this rigid frame.
            bridge.matrix = np.array([[0., 1.], [1., 0.]])
        else:
            angle = math.atan2(np.mean([math.sin(p[2]) for p in pairs]),
                               np.mean([math.cos(p[2]) for p in pairs]))
            bridge.matrix = np.array([[math.sin(angle), math.cos(angle)],
                                      [math.cos(angle), -math.sin(angle)]])
        bridge.translation = np.median([np.array(p[1][:2]) - bridge.matrix @ p[0][:2]
                                        for p in pairs], axis=0)
        bridge.z_translation = float(np.median([p[1][2]+p[0][2] for p in pairs]))
        result['alignment'] = dict(xy_matrix=bridge.matrix.tolist(),
                                   xy_translation=bridge.translation.tolist(),
                                   z_translation=bridge.z_translation, paired_samples=len(pairs),
                                   rotation_source=('fixed ENU/NED for zero-yaw SITL spawn'
                                                    if fixed_enu_ned_transform else 'paired estimator headings'),
                                   certified=False)
        p = node.odom.pose.pose.position
        origin_livo = (p.x, p.y, p.z)
        goal_queue = ([(3.0, 3.0), (goal_distance_m, goal_y_offset_m)]
                      if obstacle_audit and not single_goal else [(goal_distance_m, goal_y_offset_m)])
        if route_offsets is not None:
            goal_queue = route_offsets
        result['route_offsets_m'] = goal_queue
        result['origin_livo'] = list(origin_livo)
        goal_index = 0
        target = None
        goal_sent_ns = None

        def publish_goal(offset_x, offset_y):
            nonlocal target, goal_sent_ns
            gx, gy, gz = origin_livo[0] + offset_x, origin_livo[1] + offset_y, origin_livo[2]
            goal = PoseStamped()
            goal.header.frame_id = 'camera_init'
            goal.header.stamp = node.get_clock().now().to_msg()
            goal_sent_ns = stamp_ns(goal.header.stamp)
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, gz
            goal.pose.orientation.w = 1.
            target = bridge.transform((gx, gy, gz))
            result['goal_livo'] = [gx, gy, gz]
            result['goal_ned'] = target.tolist()
            goal_pub.publish(goal)
            node.event('ego_goal_sent', goal=result['goal_livo'], target_ned=target.tolist(),
                       goal_index=goal_index)

        publish_goal(*goal_queue[goal_index])
        leg_timeout = 100. if obstacle_audit else 25.
        deadline, next_send, next_tick = time.monotonic() + leg_timeout, 0., 0.
        final_sim_ns = None
        next_diagnostic = 0.
        while time.monotonic() < deadline:
            receive_batch(executor)
            node.check_safe()
            now = time.monotonic()
            if not handed:
                if now >= next_diagnostic:
                    node.event('waiting_for_ego', bridge_health=bridge.health(),
                               accepted_spline=preview.spline is not None,
                               trajectory_valid=preview.valid,
                               planner_heartbeat_age=None if preview.planner_wall is None else
                               now-preview.planner_wall)
                    next_diagnostic = now+2.
                if now >= next_send:
                    node.send(high)
                    next_send = now+.05
                if not bridge.health():
                    handed = True
                    node.phase = 'ego_execution'
                    result['truth_start'] = list(node.truth)
                    node.event('ego_control_handoff')
                    next_tick = now
                continue
            if (preview.span is not None and preview.trajectory_start_ns is not None
                    and preview.trajectory_start_ns >= goal_sent_ns):
                final_sim_ns = preview.trajectory_start_ns + int((preview.span[1]-preview.span[0])*1e9)
            # Normal expiry is handled by the supervisor, never disguised as a fault.
            # Continue endpoint tracking under the same freshness/bounds protections.
            if (final_sim_ns is not None and
                    node.get_clock().now().nanoseconds >= final_sim_ns-20_000_000 and
                    preview.ready and preview.fresh(preview.planner_wall, .25) and
                    (preview.valid or preview.spline is None)
                    and np.linalg.norm(target - [node.local.x, node.local.y, node.local.z]) <= .20):
                if goal_index + 1 < len(goal_queue):
                    node.event('ego_waypoint_reached', goal_index=goal_index)
                    goal_index += 1
                    publish_goal(*goal_queue[goal_index])
                    final_sim_ns = None
                    deadline = now + leg_timeout
                    continue
                node.event('ego_waypoint_reached', goal_index=goal_index)
                node.event('ego_trajectory_sampled_to_end')
                result['trajectory_sampled_to_end'] = True
                break
            if now >= next_tick:
                bridge.tick()
                bridge_rows.append(dict(sim_ns=node.get_clock().now().nanoseconds,
                    wall_ns=time.monotonic_ns(), state=bridge.mode, truth=list(node.truth), goal_index=goal_index,
                    tracking_yaw_ned_rad=bridge.tracking_yaw,
                    px4=[node.local.x,node.local.y,node.local.z],
                    target=None if bridge.pose is None else bridge.transform((
                        bridge.pose.pose.position.x, bridge.pose.pose.position.y,
                        bridge.pose.pose.position.z)).tolist()))
                if obstacle_audit:
                    # Static boxes in astra_px4_open.sdf.
                    x, y = node.truth[:2]
                    rectangles = ((3.65, 4.35, 1.65, 2.35),
                                  (3.65, 4.35, -2.35, -1.65),
                                  (6.5, 7.5, -.4, .4))
                    clearance = (float(clearances([[x, y]], obstacles)[0]) if obstacles else
                                 min(math.hypot(max(x-xmax, 0., xmin-x),
                                                max(y-ymax, 0., ymin-y))
                                     for xmin, xmax, ymin, ymax in rectangles))
                    result.setdefault('obstacle_clearance_samples_m', []).append(clearance)
                    if clearance < .55:
                        raise RuntimeError(f'obstacle clearance below 0.55 m: {clearance:.3f} m')
                if bridge.fault_since is not None:
                    raise RuntimeError('bridge fault: '+bridge.fault_reason)
                next_tick = now+.05
        else:
            raise RuntimeError('EGO trajectory did not finish before its bounded deadline; '+bridge.health())
    finally:
        for n in (node, preview, bridge):
            executor.remove_node(n)
        executor.shutdown()
        preview.destroy_node()
        bridge.destroy_node()
        node.destroy_subscription(subscription)
        result['bridge_samples'] = bridge_rows
        result['bsplines'] = raw_splines
    # Bridge has stopped. Only the supervisor now owns setpoints for endpoint settling.
    endpoint_yaw = (bridge.tracking_yaw if bridge.freeze_tracking_yaw and
                    bridge.tracking_yaw is not None else node.local.heading)
    endpoint = (*result['goal_ned'], endpoint_yaw)
    node.guarded_segment('endpoint_hold', 3., endpoint, endpoint)
    result['truth_end'] = list(node.truth)
    result['actual_displacement_enu'] = (np.array(node.truth)-result['truth_start']).tolist()
    result['endpoint_px4_error_m'] = float(np.linalg.norm(
        np.array(result['goal_ned'])-[node.local.x,node.local.y,node.local.z]))
    path = np.asarray([row['truth'] for row in bridge_rows], dtype=float)
    result['actual_sampled_path_length_m'] = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
    if result['actual_sampled_path_length_m'] < .25:
        raise RuntimeError('insufficient actual motion for planner-driven flight')
    result['flight_complete'] = True
    node.event('ego_flight_complete', actual_displacement=result['actual_displacement_enu'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-prefix', required=True)
    parser.add_argument('--goal-distance-m', type=float, default=.5)
    parser.add_argument('--goal-y-offset-m', type=float, default=0.0)
    parser.add_argument('--obstacle-audit', action='store_true')
    parser.add_argument('--single-goal', action='store_true')
    parser.add_argument('--calibrate-ground-height',action='store_true')
    parser.add_argument('--max-tracking-error-m', type=float, default=.30)
    parser.add_argument('--max-estimator-xy-disagreement-m', type=float, default=.30)
    parser.add_argument('--fixed-enu-ned-transform', action='store_true')
    parser.add_argument('--velocity-feedforward', action='store_true')
    parser.add_argument('--freeze-tracking-yaw', action='store_true')
    parser.add_argument('--max-initial-yaw-disagreement-deg', type=float, default=5.)
    parser.add_argument('--route-json', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.max_initial_yaw_disagreement_deg) or not 0 < args.max_initial_yaw_disagreement_deg <= 20.:
        parser.error('Initial yaw disagreement must be finite and in (0,20] degrees')
    route_offsets = None
    if args.route_json:
        route_offsets = json.loads(args.route_json.read_text())
        if (not isinstance(route_offsets, list) or not 1 <= len(route_offsets) <= 8
                or any(not isinstance(xy, list) or len(xy) != 2
                       or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                  and math.isfinite(v) for v in xy)
                       or math.hypot(*xy) > 7. for xy in route_offsets)):
            parser.error('Route must contain 1..8 finite XY offsets within 7 m of the frozen origin')
    geometry_path = os.environ.get('ASTRA_TEST_OBSTACLE_GEOMETRY')
    obstacles = load_obstacles(geometry_path) if geometry_path else None
    rclpy.init()
    node = GuardedPositionTest()
    result = dict(flight_complete=False, fault='', disarmed=False,
                  test_configuration=dict(fixed_enu_ned_transform=args.fixed_enu_ned_transform,
                    max_estimator_xy_disagreement_m=args.max_estimator_xy_disagreement_m,
                    max_tracking_error_m=args.max_tracking_error_m))
    result['test_configuration']['velocity_feedforward'] = args.velocity_feedforward
    result['test_configuration']['freeze_tracking_yaw'] = args.freeze_tracking_yaw
    result['test_configuration']['max_initial_yaw_disagreement_deg'] = args.max_initial_yaw_disagreement_deg
    result['obstacle_geometry'] = obstacles
    max_tracking_error_m = args.max_tracking_error_m
    sim = Parameter('use_sim_time', value=True)
    namespace = Parameter('px4_namespace', value=os.environ.get('ASTRA_PX4_NAMESPACE', 'x500_0'))
    preview = PreviewBspline([sim, namespace, Parameter('max_speed_mps', value=.30),
                              Parameter('max_accel_mps2', value=.35)])
    bridge = GuardedBsplineBridge([sim, namespace, Parameter('enable_px4_output', value=True),
                                   Parameter('velocity_feedforward', value=args.velocity_feedforward),
                                   Parameter('freeze_tracking_yaw', value=args.freeze_tracking_yaw),
                                   Parameter('max_speed_mps', value=.30),
                                   Parameter('max_tracking_error_m', value=max_tracking_error_m),
                                   Parameter('max_estimator_xy_disagreement_m',
                                             value=args.max_estimator_xy_disagreement_m)])
    bridge.output_timer.cancel()
    try:
        if args.calibrate_ground_height:
            node.align_ground_height()
        high = takeoff(node)
        execute(node, high, result, args.goal_distance_m, args.goal_y_offset_m,
                args.obstacle_audit, args.max_tracking_error_m, (preview, bridge), args.single_goal,
                args.fixed_enu_ned_transform, obstacles, route_offsets, args.max_initial_yaw_disagreement_deg)
    except Exception as exc:
        result['fault'] = str(exc)
        node.event('test_aborted', reason=str(exc))
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            node.fault_hold(str(exc))
    finally:
        result['disarmed'] = (node.land() if node.status and
            node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED else
            bool(node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED))
        result['events'] = node.events
        result['ground_height_alignment'] = node.ground_alignment.summary() if node.ground_alignment else None
        Path(args.output_prefix+'_events.json').write_text(json.dumps(result, indent=2)+'\n')
        if node.rows:
            with open(args.output_prefix+'_sent.csv', 'w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.rows[0]))
                writer.writeheader()
                writer.writerows(node.rows)
        print(json.dumps({k:v for k,v in result.items() if k not in
                          ('events','bridge_samples','bsplines')}), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    if not result['flight_complete'] or not result['disarmed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
