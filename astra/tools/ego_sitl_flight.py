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
from rclpy.parameter import Parameter
from ready_batch_executor import ReadyBatchExecutor, BoundedEventsExecutor, receive_ready_batch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/astra_px4_offboard/scripts'))
from guarded_position_sitl import GuardedPositionTest
from guarded_bspline_bridge import GuardedBsplineBridge
from preview_bspline import PreviewBspline, stamp_ns


def receive_batch(executor):
    # A /clock callback can precede already queued odometry after a scheduling
    # pause. Process a bounded batch before checking freshness, so the guard
    # sees received measurements rather than only the first ready callback.
    audit=getattr(executor,'timing_audit',None)
    if audit:
        from executor_timing_audit import scheduler_wait_ns
        wall=time.monotonic_ns();cpu=time.thread_time_ns()
        wait=scheduler_wait_ns()
    batch=receive_ready_batch(executor)
    if audit:
        finish_wait=scheduler_wait_ns()
        audit.batch_tail.append(dict(wall_start_ns=wall,wall_end_ns=time.monotonic_ns(),
            wall_duration_ns=time.monotonic_ns()-wall,cpu_duration_ns=time.thread_time_ns()-cpu,
            scheduler_wait_ns=None if wait is None or finish_wait is None else finish_wait-wait,
            **batch))


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
    node.ground_route_origin_livo = (node.odom.pose.pose.position.x, node.odom.pose.pose.position.y)
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
    executor = (BoundedEventsExecutor() if result['test_configuration'].get('execution_executor') == 'events'
                else ReadyBatchExecutor())
    for n in (node, preview, bridge):
        executor.add_node(n)
    goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
    bridge_rows = []
    raw_splines = []
    from traj_utils.msg import Bspline
    def record_spline(msg):
        raw_splines.append(dict(start_ns=stamp_ns(msg.start_time), traj_id=msg.traj_id, order=msg.order,
                                knots=list(msg.knots),
                                controls=[[p.x, p.y, p.z] for p in msg.pos_pts]))
    subscription = node.create_subscription(Bspline, '/planning/bspline', record_spline, 10)
    audit_path=result.get('executor_audit_path')
    if audit_path:
        from executor_timing_audit import ExecutorTimingAudit
        executor.timing_audit=ExecutorTimingAudit()
        for n in (node,preview,bridge):
            executor.timing_audit.install(n,with_metadata=not isinstance(executor,BoundedEventsExecutor))
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
        if node.mesh_guard is not None:
            origin_livo = (*node.ground_route_origin_livo, p.z)
            node.event('indoor_route_origin_frozen', origin_livo=list(origin_livo),
                       xy_reference='pre-takeoff FAST-LIVO2; no Gazebo calibration input')
        goal_queue = ([(3.0, 3.0), (goal_distance_m, goal_y_offset_m)]
                      if obstacle_audit and not single_goal else [(goal_distance_m, goal_y_offset_m)])
        if route_offsets is not None:
            goal_queue = route_offsets
        result['route_offsets_m'] = goal_queue
        result['origin_livo'] = list(origin_livo)
        goal_index = 0
        target = None
        goal_sent_ns = None
        terminal_spline = None

        def publish_goal(offset_x, offset_y):
            nonlocal target, goal_sent_ns, terminal_spline
            terminal_spline = None
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
                if (not bridge.health() and preview.trajectory_start_ns is not None
                        and preview.trajectory_start_ns >= goal_sent_ns):
                    handed = True
                    node.phase = 'ego_execution'
                    result.setdefault('truth_start', list(node.truth))
                    node.event('ego_control_handoff' if goal_index == 0 else 'ego_leg_control_handoff', goal_index=goal_index)
                    next_tick = now
                continue
            if (preview.span is not None and preview.trajectory_start_ns is not None
                    and preview.trajectory_start_ns >= goal_sent_ns and preview.spline is not None
                    and preview.valid):
                final_sim_ns = preview.trajectory_start_ns + int((preview.span[1]-preview.span[0])*1e9)
                terminal_spline = preview.spline
                terminal_time = preview.span[1]
                terminal_id = preview.trajectory_id
            # Transfer exclusive output ownership only after a normally completed,
            # accepted stop trajectory. Fault holds never count as goal completion.
            if (final_sim_ns is not None and
                    node.get_clock().now().nanoseconds >= final_sim_ns-20_000_000 and
                    preview.ready and preview.fresh(preview.planner_wall, .25) and
                    terminal_spline is not None and
                    (preview.valid or preview.completed_trajectory_id == terminal_id)):
                terminal = terminal_spline(terminal_time)
                if (np.linalg.norm(bridge.transform(terminal)-target) > .03 or
                        np.linalg.norm(terminal_spline.derivative()(terminal_time)) > .02):
                    raise RuntimeError('trajectory terminal is not a stopped goal')
                node.event('ego_endpoint_settling_start', goal_index=goal_index)
                # The bridge timer remains cancelled; this supervisor alone sends
                # the fixed endpoint until arrival or a bounded timeout/fault.
                settle_deadline = time.monotonic()+8.
                stable_since = None
                next_endpoint_send = 0.
                while time.monotonic() < settle_deadline:
                    receive_batch(executor)
                    node.check_safe()
                    fault = bridge.state_health()
                    if (not preview.valid and preview.completed_trajectory_id != terminal_id and
                            node.get_clock().now().nanoseconds > final_sim_ns+250_000_000):
                        raise RuntimeError('trajectory revoked without normal completion')
                    if fault or bridge.fault_since is not None:
                        raise RuntimeError('endpoint state fault: '+(fault or bridge.fault_reason))
                    now = time.monotonic()
                    error = float(np.linalg.norm(target-[node.local.x,node.local.y,node.local.z]))
                    speed = float(np.linalg.norm([node.local.vx,node.local.vy,node.local.vz]))
                    if not math.isfinite(speed) or error > max_tracking_error_m:
                        raise RuntimeError('endpoint speed invalid or tracking error exceeded')
                    if now >= next_endpoint_send:
                        yaw = bridge.tracking_yaw if bridge.tracking_yaw is not None else node.local.heading
                        node.send((*target,yaw))
                        next_endpoint_send = now+.05
                    stable_since = (stable_since or now) if error <= .045 and speed <= .08 else None
                    if stable_since is not None and now-stable_since >= .5:
                        node.event('ego_endpoint_settled', goal_index=goal_index,
                                   target_error_m=error, speed_mps=speed, stable_sec=now-stable_since)
                        break
                else:
                    raise RuntimeError('endpoint did not settle within 8 seconds')
                if goal_index + 1 < len(goal_queue):
                    node.event('ego_waypoint_reached', goal_index=goal_index)
                    # publish_goal replaces target with the NEXT destination.
                    # While waiting for its fresh spline, keep the completed
                    # endpoint; sending the next destination here bypasses EGO.
                    high = (*target, yaw)
                    goal_index += 1
                    publish_goal(*goal_queue[goal_index])
                    final_sim_ns = None
                    # Resume only on a fresh spline for the new goal; hold this
                    # completed endpoint while the planner generates it.
                    handed = False
                    bridge.preview_gap_since = None
                    bridge.replan_hold = None
                    deadline = now + leg_timeout
                    continue
                node.event('ego_waypoint_reached', goal_index=goal_index)
                node.event('ego_trajectory_sampled_to_end')
                result['trajectory_sampled_to_end'] = True
                break
            if now >= next_tick:
                bridge.tick()
                sample_wall=time.monotonic_ns()
                odom_stamp=None if node.odom is None else stamp_ns(node.odom.header.stamp)
                bridge_rows.append(dict(sim_ns=node.get_clock().now().nanoseconds,
                    wall_ns=sample_wall, state=bridge.mode, truth=list(node.truth), goal_index=goal_index,
                    supervisor_odom_measurement_ns=odom_stamp,
                    supervisor_odom_receipt_monotonic_ns=int(node.odom_wall*1e9),
                    supervisor_odom_receipt_age_sec=sample_wall/1e9-node.odom_wall,
                    supervisor_odom_measurement_age_sec=None if odom_stamp is None else
                        (node.get_clock().now().nanoseconds-odom_stamp)/1e9,
                    bridge_odom_receipt_age_sec=None if bridge.odom_wall is None else
                        sample_wall/1e9-bridge.odom_wall,
                    deferred_supervisor_odom_count=node.deferred_odom_count,
                    preview_trajectory_id=preview.trajectory_id,
                    preview_trajectory_start_ns=preview.trajectory_start_ns,
                    preview_pose_stamp_ns=None if bridge.pose is None else stamp_ns(bridge.pose.header.stamp),
                    tracking_yaw_ned_rad=bridge.tracking_yaw,
                    mesh_guard_xyz=node.mesh_guard_xyz,
                    mesh_clearance_m=node.mesh_clearance_m,
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
                    if node.mesh_guard is not None:
                        clearance=node.mesh_clearance_m
                    elif obstacles:
                        clearance=node.obstacle_clearance_m
                    else:
                        clearance=min(math.hypot(max(x-xmax,0.,xmin-x),
                                                max(y-ymax,0.,ymin-y))
                                      for xmin,xmax,ymin,ymax in rectangles)
                    result.setdefault('obstacle_clearance_samples_m', []).append(clearance)
                    if clearance < .55:
                        raise RuntimeError(f'obstacle clearance below 0.55 m: {clearance:.3f} m')
                if bridge.fault_since is not None:
                    raise RuntimeError('bridge fault: '+bridge.fault_reason)
                next_tick = now+.05
        else:
            raise RuntimeError('EGO trajectory did not finish before its bounded deadline; '+bridge.health())
    except Exception as exc:
        # Preserve the detection instant before ROS entity teardown. DDS
        # destruction can take >100 ms; the later main-level abort event alone
        # would overstate receipt age at the actual guard decision.
        result['execution_fault_detection'] = dict(
            wall_ns=time.monotonic_ns(),sim_ns=node.get_clock().now().nanoseconds,
            reason=str(exc),odom_measurement_age_sec=node.odom_age(),
            odom_receipt_age_sec=time.monotonic()-node.odom_wall)
        raise
    finally:
        if audit_path:
            executor.timing_audit.restore()
            # Serializing the trace can take hundreds of milliseconds. Keep it
            # in memory until after disarm; never block active flight for I/O.
            node.completed_executor_audit=executor.timing_audit
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
    parser.add_argument('--execution-executor', choices=('ready-batch', 'events'), default='ready-batch',
                        help='Optional experimental Jazzy event executor for isolated SITL')
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
    parser.add_argument('--executor-audit',action='store_true')
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
    result['test_configuration']['execution_executor'] = args.execution_executor
    result['test_configuration']['freeze_tracking_yaw'] = args.freeze_tracking_yaw
    result['test_configuration']['max_initial_yaw_disagreement_deg'] = args.max_initial_yaw_disagreement_deg
    if args.executor_audit:result['executor_audit_path']=args.output_prefix+'_executor_audit.json'
    result['obstacle_geometry'] = obstacles
    result['mesh_guard_configuration'] = (node.mesh_guard.metadata if node.mesh_guard is not None else None)
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
        if args.executor_audit and hasattr(node,'completed_executor_audit'):
            node.completed_executor_audit.save(result['executor_audit_path'])
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
