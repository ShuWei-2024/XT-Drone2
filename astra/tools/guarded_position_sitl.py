"""Bounded native-NED SITL position test with actual LIVO/map freshness gates."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
from collections import deque
from ground_height_alignment import GroundHeightAlignment

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from px4_settled_hold_flight import SettledFlight


class GuardedPositionTest(SettledFlight):
    def __init__(self):
        # Gazebo transport callbacks can run while the parent constructor is
        # subscribing. Initialize their state before that thread can call us.
        self.truth_body = None
        self.truth_body_previous = None
        self.truth_body_previous_stamp = None
        self.truth_body_velocity = (0.,0.,0.)
        super().__init__()
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.odom = None
        self.pending_odom = None
        self.pending_odom_wall = 0.
        self.deferred_odom_count = 0
        self.odom_wall = self.ready_wall = 0.
        self.ready = False
        self.injecting = False
        self.events, self.rows = [], []
        self.phase = 'preflight'
        self.resets = None
        self.airborne = False
        self.vertical_reference = None
        self.vertical_disagreement_since = None
        self.settle_timeout_s = 60.
        self.sample_reason = ''
        self.local_history = deque(maxlen=100)
        self.ground_alignment = None
        self.mesh_guard = None
        self.mesh_clearance_m = None
        self.mesh_guard_xyz = None
        self.mesh_guard_truth_wall = None
        self.static_obstacles = None
        self.obstacle_clearance_m = None
        if os.environ.get('ASTRA_TEST_OBSTACLE_GEOMETRY'):
            from obstacle_world_geometry_20261010 import load
            self.static_obstacles = load(os.environ['ASTRA_TEST_OBSTACLE_GEOMETRY'])
        if os.environ.get('ASTRA_TEST_MESH_GUARD'):
            from scan_mesh_guard import MeshGuard
            self.mesh_guard = MeshGuard(os.environ['ASTRA_TEST_MESH_GUARD'])
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/local_mapping/ready', self.on_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_odom(self, msg):
        if not self.injecting:
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            now = time.monotonic()
            if stamp_ns > self.get_clock().now().nanoseconds:
                self.deferred_odom_count += 1
                # DDS can deliver odom before its matching /clock callback.
                if self.pending_odom is None:
                    self.pending_odom, self.pending_odom_wall = msg, now
                return
            self.pending_odom = None
            self.odom, self.odom_wall = msg, now

    def on_local(self, msg):
        super().on_local(msg)
        if hasattr(self, 'local_history'):
            self.local_history.append(msg)

    def on_truth(self, msg):
        super().on_truth(msg)
        pose = next((item for item in msg.pose if item.name == 'astra_x500_px4'), None)
        if pose:
            q=pose.orientation
            norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if norm <= 1e-12:
                self.truth_body=None
                return
            x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
            offset=(.48*(x*z+w*y),.48*(y*z-w*x),.24*(1-2*(x*x+y*y)))
            self.truth_body=tuple(a+b for a,b in zip(self.truth,offset))
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nsec
            if self.truth_body_previous_stamp is not None:
                dt=(stamp-self.truth_body_previous_stamp)/1e9
                if 0 < dt <= .25:
                    self.truth_body_velocity=tuple((a-b)/dt for a,b in
                        zip(self.truth_body,self.truth_body_previous))
            self.truth_body_previous=self.truth_body
            self.truth_body_previous_stamp=stamp

    def mesh_fault(self):
        if self.mesh_guard is None:return ''
        if self.truth_body is None:return 'scan guard body reference unavailable'
        if self.mesh_guard_truth_wall!=self.truth_wall:
            self.mesh_guard_xyz=list(self.truth_body)
            self.mesh_clearance_m=self.mesh_guard.clearance(self.truth_body)
            self.mesh_guard_truth_wall=self.truth_wall
        return self.mesh_guard.fault(self.truth_body,self.mesh_clearance_m)

    def static_obstacle_fault(self):
        if not self.static_obstacles:return ''
        if self.truth_body is None:return 'obstacle guard body reference unavailable'
        from obstacle_world_geometry_20261010 import clearances
        self.obstacle_clearance_m=float(clearances([self.truth_body],self.static_obstacles)[0])
        if self.obstacle_clearance_m < .55:
            return f'obstacle clearance below 0.55 m: {self.obstacle_clearance_m:.3f} m'
        return ''

    def paired_height(self):
        if self.odom is None or not self.local_history:
            return None
        s = self.odom.header.stamp
        stamp = s.sec*10**9+s.nanosec
        matched = min(self.local_history, key=lambda p:abs(p.timestamp_sample*1000-stamp))
        if abs(matched.timestamp_sample*1000-stamp) > 20_000_000:
            return None
        return stamp, self.odom.pose.pose.position.z, matched

    def align_ground_height(self):
        self.phase = 'ground_height_alignment'
        alignment = GroundHeightAlignment()
        self.event('ground_height_alignment_start', calibration_s=4., validation_s=4., residual_limit_m=.15)
        deadline = time.monotonic()+25.
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.005)
            if self.fault():
                if alignment.offset_m is not None:
                    raise RuntimeError('health lost after height offset freeze: '+self.fault())
                alignment = GroundHeightAlignment()
                continue
            if math.hypot(*self.truth[:2]) > .5 or not -.2 <= self.truth[2] <= .2:
                raise RuntimeError('Gazebo vehicle not within ground calibration bounds')
            pair = self.paired_height()
            if pair:
                stamp,z,p = pair
                alignment.add(stamp,z,p.z,(p.xy_reset_counter,p.z_reset_counter,p.heading_reset_counter))
            if alignment.error:
                self.event('ground_height_alignment_rejected', **alignment.summary())
                raise RuntimeError(alignment.error)
            if alignment.complete:
                self.ground_alignment = alignment
                self.event('ground_height_alignment_complete', **alignment.summary())
                return
        raise RuntimeError('not enough healthy paired samples for ground height alignment')

    def promote_pending_odom(self):
        if self.pending_odom is not None:
            s = self.pending_odom.header.stamp
            if s.sec * 10**9 + s.nanosec <= self.get_clock().now().nanoseconds:
                self.odom, self.odom_wall = self.pending_odom, self.pending_odom_wall
                self.pending_odom = None

    def send(self, target):
        super().send(target)
        self.sample(target, self.sample_reason)

    def on_ready(self, msg):
        self.ready, self.ready_wall = msg.data, time.monotonic()

    def odom_age(self):
        self.promote_pending_odom()
        if self.odom is None:
            return math.inf
        s = self.odom.header.stamp
        return (self.get_clock().now().nanoseconds - s.sec * 10**9 - s.nanosec) / 1e9

    def fault(self):
        now = time.monotonic()
        self.promote_pending_odom()
        if not self.fresh():
            return self.fresh_failure_reason
        if now - self.local_wall > .25 or now - self.truth_wall > .25:
            return 'PX4 or Gazebo state older than 250 ms'
        if not all(math.isfinite(v) for v in (self.local.x, self.local.y, self.local.z, self.local.heading)):
            return 'non-finite PX4 position'
        bound_radius = float(os.environ.get('ASTRA_TEST_BOUNDS_RADIUS_M', '1.2'))
        if math.hypot(*self.truth[:2]) > bound_radius or not -.2 <= self.truth[2] <= 2.1:
            return 'actual position outside test bounds'
        obstacle_fault=self.static_obstacle_fault()
        if obstacle_fault:return obstacle_fault
        scan_fault=self.mesh_fault()
        if scan_fault:return scan_fault
        if self.mesh_guard is not None:
            scan_fault=self.mesh_guard.approaching_boundary(self.truth_body,self.truth_body_velocity)
            if scan_fault:return scan_fault
        if self.pending_odom is not None:
            s = self.pending_odom.header.stamp
            lead_ns = s.sec * 10**9 + s.nanosec - self.get_clock().now().nanoseconds
            if lead_ns > 20_000_000 or now - self.pending_odom_wall > .02:
                return 'FAST-LIVO2 future stamp did not resolve within 20 ms'
        if self.odom is None or now - self.odom_wall > .25:
            return 'FAST-LIVO2 receipt age exceeds 250 ms'
        if not 0 <= self.odom_age() <= .25:
            return 'FAST-LIVO2 measurement age outside 0..250 ms'
        p = self.odom.pose.pose.position
        if self.odom.header.frame_id != 'camera_init' or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return 'invalid FAST-LIVO2 position'
        if not self.ready or now - self.ready_wall > .25:
            return 'local map not ready or heartbeat stale'
        if self.ground_alignment is not None:
            pair = self.paired_height()
            if pair is None:
                return 'ground-aligned height has no measurement pair within 20 ms'
            _,z,matched = pair
            if abs(matched.z+z-self.ground_alignment.offset_m) > .20:
                return 'PX4/LIVO height drift exceeds frozen ground offset by 0.20 m'
        if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            if self.vertical_reference is not None:
                vertical_disagreement = abs(self.local.z + self.truth[2] - self.vertical_reference)
                if vertical_disagreement > .35:
                    self.vertical_disagreement_since = self.vertical_disagreement_since or now
                    if now - self.vertical_disagreement_since > 1.:
                        return 'PX4 and Gazebo vertical motion disagree by more than 0.35 m for 1 s'
                else:
                    self.vertical_disagreement_since = None
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                return 'PX4 left Offboard'
            reset = (self.local.xy_reset_counter, self.local.z_reset_counter, self.local.heading_reset_counter)
            if self.resets is not None and reset != self.resets:
                if (self.phase in ('climb', 'heading_alignment') and not self.airborne
                        and reset[:2] == self.resets[:2]):
                    self.event('initial_heading_reset', before=self.resets, after=reset)
                    self.resets = reset
                else:
                    return 'PX4 estimator reset'
            if self.airborne and not self.local.heading_good_for_control:
                return 'PX4 heading invalid for control'
        return ''

    def check_safe(self):
        reason = self.fault()
        if reason:
            raise RuntimeError(reason)

    def initial_alignment_fault(self):
        if self.local is None or self.truth is None:
            return 'initial position not available'
        if self.ground_alignment is not None:
            pair = self.paired_height()
            if pair is None:
                return 'no paired ground height'
            _,z,p = pair
            if abs(p.z+z-self.ground_alignment.offset_m) > .15:
                return 'ground height outside 0.15 m frozen-offset residual bounds'
            if math.hypot(*self.truth[:2]) > .5 or not -.2 <= self.truth[2] <= .2:
                return 'initial actual world position outside ground bounds'
            return ''
        if math.hypot(*self.truth[:2]) > .5 or abs(self.local.z + self.truth[2]) > .15:
            return 'initial position/height alignment outside 0.5 m XY / 0.15 m Z bounds'
        return ''

    def stable_hold(self, name, target):
        self.phase = name
        self.event('segment_start', name=name, target=target)
        # Reuse the existing 4-second gate, adding this test's freshness protections.
        self.await_settled_hold(name, target, timeout_s=self.settle_timeout_s, hold_s=4.)
        hold = self.events.pop()
        self.event('stable_hold_complete', **hold)
        self.event('segment_end', name=name)

    def observed_hold(self, name, target):
        """Open-area motion audit: arrival gate, then observe without speed certification."""
        self.phase = name
        self.event('segment_start', name=name, target=target)
        deadline = time.monotonic() + 20.
        arrived_since = observing_since = None
        initial_truth = None
        next_send = 0.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.005)
            self.check_safe()
            now = time.monotonic()
            metrics = self.gate_metrics(target)
            arrived = (metrics['px4_xy_target_error_m'] <= .15
                       and metrics['px4_z_target_error_m'] <= .12)
            if observing_since is None:
                arrived_since = (arrived_since or now) if arrived else None
                if arrived_since is not None and now - arrived_since >= 1.:
                    observing_since = now
                    initial_truth = tuple(self.truth)
                    self.event('observation_started', name=name, target=target,
                               qualified_stable_hold=False)
            else:
                if metrics['px4_xy_target_error_m'] > .20 or metrics['px4_z_target_error_m'] > .15:
                    raise RuntimeError(name + ' left open-area target tracking bounds')
                if math.hypot(self.truth[0] - initial_truth[0],
                              self.truth[1] - initial_truth[1]) > .20:
                    raise RuntimeError(name + ' actual observation drift exceeds 0.20 m')
                if now - observing_since >= 4.:
                    self.event('observation_complete', name=name,
                               qualified_stable_hold=False,
                               truth_delta_enu=[b - a for a, b in zip(initial_truth, self.truth)])
                    self.event('segment_end', name=name)
                    return
            if now >= next_send:
                self.send(target)
                self.gate_rows.append({'hold': name, **metrics})
                next_send = now + .05
        raise RuntimeError(name + ' did not reach open-area arrival bounds within 20 s')

    def event(self, event_name, **values):
        row = dict(event=event_name, wall_ns=time.monotonic_ns(), sim_ns=self.get_clock().now().nanoseconds,
                   odom_age_sec=self.odom_age(),
                   odom_receipt_age_sec=time.monotonic() - self.odom_wall,
                   map_ready=self.ready,
                   heading_good=bool(self.local and self.local.heading_good_for_control),
                   truth=self.truth, px4=None if self.local is None else
                   [self.local.x, self.local.y, self.local.z, self.local.heading], **values)
        self.events.append(row)
        print(json.dumps(row), flush=True)

    def sample(self, target, reason=''):
        self.rows.append(dict(wall_ns=time.monotonic_ns(), sim_ns=self.get_clock().now().nanoseconds,
                              phase=self.phase, odom_age_sec=self.odom_age(),
                              odom_receipt_age_sec=time.monotonic() - self.odom_wall,
                              ready=int(self.ready), reason=reason,
                              deferred_odom_count=self.deferred_odom_count,
                              truth_x=self.truth[0], truth_y=self.truth[1], truth_z=self.truth[2],
                              px4_x=self.local.x, px4_y=self.local.y, px4_z=self.local.z,
                              px4_vx=self.local.vx, px4_vy=self.local.vy, px4_vz=self.local.vz,
                              target_x=target[0], target_y=target[1], target_z=target[2]))
        self.rows[-1]['mesh_clearance_m']=self.mesh_clearance_m

    def guarded_segment(self, name, seconds, start, end):
        self.phase = name
        self.event('segment_start', name=name, target=end)
        begin = time.monotonic()
        next_send = begin
        while rclpy.ok() and time.monotonic() - begin < seconds:
            rclpy.spin_once(self, timeout_sec=.005)
            reason = self.fault()
            if reason:
                raise RuntimeError(reason)
            now = time.monotonic()
            if now >= next_send:
                fraction = min(1., (now - begin) / seconds)
                target = tuple(a + fraction * (b - a) for a, b in zip(start, end))
                self.send(target)
                next_send = now + .05
        self.event('segment_end', name=name)

    def fault_hold(self, reason):
        self.phase = 'fault_hold'
        self.sample_reason = reason
        self.event('fault_latched', reason=reason)
        if (time.monotonic() - self.local_wall > .25 or not self.local.xy_valid or
                not self.local.z_valid or not self.local.heading_good_for_control):
            return
        hold = (self.local.x, self.local.y, self.local.z, self.local.heading)
        self.event('hold_target_frozen', target=hold)
        end = time.monotonic() + 1.
        next_send = 0.
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.005)
            scan_fault = self.mesh_fault()
            if scan_fault:
                self.event('recovery_mesh_fault', reason=scan_fault, mesh_clearance_m=self.mesh_clearance_m)
                break
            if (time.monotonic() - self.local_wall > .25 or not self.local.xy_valid or
                    not self.local.z_valid or not self.local.heading_good_for_control or
                    self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                break
            if time.monotonic() >= next_send:
                self.send(hold)
                next_send = time.monotonic() + .05

    def wait_heading_alignment(self, target):
        self.phase = 'heading_alignment'
        self.event('heading_alignment_wait', target=target)
        deadline = time.monotonic() + 18.
        good_since = None
        next_send = 0.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.005)
            reason = self.fault()
            if reason:
                raise RuntimeError(reason)
            now = time.monotonic()
            if now >= next_send:
                self.send(target)
                next_send = now + .05
            if self.local.heading_good_for_control and self.truth[2] > 1.5:
                good_since = good_since or now
                if now-good_since >= 1.:
                    self.airborne = True
                    self.event('heading_alignment_complete')
                    return
            else:
                good_since = None
        raise RuntimeError('PX4 in-flight heading alignment did not complete')

    def land(self):
        self.phase = 'land'
        self.event('land_requested')
        deadline = time.monotonic() + 30.
        next_request = 0.
        previous_scan_fault = None
        next_scan_sample = 0.
        while rclpy.ok() and time.monotonic() < deadline:
            if self.status and self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.event('disarmed')
                return True
            if time.monotonic() >= next_request:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                next_request = time.monotonic() + .5
            rclpy.spin_once(self, timeout_sec=.02)
            if (self.mesh_guard is not None or self.static_obstacles) and time.monotonic() >= next_scan_sample:
                scan_fault = self.mesh_fault() or self.static_obstacle_fault()
                self.event('landing_mesh_sample', reason=scan_fault,
                           mesh_guard_xyz=self.mesh_guard_xyz, mesh_clearance_m=self.mesh_clearance_m,
                           truth_body=self.truth_body, obstacle_clearance_m=self.obstacle_clearance_m)
                if scan_fault and scan_fault != previous_scan_fault:
                    self.event('recovery_mesh_fault', reason=scan_fault, mesh_clearance_m=self.mesh_clearance_m)
                previous_scan_fault = scan_fault
                next_scan_sample = time.monotonic() + .1
        self.event('landing_confirmation_timeout')
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-prefix', required=True)
    parser.add_argument('--inject-stale', action='store_true')
    parser.add_argument('--require-stable-hold', action='store_true')
    parser.add_argument('--open-motion-audit', action='store_true')
    parser.add_argument('--calibrate-ground-height',action='store_true')
    parser.add_argument('--segment-distance-m', type=float, default=.25)
    parser.add_argument('--settle-timeout-s', type=float, default=60.)
    args = parser.parse_args()
    if args.require_stable_hold and args.open_motion_audit:
        parser.error('stable certification and open motion audit are separate modes')
    if not .1 <= args.segment_distance_m <= .4:
        parser.error('--segment-distance-m must be between 0.1 and 0.4')
    if not 20. <= args.settle_timeout_s <= 60.:
        parser.error('--settle-timeout-s must be between 20 and 60')
    rclpy.init()
    node = GuardedPositionTest()
    node.settle_timeout_s = args.settle_timeout_s
    result = dict(route_complete=False, fault='', injection_requested=args.inject_stale,
                  injection_started=False, disarmed=False,
                  test_mode='open_motion_audit' if args.open_motion_audit else
                  'stable_hold' if args.require_stable_hold else 'native_motion')
    try:
        if args.calibrate_ground_height:
            node.align_ground_height()
        deadline = time.monotonic() + 25.
        good_since = None
        last_preflight_reason = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            preflight_reason = node.fault() or node.initial_alignment_fault()
            if not preflight_reason:
                good_since = good_since or time.monotonic()
                if time.monotonic() - good_since >= 2.:
                    break
            else:
                if preflight_reason != last_preflight_reason:
                    node.event('preflight_health_lost', reason=preflight_reason)
                    last_preflight_reason = preflight_reason
                good_since = None
        else:
            raise RuntimeError('preflight did not remain healthy for 2 s: ' + (node.fault() or node.initial_alignment_fault()))
        p = node.local
        origin = (p.x, p.y, p.z, p.heading)
        node.vertical_reference = p.z + node.truth[2]
        if node.initial_alignment_fault():
            raise RuntimeError('initial position/height alignment invalid')
        node.guarded_segment('prestream', 2., origin, origin)
        node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
        node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            reason = node.fault()
            if reason:
                raise RuntimeError(reason)
            node.send(origin)
            rclpy.spin_once(node, timeout_sec=.05)
            if node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and node.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                break
        else:
            raise RuntimeError('PX4 rejected Offboard/arming')
        node.resets = (node.local.xy_reset_counter, node.local.z_reset_counter, node.local.heading_reset_counter)
        high = (*origin[:2], origin[2] - 1.7, origin[3])
        north = (high[0] + args.segment_distance_m, high[1], high[2], high[3])
        east = (north[0], north[1] + args.segment_distance_m, high[2], high[3])
        node.guarded_segment('climb', 16., origin, high)
        node.wait_heading_alignment(high)
        move_s = 5. if args.require_stable_hold or args.open_motion_audit else 3.
        if args.require_stable_hold:
            node.stable_hold('settle', high)
        elif args.open_motion_audit:
            node.observed_hold('settle', high)
        else:
            node.guarded_segment('settle', 6., high, high)
        node.guarded_segment('north', move_s, high, north)
        if args.require_stable_hold:
            node.stable_hold('hold_north', north)
        elif args.open_motion_audit:
            node.observed_hold('hold_north', north)
        else:
            node.guarded_segment('hold_north', 3., north, north)
        node.guarded_segment('east', move_s, north, east)
        if args.require_stable_hold:
            node.stable_hold('hold_east', east)
        elif args.open_motion_audit:
            node.observed_hold('hold_east', east)
        else:
            node.guarded_segment('hold_east', 3., east, east)
        if args.inject_stale:
            node.injecting = True
            result['injection_started'] = True
            node.event('odom_consumption_paused')
            node.guarded_segment('injected_hold', 2., east, east)
            raise RuntimeError('fault injection did not trip freshness gate')
        node.guarded_segment('return', 5., east, high)
        if args.require_stable_hold:
            node.stable_hold('hold_final', high)
        elif args.open_motion_audit:
            node.observed_hold('hold_final', high)
        else:
            node.guarded_segment('hold_final', 3., high, high)
        result['route_complete'] = True
    except Exception as exc:
        result['fault'] = str(exc)
        node.event('test_aborted', reason=str(exc))
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            node.fault_hold(str(exc))
    finally:
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            result['disarmed'] = node.land()
        else:
            result['disarmed'] = bool(node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED)
        prefix = Path(args.output_prefix)
        result['deferred_odom_count'] = node.deferred_odom_count
        result['ground_height_alignment'] = node.ground_alignment.summary() if node.ground_alignment else None
        result['setpoint_recording'] = 'all_send_calls'
        Path(str(prefix) + '_events.json').write_text(json.dumps(dict(result=result, events=node.events), indent=2) + '\n')
        if node.rows:
            with Path(str(prefix) + '_sent.csv').open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.rows[0]))
                writer.writeheader()
                writer.writerows(node.rows)
        if node.gate_rows:
            with Path(str(prefix) + '_gate.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.gate_rows[0]))
                writer.writeheader()
                writer.writerows(node.gate_rows)
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    if not result['disarmed'] or (not result['route_complete'] and not result['injection_started']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
