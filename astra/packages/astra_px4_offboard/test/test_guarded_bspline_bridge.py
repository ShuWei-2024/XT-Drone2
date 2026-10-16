"""Focused SITL bridge checks without a live PX4 or planner process."""

import math
import sys
import time
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
import numpy as np
from scipy.interpolate import BSpline
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool, Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from guarded_bspline_bridge import GuardedBsplineBridge  # noqa: E402
from preview_bspline import PreviewBspline  # noqa: E402


class PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class BridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GuardedBsplineBridge()
        stamp = self.node.get_clock().now().to_msg()
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        pose = PoseStamped()
        pose.header.frame_id = 'camera_init'
        pose.header.stamp = stamp
        pose.pose.orientation.w = 1.0
        twist = TwistStamped()
        twist.header = pose.header
        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose.orientation.w = 1.0
        px4 = VehicleLocalPosition()
        px4.timestamp = ns // 1000
        px4.timestamp_sample = ns // 1000
        px4.x, px4.y = map(float, self.node.translation)
        px4.z = self.node.z_translation
        px4.xy_valid = px4.z_valid = px4.heading_good_for_control = True
        px4.heading = math.atan2(
            self.node.matrix[1, 0], self.node.matrix[0, 0])
        self.node.on_pose(pose)
        self.node.on_velocity(twist)
        self.node.on_odom(odom)
        self.node.on_px4(px4)
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node.on_status(status)
        self.node.on_mode(String(data='TRACK_PREVIEW'))
        self.node.on_ready(Bool(data=True))
        self.node.on_planner_heartbeat(Empty())

    def tearDown(self):
        self.node.destroy_node()

    def test_serial_preview_refresh_after_delayed_timer_keeps_original_gates(self):
        preview = PreviewBspline()
        preview.output_timer.cancel()
        preview.sample_callback = self.node.on_preview_sample
        preview.on_ready(Bool(data=True))
        preview.on_valid(Bool(data=True))
        preview.on_planner_heartbeat(Empty())
        preview.on_odom(self.node.odom)
        preview.spline = BSpline(np.arange(-3., 7.), np.zeros((6, 3)), 3)
        preview.span = (0., 3.)
        common_time = Time(nanoseconds=self.node.get_clock().now().nanoseconds+12_000_000)
        self.node.control_clock = preview.control_clock = SimpleNamespace(now=lambda: common_time)
        preview.trajectory_start_ns = common_time.nanoseconds-10_000_000
        try:
            # The timer hasn't run, but all actual inputs remain healthy.
            self.node.mode_wall = time.monotonic()-.21
            self.assertEqual('preview not tracking', self.node.health())
            preview.tick()
            self.assertEqual('', self.node.health())
            self.assertEqual('tracking', preview.reason)
            self.assertEqual(self.node.pose.header.stamp, self.node.velocity.header.stamp)
            self.assertEqual(common_time.to_msg(), self.node.pose.header.stamp)
            # Strict future-stamp rejection remains; do not add tolerance.
            self.node.pose.header.stamp = Time(nanoseconds=common_time.nanoseconds+1).to_msg()
            self.assertEqual('stale measurement timestamp', self.node.health())
            # A source fault must still revoke tracking in the same call.
            preview.planner_wall = time.monotonic()-.4
            preview.tick()
            self.assertIn('planner heartbeat stale', preview.reason)
            self.assertEqual('preview not tracking', self.node.health())
            preview.on_planner_heartbeat(Empty())
            preview.odom_wall = time.monotonic()-.3
            preview.tick()
            self.assertIn('odometry stale or invalid', preview.reason)
            self.assertEqual('NO_VALID_HOLD', self.node.preview_mode)
            preview.on_odom(self.node.odom)
            preview.on_valid(Bool(data=False))
            preview.tick()
            self.assertIn('trajectory invalid', preview.reason)
            self.assertNotEqual('TRACK_PREVIEW', self.node.preview_mode)
        finally:
            preview.destroy_node()

    def test_composed_bridge_cannot_receive_old_preview_dds_callbacks(self):
        node = GuardedBsplineBridge(subscribe_preview=False)
        try:
            topics = {sub.topic_name for sub in node.subscriptions}
            self.assertNotIn('/planning/preview_mode', topics)
            self.assertNotIn('/planning/position_preview', topics)
            self.assertNotIn('/planning/velocity_preview', topics)
            self.assertIn('/fast_livo2/odom', topics)
        finally:
            node.destroy_node()

    def test_dry_run_publishes_no_px4_input_and_transforms_position(self):
        self.assertEqual('', self.node.health())
        self.node.tick()
        self.assertEqual('TRACK_DRY_RUN', self.node.mode)
        self.assertFalse(hasattr(self.node, 'setpoint_pub'))
        self.assertFalse(hasattr(self.node, 'command_pub'))

    def test_fault_freezes_px4_pose_and_then_requests_land(self):
        mode, setpoints, commands = PublisherSpy(), PublisherSpy(), PublisherSpy()
        self.node.enabled = True
        self.node.mode_pub = mode
        self.node.setpoint_pub = setpoints
        self.node.command_pub = commands
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.tick()
        self.assertEqual('TRACK_SITL', self.node.mode)
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.preview_gap_since = time.monotonic() - .41
        self.node.tick()
        self.assertEqual('FAULT_LATCHED: preview not tracking', self.node.mode)
        for expected, actual in zip(self.node.frozen_hold[:3],
                                    setpoints.messages[-1].position):
            self.assertAlmostEqual(expected, actual, places=6)
        self.node.limits['max_hold_sec'] = 0.001
        self.node.fault_since = time.monotonic() - 0.01
        self.node.tick()
        self.assertEqual(1, len(commands.messages))
        self.assertEqual(commands.messages[0].VEHICLE_CMD_NAV_LAND,
                         commands.messages[0].command)
        self.node.tick()
        self.assertEqual(1, len(commands.messages))
        self.node.last_land_request = time.monotonic() - 0.6
        self.node.tick()
        self.assertEqual(2, len(commands.messages))

    def test_preflight_streaming_and_offboard_loss(self):
        modes, setpoints = PublisherSpy(), PublisherSpy()
        self.node.enabled = True
        self.node.mode_pub = modes
        self.node.setpoint_pub = setpoints
        self.node.offboard_seen = False
        status = VehicleStatus()
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.node.on_status(status)
        self.node.tick()
        self.assertEqual('TRACK_SITL', self.node.mode)
        self.assertEqual(1, len(modes.messages))
        self.assertEqual(1, len(setpoints.messages))
        self.assertGreater(setpoints.messages[0].timestamp, 0)
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node.on_status(status)
        status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.node.on_status(status)
        self.assertEqual('PX4 left Offboard', self.node.health())

    def test_estimator_reset_latches_fault(self):
        self.assertEqual('', self.node.health())
        self.node.px4.xy_reset_counter += 1
        self.assertEqual('PX4 estimator reset', self.node.health())

    def test_estimator_comparison_uses_measurement_matched_history(self):
        self.node.limits['max_estimator_xy_disagreement_m'] = .05
        latest = deepcopy(self.node.px4)
        latest.timestamp_sample += 10_000
        latest.x += .15
        self.node.on_px4(latest)
        # The latest pose is used for tracking; the original paired pose is
        # used for estimator disagreement. Neither exceeds its own limit.
        self.assertEqual('', self.node.health())

    def test_disagreement_reports_the_measured_component(self):
        self.node.px4.x += .31
        reason = self.node.health()
        self.assertIn('FAST-LIVO2 and PX4 disagree', reason)
        self.assertIn('xy=0.310m', reason)
        self.node.px4.x -= .31
        self.node.px4.z += .21
        self.assertIn('z=0.210m', self.node.health())

    def test_single_missing_pair_uses_fresh_history_and_expires(self):
        source=deepcopy(self.node.odom)
        epoch=self.node.get_clock().now().nanoseconds-30_000_000
        source.header.stamp.sec,source.header.stamp.nanosec=divmod(epoch,10**9)
        # Latest source has no PX4 sample within 20 ms, preceding real pair does.
        self.node.on_odom(source)
        self.assertEqual('',self.node.health())
        self.assertLessEqual(self.node.estimator_pair_dt_sec,.02)
        for msg,_ in self.node.odom_history:
            t=self.node.get_clock().now().nanoseconds-260_000_000
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(t,10**9)
        # Keep a current source, with only unmatched PX4 samples in history.
        fresh=deepcopy(source);fresh.header.stamp=self.node.get_clock().now().to_msg()
        for msg in self.node.px4_history:msg.timestamp_sample-=100_000
        self.node.on_odom(fresh)
        self.assertEqual('estimator measurement times differ',self.node.state_health())

    def test_fresh_history_does_not_hide_latest_invalid_position(self):
        source=deepcopy(self.node.odom);source.pose.pose.position.x=math.nan
        self.node.on_odom(source)
        self.assertEqual('non-finite estimator position',self.node.health())

    def test_expired_nearest_sample_does_not_hide_valid_measurement_pair(self):
        now=self.node.get_clock().now().nanoseconds
        source=deepcopy(self.node.odom)
        source.header.stamp.sec,source.header.stamp.nanosec=divmod(now-244_000_000,10**9)
        expired,eligible=deepcopy(self.node.px4),deepcopy(self.node.px4)
        expired.timestamp_sample=(now-252_000_000)//1000  # nearer, but expired
        eligible.timestamp_sample=(now-232_000_000)//1000  # 12 ms apart, both fresh
        self.node.odom_history.clear();self.node.px4_history.clear()
        self.node.on_odom(source);self.node.on_px4(expired);self.node.on_px4(eligible)
        with patch.object(self.node,'control_clock') as clock:
            clock.now.return_value.nanoseconds=now
            self.assertEqual('',self.node.state_health())
            self.assertLessEqual(self.node.estimator_pair_dt_sec,.02)
            self.assertLessEqual(self.node.estimator_pair_age_sec,.25)
            # An ineligible nearest sample remains unusable when no good pair exists.
            self.node.px4_history.pop()
            self.assertEqual('estimator measurement times differ',self.node.state_health())

    def test_separate_preview_callbacks_commit_only_matching_pairs(self):
        preceding = self.node.pose
        pose, twist = deepcopy(self.node.pose), deepcopy(self.node.velocity)
        now = self.node.get_clock().now().nanoseconds
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(now, 10**9)
        twist.header = deepcopy(pose.header)
        self.node.on_pose(pose)
        self.assertIs(preceding, self.node.pose)
        self.assertEqual('', self.node.health())
        self.node.on_velocity(twist)
        self.assertIs(pose, self.node.pose)
        self.assertEqual('', self.node.health())
        pose = deepcopy(pose)
        pose.header.stamp.nanosec += 1
        self.node.on_pose(pose)
        self.node.pose_wall = time.monotonic() - .21
        self.assertEqual('stale trajectory preview', self.node.health())

    def test_stale_px4_pose_requests_land_without_stale_hold(self):
        mode, setpoints, commands = PublisherSpy(), PublisherSpy(), PublisherSpy()
        self.node.enabled = True
        self.node.mode_pub = mode
        self.node.setpoint_pub = setpoints
        self.node.command_pub = commands
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.tick()
        self.node.px4_wall = time.monotonic() - 1.0
        self.node.tick()
        self.assertIsNone(self.node.frozen_hold)
        self.assertEqual(1, len(commands.messages))

    def test_invalid_px4_pose_is_not_used_as_hold(self):
        mode, setpoints, commands = PublisherSpy(), PublisherSpy(), PublisherSpy()
        self.node.enabled = True
        self.node.mode_pub = mode
        self.node.setpoint_pub = setpoints
        self.node.command_pub = commands
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.tick()
        self.node.px4.x = math.nan
        self.node.tick()
        self.assertIsNone(self.node.frozen_hold)
        self.assertEqual(1, len(setpoints.messages))
        self.assertEqual(1, len(commands.messages))

    def test_short_replan_gap_does_not_latch_fault(self):
        mode, setpoints = PublisherSpy(), PublisherSpy()
        self.node.enabled = True
        self.node.mode_pub = mode
        self.node.setpoint_pub = setpoints
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.on_planner_heartbeat(Empty())
        self.node.tick()
        self.assertIsNone(self.node.fault_since)
        self.assertEqual('WAITING_REPLAN', self.node.mode)

    def test_replan_hold_is_frozen_while_estimated_position_moves(self):
        self.node.enabled = True
        self.node.mode_pub = PublisherSpy()
        self.node.setpoint_pub = PublisherSpy()
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.tick()
        initial = list(self.node.setpoint_pub.messages[-1].position)
        self.node.px4.x += .02
        self.node.tick()
        self.assertEqual('WAITING_REPLAN', self.node.mode)
        self.assertEqual(initial, list(self.node.setpoint_pub.messages[-1].position))

    def test_replan_wait_does_not_hide_state_faults(self):
        for name, mutate, expected in (
            ('odom', lambda: setattr(self.node, 'odom_wall', time.monotonic() - .3),
             'stale FAST-LIVO2 odometry'),
            ('map', lambda: self.node.on_ready(Bool(data=False)), 'local map not ready'),
            ('map heartbeat', lambda: setattr(self.node, 'ready_wall', time.monotonic() - .3),
             'stale local map heartbeat'),
            ('planner', lambda: setattr(self.node, 'planner_heartbeat_wall', time.monotonic() - .3),
             'stale planner heartbeat'),
            ('reset', lambda: setattr(self.node.px4, 'xy_reset_counter', 1), 'PX4 estimator reset'),
        ):
            with self.subTest(name=name):
                now = time.monotonic()
                self.node.odom_wall = self.node.px4_wall = self.node.status_wall = now
                self.node.on_ready(Bool(data=True))
                self.node.on_planner_heartbeat(Empty())
                self.node.px4.xy_reset_counter = 0
                self.node.fault_since = None
                self.node.tracked = True
                self.node.reset_counters = (0, 0, 0)
                self.node.on_mode(String(data='HOLD_PREVIEW'))
                mutate()
                self.node.tick()
                self.assertEqual(expected, self.node.fault_reason)
                self.assertEqual('FAULT_LATCHED: ' + expected, self.node.mode)

    def test_persistent_replan_gap_latches_fault(self):
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.tick()
        self.node.preview_gap_since = time.monotonic() - .41
        self.node.tick()
        self.assertEqual('FAULT_LATCHED: preview not tracking', self.node.mode)

    def test_tracking_resume_releases_replan_hold(self):
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.tick()
        self.assertIsNotNone(self.node.replan_hold)
        self.node.on_mode(String(data='TRACK_PREVIEW'))
        self.node.tick()
        self.assertEqual('TRACK_DRY_RUN', self.node.mode)
        self.assertIsNone(self.node.replan_hold)
        self.assertIsNone(self.node.preview_gap_since)

    def test_nonfinite_heading_is_rejected(self):
        self.node.px4.heading = math.nan
        self.assertEqual('non-finite estimator heading', self.node.health())

    def test_optional_velocity_feedforward_uses_ned_axes_and_clears_for_hold(self):
        self.node.velocity_feedforward = True
        self.node.velocity.twist.linear.x = .10
        self.node.velocity.twist.linear.y = .05
        self.node.velocity.twist.linear.z = .03
        self.node.preview_pub = PublisherSpy()
        self.node.tick()
        v = self.node.preview_pub.messages[-1].velocity
        self.assertAlmostEqual(v[0], self.node.matrix[0, 0] * .10 + self.node.matrix[0, 1] * .05)
        self.assertAlmostEqual(v[1], self.node.matrix[1, 0] * .10 + self.node.matrix[1, 1] * .05)
        self.assertAlmostEqual(v[2], -.03)
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.tick()
        self.assertTrue(all(math.isnan(v) for v in self.node.preview_pub.messages[-1].velocity))

    def test_position_only_remains_default(self):
        self.node.preview_pub = PublisherSpy()
        self.node.tick()
        self.assertTrue(all(math.isnan(v) for v in self.node.preview_pub.messages[-1].velocity))

    def test_frozen_yaw_survives_estimate_changes_and_replan(self):
        self.node.freeze_tracking_yaw = True
        self.node.preview_pub = PublisherSpy()
        self.node.tick()
        initial = self.node.preview_pub.messages[-1].yaw
        self.node.px4.heading += .02
        self.node.tick()
        self.assertAlmostEqual(initial, self.node.preview_pub.messages[-1].yaw)
        self.node.tracked = True
        self.node.on_mode(String(data='HOLD_PREVIEW'))
        self.node.tick()
        self.assertEqual('WAITING_REPLAN', self.node.mode)
        self.assertAlmostEqual(initial, self.node.preview_pub.messages[-1].yaw)
        self.node.on_mode(String(data='TRACK_PREVIEW'))
        self.node.tick()
        self.assertAlmostEqual(initial, self.node.preview_pub.messages[-1].yaw)
        self.node.px4.heading_reset_counter += 1
        self.node.tick()
        self.assertEqual('FAULT_LATCHED: PX4 estimator reset', self.node.mode)

    def test_follow_heading_mode_remains_available(self):
        self.assertFalse(self.node.freeze_tracking_yaw)
        self.node.preview_pub = PublisherSpy()
        self.node.tick()
        self.node.px4.heading += .02
        self.node.tick()
        self.assertAlmostEqual(self.node.px4.heading, self.node.preview_pub.messages[-1].yaw)


if __name__ == '__main__':
    unittest.main()
