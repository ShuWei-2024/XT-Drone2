#!/usr/bin/env python3
"""Measure camera_init to PX4-local alignment in Gazebo without commanding flight."""

import argparse
import bisect
import csv
import json
from pathlib import Path
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleLocalPosition
from px4_topics import px4_topic
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def ros_stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def gazebo_stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nsec


def yaw_from_quaternion(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z)))


def heading_error_deg(source_yaw, target_yaw, rotation):
    source = np.column_stack((np.cos(source_yaw), np.sin(source_yaw)))
    target = np.column_stack((np.cos(target_yaw), np.sin(target_yaw)))
    predicted = source @ rotation
    dot = np.sum(predicted * target, axis=1)
    cross = predicted[:, 0] * target[:, 1] - predicted[:, 1] * target[:, 0]
    return np.degrees(np.abs(np.arctan2(cross, dot)))


def nearest_pairs(source, target, source_key, target_key, tolerance_ns):
    target = sorted(target, key=lambda row: row[target_key])
    keys = [row[target_key] for row in target]
    paired = []
    for row in source:
        index = bisect.bisect_left(keys, row[source_key])
        choices = target[max(0, index - 1):min(len(target), index + 1)]
        if choices:
            match = min(choices, key=lambda item: abs(item[target_key] - row[source_key]))
            if abs(match[target_key] - row[source_key]) <= tolerance_ns:
                paired.append((row, match, abs(match[target_key] - row[source_key])))
    return paired


def fit_horizontal(source, target):
    a = np.asarray(source, dtype=float)
    b = np.asarray(target, dtype=float)
    center_a, center_b = a.mean(axis=0), b.mean(axis=0)
    u, singular, vt = np.linalg.svd((a - center_a).T @ (b - center_b))
    rotation = u @ vt
    translation = center_b - center_a @ rotation
    error = np.linalg.norm(a @ rotation + translation - b, axis=1)
    return rotation, translation, error, singular


class FrameAudit(Node):
    def __init__(self, world, model, px4_timebase='dds', px4_namespace='', truth_reference_offset=(0., 0., 0.)):
        super().__init__('frame_alignment_audit')
        self.px4_timebase = px4_timebase
        self.truth_reference_offset = np.asarray(truth_reference_offset, dtype=float)
        self.lock = threading.Lock()
        self.truth_rows = []
        self.lio_rows = []
        self.px4_rows = []
        self.offset_rows = []
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self.on_truth)
        self.model = model
        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_lio, 10)
        self.create_subscription(VehicleLocalPosition, px4_topic(
            'out', 'vehicle_local_position', VehicleLocalPosition, px4_namespace),
                                 self.on_px4, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, px4_topic(
            'out', 'timesync_status', TimesyncStatus, px4_namespace),
                                 self.on_timesync, qos_profile_sensor_data)

    def on_truth(self, msg):
        pose = next((item for item in msg.pose if item.name == self.model), None)
        if pose is None:
            return
        p = pose.position
        q = pose.orientation
        # Rotate a model-fixed lever arm; never add a constant in world Z.
        vector = np.array((q.x, q.y, q.z))
        offset = self.truth_reference_offset
        rotated_offset = offset + 2*np.cross(vector, np.cross(vector, offset)+q.w*offset)
        row = (gazebo_stamp_ns(msg.header.stamp), time.monotonic_ns(),
               np.array((p.x, p.y, p.z), dtype=float) + rotated_offset,
               yaw_from_quaternion(pose.orientation))
        with self.lock:
            if not self.truth_rows or row[0] - self.truth_rows[-1][0] >= 10_000_000:
                self.truth_rows.append(row)

    def on_lio(self, msg):
        if msg.header.frame_id != 'camera_init':
            return
        p = msg.pose.pose.position
        row = (ros_stamp_ns(msg.header.stamp), time.monotonic_ns(),
               np.array((p.x, p.y, p.z), dtype=float),
               yaw_from_quaternion(msg.pose.pose.orientation))
        if row[0] > 0 and np.isfinite(row[2]).all():
            self.lio_rows.append(row)

    def on_px4(self, msg):
        p = np.array((msg.x, msg.y, msg.z), dtype=float)
        if msg.xy_valid and msg.z_valid and msg.timestamp_sample > 0 and np.isfinite(p).all():
            self.px4_rows.append((msg.timestamp_sample * 1000,
                                  time.monotonic_ns(), p,
                                  bool(msg.heading_good_for_control),
                                  (msg.xy_reset_counter, msg.z_reset_counter,
                                   msg.heading_reset_counter), float(msg.heading),
                                  msg.timestamp * 1000))

    def on_timesync(self, msg):
        if msg.source_protocol == TimesyncStatus.SOURCE_PROTOCOL_DDS:
            self.offset_rows.append((time.monotonic_ns(), msg.estimated_offset,
                                     msg.round_trip_time))

    def export_samples(self, path):
        with self.lock:
            truth = list(self.truth_rows)
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(('source', 'measurement_time_ns', 'receipt_monotonic_ns',
                             'publication_time_ns', 'x_m', 'y_m', 'z_m', 'yaw_rad',
                             'heading_good', 'xy_reset', 'z_reset', 'heading_reset'))
            for timestamp, receipt, position, yaw in truth:
                writer.writerow(('gazebo', timestamp, receipt, '', *position, yaw,
                                 '', '', '', ''))
            for timestamp, receipt, position, yaw in self.lio_rows:
                writer.writerow(('fast_livo2', timestamp, receipt, '', *position, yaw,
                                 '', '', '', ''))
            for timestamp, receipt, position, ready, resets, heading, published in self.px4_rows:
                writer.writerow(('px4', timestamp, receipt, published, *position,
                                 heading, int(ready), *resets))

    def analyze(self):
        with self.lock:
            truth = list(self.truth_rows)
        lio = list(self.lio_rows)
        px4 = list(self.px4_rows)
        offsets = list(self.offset_rows)
        all_lio_truth = nearest_pairs(lio, truth, 0, 0, 30_000_000)
        if self.px4_timebase == 'sim':
            synced_px4 = px4
        else:
            paired_offsets = nearest_pairs(px4, offsets, 1, 0, 1_000_000_000)
            synced_px4 = [(row[0] + offset[1] * 1000, row[1], row[2], row[3], row[4], row[5])
                          for row, offset, _ in paired_offsets]
        all_px4_truth = nearest_pairs(synced_px4, truth, 0, 0, 30_000_000)
        aligned_pose = next((row for row in all_px4_truth
                             if row[1][2][2] > 0.3 and row[0][3]), None)
        alignment_start_ns = aligned_pose[0][0] if aligned_pose else None
        lio_truth = ([row for row in all_lio_truth if row[0][0] >= alignment_start_ns]
                     if alignment_start_ns else all_lio_truth)
        px4_truth = ([row for row in all_px4_truth if row[0][0] >= alignment_start_ns]
                     if alignment_start_ns else all_px4_truth)
        report = {
            'truth_samples': len(truth), 'lio_samples': len(lio),
            'px4_valid_samples': len(px4), 'timesync_samples': len(offsets),
            'px4_with_timesync': len(synced_px4) if self.px4_timebase == 'dds' else None,
            'px4_with_timebase': len(synced_px4),
            'lio_truth_pairs': len(all_lio_truth), 'px4_truth_pairs': len(all_px4_truth),
            'fit_lio_pairs': len(lio_truth), 'fit_px4_pairs': len(px4_truth),
            'alignment_start_ns': alignment_start_ns,
            'px4_timebase': self.px4_timebase,
            'certified': False,
            'truth_reference_offset_model_m': self.truth_reference_offset.tolist(),
        }
        if min(len(lio_truth), len(px4_truth)) < 50:
            report['reason'] = 'insufficient time-matched poses after heading alignment'
            return report

        lio_points = np.asarray([row[0][2] for row in lio_truth])
        lio_ground = np.asarray([row[1][2] for row in lio_truth])
        px4_ground = np.asarray([row[1][2] for row in px4_truth])
        px4_points = np.asarray([row[0][2] for row in px4_truth])
        report['gazebo_xy_span_m'] = np.ptp(px4_ground[:, :2], axis=0).tolist()
        report['heading_good_fraction'] = float(np.mean([row[0][3] for row in all_px4_truth]))
        airborne_rows = [row for row in px4_truth if row[1][2][2] > 0.3]
        report['airborne_heading_good_fraction'] = (float(np.mean([row[0][3] for row in airborne_rows]))
                                                    if airborne_rows else None)
        report['px4_resets_during_audit'] = np.ptp(
            np.asarray([row[0][4] for row in all_px4_truth]), axis=0).tolist()
        report['px4_resets_after_heading_alignment'] = (
            np.ptp(np.asarray([row[0][4] for row in px4_truth]), axis=0).tolist()
            if alignment_start_ns else None)
        report['lio_truth_max_dt_ms'] = max(row[2] for row in all_lio_truth) / 1e6
        report['px4_truth_max_dt_ms'] = max(row[2] for row in all_px4_truth) / 1e6
        report['timesync_rtt_p95_ms'] = (float(np.percentile(
            [row[2] for row in offsets], 95) / 1000) if offsets else None)
        report['timesync_offset_jumps_over_20ms'] = (int(np.count_nonzero(
            np.abs(np.diff([row[1] for row in offsets])) > 20_000)) if offsets else None)
        report['px4_measurement_time_p95_ms'] = float(np.percentile(
            [row[2] for row in all_px4_truth], 95) / 1e6)
        report['px4_sample_to_publish_p95_ms'] = float(np.percentile(
            [(row[6] - row[0]) / 1e6 for row in px4], 95))

        rotation_gl, translation_gl, error_gl, singular_gl = fit_horizontal(
            lio_points[:, :2], lio_ground[:, :2])
        rotation_gp, translation_gp, error_gp, singular_gp = fit_horizontal(
            px4_ground[:, :2], px4_points[:, :2])
        rotation_lp = rotation_gl @ rotation_gp
        translation_lp = translation_gl @ rotation_gp + translation_gp
        lio_z_offset = float(np.median(lio_ground[:, 2] - lio_points[:, 2]))
        px4_z_offset = float(np.median(px4_points[:, 2] + px4_ground[:, 2]))
        motion_constrained = (np.min(report['gazebo_xy_span_m']) >= 0.3 and
                              singular_gl[-1] >= 0.1 and singular_gp[-1] >= 0.1)
        report['camera_init_to_px4_ned_xy'] = ({
            'matrix': rotation_lp.T.tolist(),
            'translation_m': translation_lp.tolist(),
            'z_sign': -1,
            'z_translation_m': px4_z_offset - lio_z_offset,
        } if motion_constrained and alignment_start_ns else None)
        report['lio_to_gazebo_p95_m'] = float(np.percentile(error_gl, 95))
        report['gazebo_to_px4_p95_m'] = float(np.percentile(error_gp, 95))
        report['lio_z_p95_m'] = float(np.percentile(np.abs(
            lio_points[:, 2] + lio_z_offset - lio_ground[:, 2]), 95))
        report['px4_z_p95_m'] = float(np.percentile(np.abs(
            px4_points[:, 2] + px4_ground[:, 2] - px4_z_offset), 95))
        report['lio_motion_singular_values'] = singular_gl.tolist()
        report['px4_motion_singular_values'] = singular_gp.tolist()
        report['lio_heading_p95_deg'] = None
        report['px4_heading_p95_deg'] = None
        if motion_constrained:
            lio_heading_error = heading_error_deg(
                [row[0][3] for row in lio_truth], [row[1][3] for row in lio_truth],
                rotation_gl)
            report['lio_heading_p95_deg'] = float(np.percentile(lio_heading_error, 95))
            px4_good_heading = [(row[1][3], row[0][5]) for row in px4_truth
                                if row[0][3] and np.isfinite(row[0][5])]
            if px4_good_heading:
                px4_heading_error = heading_error_deg(
                    [row[0] for row in px4_good_heading],
                    [row[1] for row in px4_good_heading], rotation_gp)
                report['px4_heading_p95_deg'] = float(np.percentile(px4_heading_error, 95))
        failures = []
        if np.min(report['gazebo_xy_span_m']) < 0.3:
            failures.append('requires at least 0.3 m motion on each horizontal axis')
        if singular_gl[-1] < 0.1 or singular_gp[-1] < 0.1:
            failures.append('horizontal motion does not constrain both axes')
        if alignment_start_ns is None:
            failures.append('PX4 in-flight heading alignment was not observed')
        elif report['airborne_heading_good_fraction'] < 0.9:
            failures.append('PX4 heading quality is not consistently valid after in-flight alignment')
        if motion_constrained and (report['lio_heading_p95_deg'] > 10.0 or
                                   report['px4_heading_p95_deg'] is None or
                                   report['px4_heading_p95_deg'] > 10.0):
            failures.append('body heading disagrees with fitted frame transform')
        if report['lio_to_gazebo_p95_m'] > 0.10 or report['gazebo_to_px4_p95_m'] > 0.10:
            failures.append('frame fit residual exceeds 0.10 m')
        if report['lio_z_p95_m'] > 0.10 or report['px4_z_p95_m'] > 0.10:
            failures.append('vertical frame fit residual exceeds 0.10 m')
        if report['px4_resets_after_heading_alignment'] and any(report['px4_resets_after_heading_alignment']):
            failures.append('PX4 local estimator reset after heading alignment')
        if self.px4_timebase == 'dds' and report['timesync_offset_jumps_over_20ms']:
            failures.append('PX4 time synchronization jumped during audit')
        if self.px4_timebase == 'dds' and len(synced_px4) < 0.9 * len(px4):
            failures.append('PX4 time-sync coverage below 90 percent')
        if len(all_px4_truth) < 0.9 * len(px4) or report['px4_measurement_time_p95_ms'] > 20.0:
            failures.append('PX4 measurement-time alignment is insufficient')
        if np.linalg.det(rotation_lp) > -0.9:
            failures.append('horizontal frame handedness differs from expected ENU/NED')
        report['reason'] = '; '.join(failures) if failures else 'alignment checks passed'
        report['fitted_alignment_checks_passed'] = not failures
        # This fit uses the entire recorded route and cannot certify a frozen
        # transform used prospectively by an execution bridge.
        report['certified'] = False
        report['certification_scope'] = 'diagnostic whole-record fit only; use the frozen-start motion audit for acceptance'
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=45.0)
    parser.add_argument('--world', default='astra_px4_open')
    parser.add_argument('--model', default='astra_x500_px4')
    parser.add_argument('--px4-timebase', choices=('dds', 'sim'), default='dds',
                        help='Use DDS-synchronized timestamps or raw PX4 simulation timestamps')
    parser.add_argument('--output')
    parser.add_argument('--px4-namespace', default='')
    parser.add_argument('--truth-reference-offset', nargs=3, type=float, default=(0., 0., 0.),
                        help='Audited model-to-body translation, rotated using Gazebo attitude')
    parser.add_argument('--stop-file', type=Path,
                        help='Finish and export when the controller creates this file')
    parser.add_argument('--samples-output',
                        help='Write timestamped Gazebo, FAST-LIVO2, and PX4 poses as CSV')
    args = parser.parse_args()
    rclpy.init()
    node = FrameAudit(args.world, args.model, args.px4_timebase, args.px4_namespace,
                      args.truth_reference_offset)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            if args.stop_file and args.stop_file.exists():
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.analyze()
        result = json.dumps(report, indent=2)
        print(result)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as stream:
                stream.write(result + '\n')
        if args.samples_output:
            node.export_samples(args.samples_output)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
