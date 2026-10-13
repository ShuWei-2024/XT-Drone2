#!/usr/bin/env python3
"""SITL-only route that starts timed holds only after measured settling."""

import argparse
from collections import deque
import csv
import json
import math
import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus

from px4_calibration_flight import CalibrationFlight


class SettledFlight(CalibrationFlight):
    def __init__(self):
        self.truth_history = deque(maxlen=300)
        super().__init__()
        self.events = []
        self.gate_rows = []
        self.setpoint_rows = []

    def send(self, target):
        super().send(target)
        self.setpoint_rows.append({
            'measurement_time_us': int(self.local.timestamp_sample),
            'publication_time_us': int(self.local.timestamp),
            'target_x_ned_m': float(target[0]),
            'target_y_ned_m': float(target[1]),
            'target_z_ned_m': float(target[2]),
            'target_yaw_rad': float(target[3]),
        })

    def on_truth(self, msg):
        super().on_truth(msg)
        if self.truth is not None:
            stamp = msg.header.stamp
            sim_s = stamp.sec + stamp.nsec * 1e-9
            self.truth_history.append((sim_s, self.truth))

    def check_safe(self):
        if not self.fresh():
            raise RuntimeError('stale or invalid PX4/Gazebo state')
        if (math.hypot(self.truth[0], self.truth[1]) > 2.0 or
                self.truth[2] < -0.2 or self.truth[2] > 2.5):
            raise RuntimeError('Gazebo vehicle left calibration bounds')
        if (self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            raise RuntimeError('PX4 left Offboard during calibration')

    def truth_motion(self):
        samples = list(self.truth_history)
        if not samples:
            return None
        latest = samples[-1][0]
        samples = [row for row in samples if row[0] >= latest - 1.5]
        if len(samples) < 10 or samples[-1][0] - samples[0][0] < 1.3:
            return None
        dt = samples[-1][0] - samples[0][0]
        xy_path = sum(math.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1])
                      for a, b in zip(samples, samples[1:]))
        z_path = sum(abs(b[1][2] - a[1][2]) for a, b in zip(samples, samples[1:]))
        xy_net = math.hypot(samples[-1][1][0] - samples[0][1][0],
                            samples[-1][1][1] - samples[0][1][1])
        z_net = abs(samples[-1][1][2] - samples[0][1][2])
        return xy_net / dt, z_net / dt, xy_path / dt, z_path / dt

    def gate_metrics(self, target):
        motion = self.truth_motion()
        px4_xy_error = math.hypot(self.local.x - target[0], self.local.y - target[1])
        px4_z_error = abs(self.local.z - target[2])
        px4_speed = math.hypot(self.local.vx, self.local.vy)
        metrics = {
            'measurement_time_us': int(self.local.timestamp_sample),
            'px4_xy_target_error_m': px4_xy_error,
            'px4_z_target_error_m': px4_z_error,
            'px4_xy_speed_mps': px4_speed,
            'px4_z_speed_mps': abs(self.local.vz),
            'truth_xy_window_speed_mps': motion[0] if motion else math.nan,
            'truth_z_window_speed_mps': motion[1] if motion else math.nan,
            'truth_xy_path_speed_mps': motion[2] if motion else math.nan,
            'truth_z_path_speed_mps': motion[3] if motion else math.nan,
            'heading_good': bool(self.local.heading_good_for_control),
        }
        checks = {
            'truth_motion_available': motion is not None,
            'heading_good': metrics['heading_good'],
            'px4_xy_target': px4_xy_error <= 0.12,
            'px4_z_target': px4_z_error <= 0.10,
            'px4_xy_speed': px4_speed <= 0.08,
            'px4_z_speed': metrics['px4_z_speed_mps'] <= 0.06,
            'truth_xy_speed': motion is not None and motion[0] <= 0.08,
            'truth_z_speed': motion is not None and motion[1] <= 0.05,
        }
        metrics['failed_checks'] = ','.join(name for name, passed in checks.items()
                                            if not passed)
        return metrics

    def await_settled_hold(self, name, target, timeout_s=60.0, hold_s=4.0):
        deadline = time.monotonic() + timeout_s
        settle_resets = (self.local.xy_reset_counter, self.local.z_reset_counter,
                         self.local.heading_reset_counter)
        good_since = None
        hold_started = None
        hold_start_truth = None
        hold_start_px4 = None
        hold_start_sim_us = None
        hold_start_resets = None
        next_send = time.monotonic()
        next_record = time.monotonic()
        last_metrics = None
        best_streak_s = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.check_safe()
            now = time.monotonic()
            if now >= next_send:
                self.send(target)
                next_send = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check_safe()
            metrics = self.gate_metrics(target)
            last_metrics = metrics
            resets = (self.local.xy_reset_counter, self.local.z_reset_counter,
                      self.local.heading_reset_counter)
            metrics['truth_hold_xy_displacement_m'] = math.nan
            metrics['px4_hold_xy_displacement_m'] = math.nan
            metrics['px4_xy_reset_counter'], metrics['px4_z_reset_counter'], \
                metrics['px4_heading_reset_counter'] = resets
            if resets != settle_resets:
                metrics['failed_checks'] = ','.join(filter(None, (
                    metrics['failed_checks'], 'px4_reset')))
                settle_resets = resets
                good_since = None
                hold_started = None
                self.gate_rows.append({'hold': name, **metrics})
                continue
            if not metrics['failed_checks']:
                if good_since is None:
                    good_since = now
                best_streak_s = max(best_streak_s, now - good_since)
                if hold_started is None and now - good_since >= 1.5:
                    hold_started = now
                    hold_start_truth = tuple(self.truth)
                    hold_start_px4 = (self.local.x, self.local.y, self.local.z)
                    hold_start_sim_us = int(self.local.timestamp_sample)
                    hold_start_resets = (self.local.xy_reset_counter,
                                         self.local.z_reset_counter,
                                         self.local.heading_reset_counter)
                if hold_started is not None:
                    truth_hold_xy = math.hypot(self.truth[0] - hold_start_truth[0],
                                               self.truth[1] - hold_start_truth[1])
                    px4_hold_xy = math.hypot(self.local.x - hold_start_px4[0],
                                             self.local.y - hold_start_px4[1])
                    metrics['truth_hold_xy_displacement_m'] = truth_hold_xy
                    metrics['px4_hold_xy_displacement_m'] = px4_hold_xy
                    if (truth_hold_xy > 0.08 or px4_hold_xy > 0.08 or
                            resets != hold_start_resets):
                        metrics['failed_checks'] = ','.join(filter(None, (
                            metrics['failed_checks'],
                            'truth_hold_drift' if truth_hold_xy > 0.08 else '',
                            'px4_hold_drift' if px4_hold_xy > 0.08 else '',
                            'px4_reset' if resets != hold_start_resets else '')))
                        self.gate_rows.append({'hold': name, **metrics})
                        good_since = None
                        hold_started = None
                        continue
                if (hold_started is not None and now - hold_started >= hold_s and
                        int(self.local.timestamp_sample) - hold_start_sim_us >=
                        int(hold_s * 1_000_000)):
                    truth_delta = tuple(b - a for a, b in zip(hold_start_truth,
                                                               self.truth))
                    px4_delta = tuple(b - a for a, b in zip(hold_start_px4,
                                                             (self.local.x, self.local.y,
                                                              self.local.z)))
                    self.events.append({
                        'name': name,
                        'start_sim_us': hold_start_sim_us,
                        'end_sim_us': int(self.local.timestamp_sample),
                        'target_ned': list(target),
                        'truth_start_enu': list(hold_start_truth),
                        'truth_delta_enu': list(truth_delta),
                        'px4_delta_ned': list(px4_delta),
                        'max_hold_xy_displacement_m': 0.08,
                        'settled_hold': True,
                    })
                    print('settled hold', name, 'truth_delta', truth_delta,
                          'px4_delta', px4_delta, flush=True)
                    return
            else:
                good_since = None
                hold_started = None
            if now >= next_record:
                self.gate_rows.append({'hold': name, **metrics})
                next_record = now + 0.1
        raise RuntimeError(f'{name} did not remain settled for {hold_s:.1f}s '
                           f'within {timeout_s:.1f}s; best_streak_s={best_streak_s:.2f}; '
                           f'last_metrics={last_metrics}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--gate-csv', required=True)
    parser.add_argument('--setpoint-csv')
    parser.add_argument('--settle-timeout-s', type=float, default=60.0)
    args = parser.parse_args()
    if args.settle_timeout_s < 5.5 or args.settle_timeout_s > 120:
        parser.error('--settle-timeout-s must be between 5.5 and 120')
    rclpy.init()
    node = SettledFlight()
    armed = False
    completed = False
    abort_reason = None
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not node.fresh():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.fresh() or math.hypot(*node.truth[:2]) > 0.5:
            raise RuntimeError('SITL world or vehicle not ready at origin')
        p = node.local
        origin = (p.x, p.y, p.z, p.heading)
        if not all(math.isfinite(value) for value in origin):
            raise RuntimeError('non-finite PX4 local position or heading')
        if abs(p.z + node.truth[2]) > 0.15:
            raise RuntimeError('PX4 and Gazebo ground height differ by more than 0.15 m')
        node.spin_for(2.5, origin, origin)
        node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
        node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not node.fresh():
                raise RuntimeError('state lost after arming request')
            node.send(origin)
            rclpy.spin_once(node, timeout_sec=0.05)
            if (node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                    node.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                armed = True
                break
        if not armed:
            raise RuntimeError('PX4 rejected Offboard or arming')

        climb_target = (origin[0], origin[1], origin[2] - 1.7, origin[3])
        node.spin_for(10.0, origin, climb_target)
        climb_deadline = time.monotonic() + 20.0
        while node.truth[2] < 1.3 and time.monotonic() < climb_deadline:
            node.check_safe()
            node.send(climb_target)
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.truth[2] < 1.3:
            raise RuntimeError(f'climb did not reach safe yaw-excitation height; '
                               f'Gazebo z={node.truth[2]:.3f} m, '
                               f'PX4 z={node.local.z:.3f} m')
        yaw_start = (node.local.x, node.local.y, node.local.z, origin[3])
        yaw_out = (yaw_start[0], yaw_start[1], yaw_start[2], origin[3] + 0.35)
        node.spin_for(3.0, yaw_start, yaw_out)
        node.spin_for(3.0, yaw_out, yaw_start)
        high = (node.local.x, node.local.y, node.local.z, origin[3])
        north = (high[0] + 0.35, high[1], high[2], high[3])
        northeast = (north[0], north[1] + 0.35, high[2], high[3])
        east = (high[0], northeast[1], high[2], high[3])
        for name, start, target in (
                ('hover', high, high),
                ('north', high, north),
                ('east', north, northeast),
                ('return_north', northeast, east),
                ('return_east', east, high)):
            if name != 'hover':
                node.spin_for(4.0, start, target)
            node.await_settled_hold(name, target, timeout_s=args.settle_timeout_s)
        completed = True
        print('settled-hold route complete; requesting land', flush=True)
    except Exception as exc:
        abort_reason = str(exc)
        print('ABORT:', abort_reason, flush=True)
    finally:
        if armed or (node.status and
                     node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    break
            if (node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and node.truth is not None and node.truth[2] < 0.05
                    and time.monotonic() - node.truth_wall < 0.3):
                node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 21196)
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if node.status and node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                        break
        result = {'completed': completed, 'abort_reason': abort_reason,
                  'events': node.events,
                  'disarmed': bool(node.status and
                                   node.status.arming_state != VehicleStatus.ARMING_STATE_ARMED)}
        with open(args.output, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
        with open(args.gate_csv, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=('hold', 'measurement_time_us',
                'px4_xy_target_error_m', 'px4_z_target_error_m', 'px4_xy_speed_mps',
                'px4_z_speed_mps', 'truth_xy_window_speed_mps',
                'truth_z_window_speed_mps', 'truth_xy_path_speed_mps',
                'truth_z_path_speed_mps',
                'heading_good', 'truth_hold_xy_displacement_m',
                'px4_hold_xy_displacement_m', 'px4_xy_reset_counter',
                'px4_z_reset_counter', 'px4_heading_reset_counter',
                'failed_checks'))
            writer.writeheader()
            writer.writerows(node.gate_rows)
        if args.setpoint_csv:
            with open(args.setpoint_csv, 'w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=(
                    'measurement_time_us', 'publication_time_us',
                    'target_x_ned_m', 'target_y_ned_m', 'target_z_ned_m',
                    'target_yaw_rad'))
                writer.writeheader()
                writer.writerows(node.setpoint_rows)
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
