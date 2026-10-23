#!/usr/bin/env python3
"""Initialize a missing absolute altitude origin after EV starts, disarmed SITL only.

No Gazebo truth, parameter changes, arming, modes or trajectory commands. Keep
the established horizontal origin and local Z; use measured GNSS MSL altitude
to georeference it. An accepted ACK alone is insufficient: check local/global
outputs and continuous EV fusion before allowing the runner to proceed.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (SensorGps, VehicleCommand, VehicleCommandAck,
                          VehicleGlobalPosition, VehicleLocalPosition,
                          VehicleStatus, EstimatorStatusFlags)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'packages/astra_px4_offboard/scripts'))
from px4_topics import px4_topic


class TelemetryPending(ValueError):
    """No valid decision yet; wait disarmed without relaxing any age gate."""


def position_stability_window(samples, duration_us=2_000_000,
                              max_axis_excursion_m=.02, min_samples=20):
    """Require a genuinely quiet, reset-free local-position history on ground.

    Samples are ``(timestamp_sample_us, x, y, z, reset_counters)``. The
    position is checked per axis over the whole measured-time window, so slow
    drift cannot pass merely because instantaneous velocity is small.
    """
    if not samples:
        return False, dict(reason='no local-position history', samples=0)
    end_us = samples[-1][0]
    window = [s for s in samples if end_us-duration_us <= s[0] <= end_us]
    details = dict(samples=len(window), duration_us=(window[-1][0]-window[0][0]
                   if len(window) > 1 else 0), required_duration_us=duration_us,
                   max_axis_excursion_m=max_axis_excursion_m)
    # A 100 Hz measured stream commonly spans 1.992 s inside a 2 s window
    # because its first sample falls just after the lower time boundary.
    if len(window) < min_samples or details['duration_us'] < duration_us-50_000:
        details['reason'] = 'insufficient measured-time coverage'
        return False, details
    stamps = [s[0] for s in window]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        details['reason'] = 'duplicate or backwards local-position measurement time'
        return False, details
    if max(b-a for a, b in zip(stamps, stamps[1:])) > 250_000:
        details['reason'] = 'local-position measurement gap exceeds 250 ms'
        return False, details
    reset_vectors = {tuple(s[4]) for s in window}
    if len(reset_vectors) != 1:
        details['reason'] = 'PX4 reset during local-position stability window'
        return False, details
    ranges = [max(s[i] for s in window)-min(s[i] for s in window)
              for i in (1, 2, 3)]
    details['axis_excursion_m'] = ranges
    details['reset_counters'] = list(next(iter(reset_vectors)))
    if max(ranges) > max_axis_excursion_m:
        details['reason'] = 'local-position excursion exceeds 0.02 m'
        return False, details
    details['reason'] = ''
    return True, details


def absolute_origin_valid(local, global_position):
    # PX4 1.17 sets lpos.z_global with the horizontal origin and returns zero
    # for an uninitialized ref_alt. Only gpos.alt_valid checks actual validity.
    return bool(global_position is not None and global_position.alt_valid and
                local.z_global and math.isfinite(local.ref_alt))


def check_origin_displacement(before, local, report):
    """Keep the existing gate and retain the exact offending estimator sample."""
    after = [local.x, local.y, local.z]
    displacement = math.dist(before, after)
    if displacement > .03:
        report['local_displacement_failure'] = dict(
            before_local_ned_m=list(before), after_local_ned_m=after,
            delta_local_ned_m=[new-old for old, new in zip(before, after)],
            displacement_m=displacement, limit_m=.03,
            timestamp_sample_us=int(local.timestamp_sample),
            after_resets=[local.xy_reset_counter, local.z_reset_counter, local.heading_reset_counter],
            ref_alt_m=float(local.ref_alt))
        raise RuntimeError('origin initialization moved local position by more than 0.03 m')


def origin_candidate(local, gps, status, flags, now_us):
    """Require measured, stationary, disarmed data; never move an existing XY origin."""
    if status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
        raise ValueError('vehicle is not disarmed')
    if not (flags.cs_ev_pos and flags.cs_ev_hgt):
        raise ValueError('EV XY/Z fusion has not started')
    if not (local.xy_valid and local.z_valid and local.xy_global):
        raise ValueError('valid local position and horizontal global origin required')
    values = [local.x, local.y, local.z, local.vx, local.vy, local.vz,
              local.ref_lat, local.ref_lon, gps.altitude_msl_m, gps.eph, gps.epv,
              gps.vel_m_s]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite origin input')
    if not (-90 <= local.ref_lat <= 90 and -180 <= local.ref_lon <= 180):
        raise ValueError('invalid horizontal origin')
    if not (0 <= now_us-local.timestamp_sample <= 250_000 and
            0 <= now_us-gps.timestamp_sample <= 500_000):
        raise TelemetryPending('stale or future measured position: '
            f'local_age_us={now_us-local.timestamp_sample}, gps_age_us={now_us-gps.timestamp_sample}')
    if (gps.fix_type < 3 or gps.satellites_used < 6 or not gps.vel_ned_valid or
            not 0 <= gps.eph <= 3 or not 0 <= gps.epv <= 5):
        raise ValueError('GNSS quality insufficient')
    if (math.hypot(local.vx, local.vy) > .2 or abs(local.vz) > .15 or
            not 0 <= gps.vel_m_s < .2 or abs(local.z) > .5):
        raise ValueError('stationary ground state required')
    altitude = gps.altitude_msl_m + local.z
    if not -12_000 < altitude < 100_000:
        raise ValueError('invalid absolute altitude')
    return [local.ref_lat, local.ref_lon, altitude]


class Initializer(Node):
    def __init__(self):
        super().__init__('astra_disarmed_ev_global_origin', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.data = {}; self.receipts = {}; self.acks = []; self.request_us = None
        self.local_history = deque(maxlen=1000)
        for key, topic, cls in (
                ('local', 'vehicle_local_position', VehicleLocalPosition),
                ('global', 'vehicle_global_position', VehicleGlobalPosition),
                ('gps', 'vehicle_gps_position', SensorGps),
                ('status', 'vehicle_status', VehicleStatus),
                ('flags', 'estimator_status_flags', EstimatorStatusFlags)):
            self.create_subscription(cls, px4_topic('out', topic, cls, 'x500_0'),
                lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck,
            px4_topic('out', 'vehicle_command_ack', VehicleCommandAck, 'x500_0'),
            self.ack, qos_profile_sensor_data)
        self.pub = self.create_publisher(VehicleCommand,
            px4_topic('in', 'vehicle_command', VehicleCommand, 'x500_0'), 10)

    def receive(self, key, msg):
        self.data[key] = msg; self.receipts[key] = time.monotonic()
        if key == 'local' and msg.timestamp_sample > 0:
            self.local_history.append((int(msg.timestamp_sample), float(msg.x),
                float(msg.y), float(msg.z), (int(msg.xy_reset_counter),
                int(msg.z_reset_counter), int(msg.heading_reset_counter))))

    def ack(self, msg):
        if (self.request_us is not None and msg.timestamp >= self.request_us and
                msg.command == VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN and
                msg.target_system == 1 and msg.target_component == 197):
            self.acks.append(dict(timestamp_us=msg.timestamp, result=msg.result))

    def candidate(self):
        for key, timeout in (('local', .25), ('global', .25), ('gps', .5), ('status', 1.), ('flags', 1.5)):
            if time.monotonic()-self.receipts.get(key, 0.) > timeout:
                raise TelemetryPending(key+' telemetry stale or missing')
        return origin_candidate(*(self.data[k] for k in ('local', 'gps', 'status', 'flags')),
                                self.get_clock().now().nanoseconds//1000)

    def send(self, origin):
        msg = VehicleCommand()
        self.request_us = self.get_clock().now().nanoseconds//1000
        msg.timestamp = self.request_us
        msg.command = VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
        msg.param5, msg.param6, msg.param7 = origin
        msg.target_system = msg.source_system = 1
        msg.target_component = 1; msg.source_component = 197
        msg.from_external = True
        self.pub.publish(msg)

    def input_snapshot(self):
        now_us = self.get_clock().now().nanoseconds//1000
        wall = time.monotonic()
        return dict(sim_us=now_us,
            measurement_age_us={key:now_us-int(self.data[key].timestamp_sample)
                for key in ('local','gps') if key in self.data},
            receipt_age_sec={key:wall-value for key,value in self.receipts.items()})

    def local_position_stable(self):
        return position_stability_window(list(self.local_history))

    def run(self, report):
        deadline = time.monotonic()+60.; stable = None; reason = ''
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            try:
                origin = self.candidate()
            except ValueError as exc:
                stable = None; reason = str(exc); continue
            position_stable, stability = self.local_position_stable()
            report['pre_command_stability'] = stability
            if not position_stable:
                stable = None
                reason = stability['reason']
                continue
            if stable is None:stable = time.monotonic()
            if time.monotonic()-stable >= 1.:break
        else:raise RuntimeError('ground preparation timed out: '+reason)
        local = self.data['local']
        before = [local.x, local.y, local.z]
        report.update(before_local_ned_m=before, proposed_origin_lla=origin,
                      before_resets=[local.xy_reset_counter,local.z_reset_counter,local.heading_reset_counter])
        if not absolute_origin_valid(local,self.data.get('global')):
            # EV must already be active: origin initialization then preserves
            # local height and shifts the fixed EV bias by the absolute datum.
            self.send(origin)
            report['command_sent'] = True
        else:
            origin = [local.ref_lat,local.ref_lon,local.ref_alt]
            report['existing_origin_preserved'] = True
        stable = None
        report['post_command_pending_count'] = 0
        report['post_command_pending_samples'] = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            try:
                self.candidate()
            except TelemetryPending as exc:
                # /clock and PX4 messages are separate callbacks. An accepted
                # command must not turn one temporarily old/future sample into
                # a fatal initialization error. Remain disarmed, reset the full
                # continuous-health window and wait within the original deadline.
                stable = None; reason = str(exc)
                report['post_command_pending_count'] += 1
                sample = dict(reason=reason, **self.input_snapshot())
                if len(report['post_command_pending_samples']) < 16:
                    report['post_command_pending_samples'].append(sample)
                report['last_pending_sample'] = sample
                continue
            # Armed, unfused, invalid or moving data still fails immediately.
            local = self.data['local']; global_pos = self.data.get('global')
            check_origin_displacement(before, local, report)
            accepted = (not report['command_sent'] or any(
                a['result'] == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED for a in self.acks))
            confirmed = (accepted and local.z_global and local.xy_global and
                math.isfinite(local.ref_alt) and abs(local.ref_alt-origin[2]) < .02 and
                abs(local.ref_lat-origin[0]) < 1e-8 and abs(local.ref_lon-origin[1]) < 1e-8 and
                global_pos is not None and global_pos.alt_valid and global_pos.lat_lon_valid and
                time.monotonic()-self.receipts.get('global',0.) < .25 and
                abs(global_pos.alt-(local.ref_alt-local.z)) < .05)
            if not confirmed:stable = None; continue
            if stable is None:stable = time.monotonic()
            if time.monotonic()-stable >= 1.:
                report.update(confirmed=True, after_local_ned_m=[local.x,local.y,local.z],
                    after_resets=[local.xy_reset_counter,local.z_reset_counter,local.heading_reset_counter],
                    origin_lla=[local.ref_lat,local.ref_lon,local.ref_alt],
                    confirmed_sim_us=self.get_clock().now().nanoseconds//1000)
                return
        raise RuntimeError('origin ACK/local/global verification timed out; last pending: '+reason)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if (os.environ.get('ASTRA_117_ENV_LOADED')!='1' or os.environ.get('ROS_DOMAIN_ID')!='94' or
            os.environ.get('GZ_PARTITION')!='astra_px4_117'):
        raise RuntimeError('requires isolated Astra PX4 1.17 SITL runner')
    report=dict(confirmed=False,command_sent=False,ground_truth_used=False,
                reference='GNSS absolute origin only; EV remains local height reference',acks=[])
    rclpy.init();node=Initializer();code=1
    try:node.run(report);code=0
    except Exception as exc:
        report['failure']=str(exc)
        report['failure_input_snapshot']=node.input_snapshot()
    finally:
        report['acks']=node.acks
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
        node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(code)


if __name__=='__main__':main()
