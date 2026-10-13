"""Offline PX4 input/estimate evidence; never feeds truth into the flight stack."""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/cyf/dev/px4/venv/lib/python3.12/site-packages')
from pyulog import ULog

from summarize_stability_20261010 import stats


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def audit(root):
    logfile = next(root.glob('log/*/*.ulg'))
    ulog = ULog(str(logfile))
    data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
    events = json.loads((root / 'motion_events.json').read_text())['events']
    goals = [e for e in events if e['event'] == 'ego_goal_sent']
    stop = next(e for e in events if e['event'] == 'ego_flight_complete')['sim_ns'] / 1e3
    with (root / 'frame_samples.csv').open() as f:
        truth = sorted([r for r in csv.DictReader(f) if r['source'] == 'gazebo'],
                       key=lambda r: int(r['measurement_time_ns']))
    truth_us = np.array([int(r['measurement_time_ns']) / 1e3 for r in truth])
    truth_xy = np.array([[float(r['x_m']), float(r['y_m'])] for r in truth])
    truth_z = np.array([float(r['z_m']) for r in truth])
    # Samples are in the simulation clock domain in these isolated runs.
    truth_yaw = np.unwrap([float(r['yaw_rad']) for r in truth])

    def selected(name, lo, hi):
        source = data[name]
        ts = source.get('timestamp_sample', source['timestamp'])
        mask = (ts >= lo) & (ts <= hi)
        return {k: np.asarray(v)[mask] for k, v in source.items()}

    def input_bias(name, key, lo, hi):
        d = selected(name, lo, hi)
        ts = d.get('timestamp_sample', d['timestamp'])
        # Interpolate truth only for this offline input diagnostic. Pose-error
        # certification remains the separate <=20 ms nearest-pair auditor.
        offset = d[key] - np.interp(ts, truth_us, truth_z)
        return dict(median=float(np.median(offset)), minimum=float(offset.min()),
                    maximum=float(offset.max()))

    lo0 = goals[0]['sim_ns'] / 1e3
    initial_baro = input_bias('vehicle_air_data', 'baro_alt_meter', lo0-2e6, lo0)['median']
    gps = data['vehicle_gps_position']
    lat0, lon0 = gps['latitude_deg'][0], gps['longitude_deg'][0]
    gps_xy = np.column_stack((
        np.radians(gps['longitude_deg']-lon0)*6378137*np.cos(np.radians(lat0)),
        np.radians(gps['latitude_deg']-lat0)*6378137))
    gps_ts = gps['timestamp_sample']
    gps_truth_xy = np.column_stack([
        np.interp(gps_ts, truth_us, truth_xy[:, j]) for j in range(2)])
    gps_initial = (gps_ts >= lo0-2e6) & (gps_ts <= lo0)
    gps_bias = np.median(gps_truth_xy[gps_initial]-gps_xy[gps_initial], axis=0)
    gps_error = np.linalg.norm(gps_xy+gps_bias-gps_truth_xy, axis=1)
    report = dict(log=str(logfile.resolve()), parameters={k: ulog.initial_parameters.get(k)
                  for k in ('EKF2_HGT_REF', 'EKF2_BARO_CTRL', 'EKF2_GPS_CTRL',
                            'EKF2_EV_CTRL', 'EKF2_MAG_TYPE')}, legs=[])
    flags = ('cs_gps', 'cs_baro_hgt', 'cs_gps_hgt', 'cs_ev_pos', 'cs_ev_hgt',
             'cs_ev_yaw', 'cs_ev_vel', 'cs_mag_hdg', 'cs_mag_3d',
             'cs_mag_field_disturbed', 'reject_yaw')
    for i, goal in enumerate(goals):
        lo = goal['sim_ns'] / 1e3
        hi = goals[i+1]['sim_ns'] / 1e3 if i+1 < len(goals) else stop
        p = selected('vehicle_local_position', lo, hi)
        s = selected('trajectory_setpoint', lo, hi)
        f = selected('estimator_status_flags', lo, hi)
        b = selected('estimator_aid_src_baro_hgt', lo, hi)
        ts = p['timestamp_sample']
        estimate_yaw_enu = np.pi/2 - p['heading']
        truth_heading = np.interp(ts, truth_us, truth_yaw)
        setpoint_ts = s['timestamp']
        estimated_heading = np.interp(setpoint_ts, ts, np.unwrap(p['heading']))
        yaw_difference = wrap(s['yaw'] - estimated_heading) * 180 / np.pi
        baro = input_bias('vehicle_air_data', 'baro_alt_meter', lo, hi)
        baro['change_from_initial_median_m'] = baro['median'] - initial_baro
        gps_active = (gps_ts >= lo) & (gps_ts <= hi)
        leg = dict(index=i, start_sim_sec=lo/1e6, stop_sim_sec=hi/1e6,
                   z_setpoint_range_m=float(np.ptp(s['position[2]'])),
                   yaw_setpoint_range_deg=float(np.ptp(np.unwrap(s['yaw']))*180/np.pi),
                   yaw_setpoint_vs_estimate_abs_deg=stats(np.abs(yaw_difference)),
                   yaw_estimate_truth_signed_first_deg=float(wrap(estimate_yaw_enu[0]-truth_heading[0])*180/np.pi),
                   yaw_estimate_truth_signed_last_deg=float(wrap(estimate_yaw_enu[-1]-truth_heading[-1])*180/np.pi),
                   reset_counter_changes={k: int(np.max(p[k])-np.min(p[k]))
                                          for k in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')},
                   fusion_flag_fraction={k: float(np.mean(f[k])) for k in flags},
                   baro_minus_truth_height_m=baro,
                   gps_input_truth_xy_error_frozen_initial_m=stats(gps_error[gps_active]),
                   baro_innovation_abs_m=stats(np.abs(b['innovation'])),
                   baro_rejected_fraction=float(np.mean(b['innovation_rejected'])))
        for name in ('vehicle_gps_position', 'vehicle_air_data', 'sensor_mag', 'battery_status'):
            if name in data:
                d = selected(name, lo, hi)
                dt = np.diff(d['timestamp'].astype(float)) / 1000
                leg[name+'_publication_gap_ms'] = stats(dt)
        report['legs'].append(leg)
    (root / 'px4_motion_sources_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directories', type=Path, nargs='+')
    args = parser.parse_args()
    for directory in args.directories:
        result = audit(directory)
        print(directory.name)
        for leg in result['legs']:
            print('leg', leg['index'], 'yaw command-estimate p95 deg',
                  leg['yaw_setpoint_vs_estimate_abs_deg']['p95'],
                  'baro residual median change m',
                  leg['baro_minus_truth_height_m']['change_from_initial_median_m'])
