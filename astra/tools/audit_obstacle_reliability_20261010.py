"""Separate frozen-frame errors by flight phase using measured Gazebo poses.

Gazebo is used only for this offline audit, never as a flight pose input.
The known zero-yaw SITL spawn fixes the horizontal ENU/NED axis relation.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def describe(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'count': 0}
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=float(values.max()))


def positions(rows):
    return np.array([[float(row[k]) for k in ('x_m', 'y_m', 'z_m')] for row in rows])


def pair(source, target):
    if not source or not target:
        return []
    stamps = np.array([int(x['measurement_time_ns']) for x in target], dtype=np.int64)
    paired = []
    for x in source:
        stamp = int(x['measurement_time_ns'])
        j = int(np.searchsorted(stamps, stamp))
        candidates = [i for i in (j - 1, j) if 0 <= i < len(stamps)]
        i = min(candidates, key=lambda k: abs(int(stamps[k]) - stamp))
        if abs(int(stamps[i]) - stamp) <= 20_000_000:
            paired.append((x, target[i]))
    return paired


def audit(directory):
    result = json.loads((directory / 'motion_events.json').read_text())
    events = result['events']
    event = {x['event']: x for x in events}
    if 'ego_control_handoff' not in event:
        return {'flight_complete': result['flight_complete'], 'fault': result['fault'],
                'reason': 'No planner-driven execution; no motion certification'}
    goals = [e for e in events if e['event'] == 'ego_goal_sent']
    goal_ns = goals[0]['sim_ns']
    with (directory / 'frame_samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    streams = {kind: sorted([x for x in rows if x['source'] == kind],
                            key=lambda x: int(x['measurement_time_ns']))
               for kind in ('gazebo', 'fast_livo2', 'px4')}
    calibrate = lambda row: goal_ns - 2_000_000_000 <= int(row['measurement_time_ns']) <= goal_ns
    transforms = {'fast_livo2': np.eye(3),
                  'px4': np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])}
    bias = {}
    for kind in ('fast_livo2', 'px4'):
        paired = pair([x for x in streams[kind] if calibrate(x)], streams['gazebo'])
        if not paired:
            raise ValueError('Missing initial measurement pairs for ' + kind)
        bias[kind] = np.median(positions([b for a, b in paired]) -
                               positions([a for a, b in paired]) @ transforms[kind].T, axis=0)
    begin = event['ego_control_handoff']['sim_ns']
    end = (event.get('ego_trajectory_sampled_to_end') or event.get('test_aborted'))['sim_ns']
    intervals = {'ego_execution': (begin, end)}
    for i, goal in enumerate(goals):
        leg_end = goals[i + 1]['sim_ns'] if i + 1 < len(goals) else end
        intervals[f'leg_{i}'] = (max(begin, goal['sim_ns']), leg_end)
    if 'ego_flight_complete' in event:
        intervals['endpoint_hold'] = (end, event['ego_flight_complete']['sim_ns'])
    if 'land_requested' in event and 'disarmed' in event:
        intervals['landing'] = (event['land_requested']['sim_ns'], event['disarmed']['sim_ns'])
    report = {'flight_complete': result['flight_complete'], 'fault': result['fault'],
              'calibration': 'Frozen initial offsets, zero-yaw SITL axes; no refit on flight data',
              'initial_truth_offset_m': {k: v.tolist() for k, v in bias.items()}, 'phases': {}}
    timing = list(csv.DictReader((directory / 'livo_timing.csv').open()))
    alignment = result['alignment']
    matrix = np.array(alignment['xy_matrix'])
    translation = np.array(alignment['xy_translation'])
    for label, (lo, hi) in intervals.items():
        phase = {}
        active = lambda x: lo <= int(x['measurement_time_ns']) <= hi
        for kind in ('fast_livo2', 'px4'):
            paired = pair([x for x in streams[kind] if active(x)], streams['gazebo'])
            a, b = [x[0] for x in paired], [x[1] for x in paired]
            delta = positions(a) @ transforms[kind].T + bias[kind] - positions(b)
            phase[kind + '_truth_xy_error_m'] = describe(np.linalg.norm(delta[:, :2], axis=1))
            phase[kind + '_truth_z_error_m'] = describe(np.abs(delta[:, 2]))
            phase[kind + '_truth_pair_dt_ms'] = describe(
                [abs(int(x['measurement_time_ns']) - int(y['measurement_time_ns'])) / 1e6
                 for x, y in paired])
        paired = pair([x for x in streams['fast_livo2'] if active(x)], streams['px4'])
        livo, px4 = positions([a for a, b in paired]), positions([b for a, b in paired])
        xy_error = np.linalg.norm(livo[:, :2] @ matrix.T + translation - px4[:, :2], axis=1)
        phase['bridge_paired_xy_disagreement_m'] = describe(xy_error)
        phase['bridge_paired_xy_samples_over_030m'] = int(np.count_nonzero(xy_error > .30))
        stage = [x for x in timing if lo <= int(x['measurement_ns']) <= hi]
        vio = [x for x in stage if x['stage'] == 'VIO']
        if vio:
            phase['vio_accepted_patches'] = describe([float(x['vio_accepted_patches']) for x in vio])
            phase['vio_zero_patch_frames'] = sum(int(x['vio_accepted_patches']) == 0 for x in vio)
            phase['vio_rejected_frames'] = sum(int(x['vio_update_rejected']) for x in vio)
        lio = [x for x in stage if x['stage'] == 'LIO' and x.get('lio_correspondences')]
        if lio:
            for column in ('lio_correspondences', 'lio_near_correspondences_045m',
                           'lio_normal_x_fraction', 'lio_normal_y_fraction', 'lio_normal_z_fraction',
                           'lio_normal_min_eigen_fraction', 'lio_mean_abs_residual_m'):
                phase[column] = describe([float(x[column]) for x in lio])
            phase['lio_normal_min_eigen_fraction_minimum'] = min(float(x['lio_normal_min_eigen_fraction']) for x in lio)
        report['phases'][label] = phase
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    directory = parser.parse_args().run_dir
    report = audit(directory)
    (directory / 'reliability_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
