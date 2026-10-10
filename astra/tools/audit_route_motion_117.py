"""Report measured motion per EGO goal without refitting the initial frame."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def audit(directory):
    result = json.loads((directory / 'motion_events.json').read_text())
    geometry = json.loads((directory / 'obstacle_execution_summary.json').read_text())
    reliability = json.loads((directory / 'reliability_audit.json').read_text())
    events = result['events']
    goals = [e for e in events if e['event'] == 'ego_goal_sent']
    ends = [e for e in events if e['event'] in
            ('ego_trajectory_sampled_to_end', 'test_aborted')]
    with (directory / 'frame_samples.csv').open() as stream:
        truth = sorted([r for r in csv.DictReader(stream) if r['source'] == 'gazebo'],
                       key=lambda r: int(r['measurement_time_ns']))
    times = np.array([int(r['measurement_time_ns']) for r in truth], dtype=np.int64)
    positions = np.array([[float(r[k]) for k in ('x_m', 'y_m', 'z_m')] for r in truth])
    targets = geometry.get('goals_gazebo_from_initial_alignment_m', [])
    legs = []
    for i, goal in enumerate(goals):
        reached = next((e for e in events if e['event'] == 'ego_waypoint_reached'
                        and e.get('goal_index') == i), None)
        final = reached or next((e for e in ends if e['sim_ns'] >= goal['sim_ns']), None)
        if final is None:
            raise ValueError('Missing completion/abort event for goal ' + str(i))
        begin, end = goal['sim_ns'], final['sim_ns']
        indices = [int(np.argmin(np.abs(times - stamp))) for stamp in (begin, end)]
        deltas = [abs(int(times[j]) - stamp) for j, stamp in zip(indices, (begin, end))]
        if max(deltas) > 20_000_000:
            raise ValueError('No measured endpoint within 20 ms for goal ' + str(i))
        mask = (times >= begin) & (times <= end)
        path = positions[mask]
        path_times = times[mask]
        unique = np.r_[True, np.diff(path_times) > 0]
        path, path_times = path[unique], path_times[unique]
        start_position, end_position = positions[indices]
        leg = dict(index=i, reached=(reached is not None or
                   final['event'] == 'ego_trajectory_sampled_to_end'),
                   duration_sec=(end-begin)/1e9,
                   endpoint_pair_dt_ms=[x/1e6 for x in deltas],
                   start_truth_enu_m=start_position.tolist(),
                   end_truth_enu_m=end_position.tolist(),
                   actual_displacement_enu_m=(end_position-start_position).tolist(),
                   actual_horizontal_path_m=float(np.linalg.norm(
                       np.diff(path[:, :2], axis=0), axis=1).sum()),
                   errors=reliability['phases'].get('leg_' + str(i), {}))
        if len(path) > 1:
            velocity = np.diff(path, axis=0) / (np.diff(path_times)/1e9)[:, None]
            leg['actual_peak_horizontal_speed_mps'] = float(np.linalg.norm(
                velocity[:, :2], axis=1).max())
            leg['actual_peak_vertical_speed_mps'] = float(np.abs(velocity[:, 2]).max())
        if i < len(targets):
            leg['requested_truth_xy_from_initial_frame_m'] = targets[i]
            leg['endpoint_truth_xy_error_m'] = float(np.linalg.norm(
                end_position[:2] - np.array(targets[i])))
        legs.append(leg)
    return dict(run=directory.name, flight_complete=result['flight_complete'],
                disarmed=result['disarmed'], fault=result['fault'], legs=legs,
                scope='Measurement-time Gazebo endpoints, <=20 ms pairing; initial frame '
                      'frozen before execution; no strict stable-hover certification')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
