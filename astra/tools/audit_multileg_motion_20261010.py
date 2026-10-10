"""Measure each autonomous flight leg against one frozen initial frame."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from audit_obstacle_reliability_20261010 import pair, positions
from obstacle_world_geometry_20261010 import clearances, load
from summarize_stability_20261010 import summarize, stats


def audit(directory):
    report = summarize(directory)
    result = json.loads((directory / 'motion_events.json').read_text())
    reliability = json.loads((directory / 'reliability_audit.json').read_text())
    events = result['events']
    goals = [e for e in events if e['event'] == 'ego_goal_sent']
    reached = {e['goal_index']: e for e in events if e['event'] == 'ego_waypoint_reached'}
    final = next((e for e in events if e['event'] == 'test_aborted'), None)
    rows = list(csv.DictReader((directory / 'frame_samples.csv').open()))
    truth = sorted([r for r in rows if r['source'] == 'gazebo'],
                   key=lambda r: int(r['measurement_time_ns']))
    obstacles = load(directory / 'obstacle_geometry.json')
    bias = np.array(reliability['initial_truth_offset_m']['fast_livo2'])
    report['requested_route_offsets_m'] = result.get('route_offsets_m')
    report['requested_leg_count'] = len(result.get('route_offsets_m', goals))
    report['reached_leg_count'] = len(reached)
    report['legs'] = []
    resource = list(csv.DictReader((directory / 'process_resources.csv').open()))
    for i, goal in enumerate(goals):
        end = reached.get(i) or final or events[-1]
        active = [r for r in truth if goal['sim_ns'] <= int(r['measurement_time_ns']) <= end['sim_ns']]
        xyz = positions(active)
        leg = dict(index=i, goal_livo=goal['goal'], goal_world=(np.array(goal['goal']) + bias).tolist(),
                   reached=i in reached, elapsed_sec=(end['sim_ns'] - goal['sim_ns']) / 1e9,
                   audit=reliability['phases'].get(f'leg_{i}', {}))
        if len(xyz) > 1:
            tt = np.array([int(r['measurement_time_ns']) for r in active]) / 1e9
            keep = np.r_[True, np.diff(tt) > 0]
            xyz, tt = xyz[keep], tt[keep]
            leg.update(actual_start_world=xyz[0].tolist(), actual_end_world=xyz[-1].tolist(),
                       actual_displacement=(xyz[-1] - xyz[0]).tolist(),
                       path_length_m=float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum()),
                       endpoint_truth_xy_error_m=float(np.linalg.norm(xyz[-1, :2] - np.array(leg['goal_world'][:2]))),
                       minimum_obstacle_clearance_m=float(clearances(xyz[:, :2], obstacles).min()),
                       speed_mps=stats(np.linalg.norm(np.diff(xyz, axis=0) / np.diff(tt)[:, None], axis=1)))
        active_bridge = [r for r in result.get('bridge_samples', []) if r.get('goal_index') == i]
        leg['bridge_states'] = {state: sum(r['state'] == state for r in active_bridge)
                                for state in sorted({r['state'] for r in active_bridge})}
        working = [r for r in resource if r['role'] == 'livo' and
                   goal['wall_ns'] <= int(r['wall_ns']) <= end['wall_ns']]
        if working:
            leg['rss_mib'] = dict(first=int(working[0]['rss_kib']) / 1024,
                                  last=int(working[-1]['rss_kib']) / 1024,
                                  maximum=max(int(r['rss_kib']) / 1024 for r in working))
        report['legs'].append(leg)
    report['all_legs_completed'] = bool(result['flight_complete'] and len(reached) == report['requested_leg_count'])
    report['actual_path_length_m'] = sum(r.get('path_length_m', 0) for r in report['legs'])
    report['path_length_at_return_m'] = result.get('actual_sampled_path_length_m')
    last_hold = [e for e in events if e['event'] == 'ego_flight_complete']
    if last_hold:
        target = np.array(goals[-1]['goal']) + bias
        report['final_hold_truth_xy_error_m'] = float(np.linalg.norm(np.array(last_hold[-1]['truth'][:2]) - target[:2]))
        report['final_hold_truth_position'] = last_hold[-1]['truth']
    heading = {}
    for kind in ('fast_livo2', 'px4'):
        source = sorted([r for r in rows if r['source'] == kind], key=lambda r: int(r['measurement_time_ns']))
        initial_ns = goals[0]['sim_ns']
        initial = pair([r for r in source if initial_ns - 2_000_000_000 <= int(r['measurement_time_ns']) <= initial_ns], truth)
        def delta(a, b):
            yaw = float(a['yaw_rad']) if kind == 'fast_livo2' else np.pi / 2 - float(a['yaw_rad'])
            return (yaw - float(b['yaw_rad']) + np.pi) % (2 * np.pi) - np.pi
        initial_angles = np.array([delta(a, b) for a, b in initial])
        offset = float(np.arctan2(np.sin(initial_angles).mean(), np.cos(initial_angles).mean()))
        stop = reached.get(len(goals) - 1) or final or events[-1]
        active = pair([r for r in source if initial_ns <= int(r['measurement_time_ns']) <= stop['sim_ns']], truth)
        errors = [abs((delta(a, b) - offset + np.pi) % (2 * np.pi) - np.pi) * 180 / np.pi for a, b in active]
        heading[kind] = dict(initial_bias_deg=offset * 180 / np.pi, frozen_bias_error_deg=stats(errors))
    report['heading_audit'] = heading
    (directory / 'multileg_motion_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('directories', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    reports = [audit(d) for d in args.directories]
    args.output.write_text(json.dumps(reports, indent=2) + '\n')
    for r in reports:
        print(r['run'], 'reached', r['reached_leg_count'], '/', r['requested_leg_count'],
              'path_m', r['actual_path_length_m'], 'fault', r['fault'])
