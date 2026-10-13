"""Summarize bounded SITL runs without refitting frames on flight data."""
import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return ({'count': len(a), 'p50': float(np.percentile(a, 50)),
             'p95': float(np.percentile(a, 95)), 'max': float(a.max())} if len(a) else {})


def summarize(directory):
    s = json.loads((directory / 'obstacle_execution_summary.json').read_text())
    audit = json.loads((directory / 'reliability_audit.json').read_text())
    flight = json.loads((directory / 'motion_events.json').read_text())
    events = {r['event']: r for r in flight['events']}
    result = {'run': directory.name, 'configuration': flight.get('test_configuration', {})}
    keys = ('flight_complete', 'disarmed', 'fault', 'actual_displacement_enu',
            'px4_tracking_error_p95_m', 'px4_tracking_error_max_m',
            'minimum_all_obstacle_clearance_m', 'truth_xy_endpoint_error_after_hold_m',
            'actual_peak_speed_mps', 'planned_peak_speed_mps', 'bridge_send_gap_max_ms',
            'armed_failsafe_samples', 'execution_estimator_resets',
            'original_parameters_unchanged', 'odom_measurement_age_p95_ms', 'real_time_factor')
    result.update({k: s.get(k) for k in keys})
    result['execution'] = audit.get('phases', {}).get('ego_execution', {})
    start = events.get('ego_control_handoff', {}).get('sim_ns', 0)
    stop = (events.get('ego_trajectory_sampled_to_end') or events.get('test_aborted') or {}).get('sim_ns', 0)
    timing = list(csv.DictReader((directory / 'livo_timing.csv').open()))
    result['processing_ms'] = {}
    for stage in ('LIO', 'VIO'):
        active = [r for r in timing if r['stage'] == stage and start <= int(r['measurement_ns']) <= stop]
        result['processing_ms'][stage] = {col: stats([float(r[col]) / 1000 for r in active if r.get(col)])
            for col in ('processing_us', 'thread_cpu_us', 'thread_scheduler_wait_us')}
    samples = [r for r in flight.get('bridge_samples', []) if r['state'] == 'TRACK_SITL']
    if samples and 'initial_truth_offset_m' in audit:
        enu_from_ned = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        target_world = np.array([r['target'] for r in samples]) @ enu_from_ned.T + np.array(audit['initial_truth_offset_m']['px4'])
        truth = np.array([r['truth'] for r in samples])
        result['truth_to_command_xy_error_m'] = stats(np.linalg.norm(target_world[:, :2] - truth[:, :2], axis=1))
    resource_file = directory / 'process_resources.csv'
    if resource_file.exists():
        rows = list(csv.DictReader(resource_file.open()))
        result['resources'] = {}
        for role in sorted({r['role'] for r in rows}):
            matching = [r for r in rows if r['role'] == role]
            # The first monitor revision also recorded ros2's launcher. Keep the
            # largest-median process, rather than mixing its RSS with the worker.
            pid = max({r['pid'] for r in matching}, key=lambda p: np.median(
                [int(r['rss_kib']) for r in matching if r['pid'] == p]))
            matching = [r for r in matching if r['pid'] == pid]
            times = np.array([int(r['wall_ns']) for r in matching]) / 1e9
            rss = np.array([int(r['rss_kib']) for r in matching]) / 1024
            late = times >= times[-1] - 30
            result['resources'][role] = dict(pid=int(pid), first_mib=float(rss[0]), last_mib=float(rss[-1]),
                max_mib=float(rss.max()), span_sec=float(times[-1] - times[0]),
                last_30sec_min_mib=float(rss[late].min()), last_30sec_max_mib=float(rss[late].max()),
                last_30sec_delta_mib=float(rss[-1] - rss[late][0]))
            for seconds in (60, 120):
                mask = times >= times[-1] - seconds
                result['resources'][role][f'last_{seconds}sec_mib'] = dict(
                    minimum=float(rss[mask].min()), maximum=float(rss[mask].max()),
                    delta=float(rss[-1] - rss[mask][0]))
    for line in reversed((directory / 'livo.log').read_text().splitlines()):
        if 'retained_images=' in line:
            result['final_map_counters'] = {k: float(v) for k, v in re.findall(
                r'(lidar_voxels|visual_voxels|visual_points|observations|retained_images|image_MiB|path_poses)=(\d+(?:\.\d+)?)', line)}
            break
    fault_file = directory / 'fault_injection.json'
    if fault_file.exists():
        fault = json.loads(fault_file.read_text())
        origin = fault['injection_wall_ns']
        result['fault_response_ms'] = {name: (events[name]['wall_ns'] - origin) / 1e6
            for name in ('test_aborted', 'fault_latched', 'hold_target_frozen', 'land_requested', 'disarmed') if name in events}
        result['fault_injection'] = fault
        result['fault_test_passed'] = (not flight['flight_complete'] and flight['disarmed'] and
            'local map' in flight['fault'] and result['fault_response_ms'].get('fault_latched', float('inf')) < 400)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directories', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reports = [summarize(d) for d in args.directories]
    args.output.write_text(json.dumps(reports, indent=2) + '\n')
    for r in reports:
        print(r['run'], 'complete=', r['flight_complete'], 'fault=', r['fault'],
              'tracking_p95=', r['px4_tracking_error_p95_m'], 'fault_response=', r.get('fault_response_ms'))
