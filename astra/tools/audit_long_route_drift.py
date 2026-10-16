#!/usr/bin/env python3
"""Decompose recorded drift using the scan audit's frozen ground reference.

Offline only. Does not change any flight parameter or refit an airborne origin.
Requires audit_scan_flight.py and audit_external_vision.py outputs for the run.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from audit_height_dynamics_117 import active_isolated_simulators
from audit_obstacle_reliability_20261010 import describe, pair, positions


ENU_FROM_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])


def pose_residuals(source, truth, ground_offset, transform):
    matched = pair(source, truth)
    if not matched:
        return np.empty(0), np.empty((0, 3)), np.empty(0)
    stamp = np.array([int(a['measurement_time_ns']) for a, _ in matched], dtype=np.int64)
    residual = (positions([a for a, _ in matched]) @ transform.T
                + np.asarray(ground_offset) - positions([b for _, b in matched]))
    dt = np.array([abs(int(a['measurement_time_ns'])-int(b['measurement_time_ns']))/1e6
                   for a, b in matched])
    return stamp, residual, dt


def residual_summary(stamp, residual, dt, offered):
    if not len(stamp):
        return dict(offered_samples=offered, paired_samples=0)
    first = stamp <= stamp[0]+1_000_000_000
    last = stamp >= stamp[-1]-1_000_000_000
    return dict(offered_samples=offered, paired_samples=len(stamp),
                pairing_fraction=len(stamp)/offered,
                xy_m=describe(np.linalg.norm(residual[:, :2], axis=1)),
                z_m=describe(np.abs(residual[:, 2])),
                signed_z_first_second_m=float(np.median(residual[first, 2])),
                signed_z_last_second_m=float(np.median(residual[last, 2])),
                signed_z_change_m=float(np.median(residual[last, 2])-np.median(residual[first, 2])),
                truth_pair_dt_ms=describe(dt))


def phase_windows(events):
    windows = {}
    for i, e in enumerate(events):
        if e['event'] != 'segment_start' or e.get('name') == 'prestream':
            continue
        finish = next((x for x in events[i+1:] if x['event'] == 'segment_end'
                       and x.get('name') == e.get('name')), None)
        if finish:
            windows[e['name']] = (e['sim_ns'], finish['sim_ns'])
    goals = {}
    for e in events:
        if e['event'] == 'ego_goal_sent':
            goals.setdefault(e['goal_index'], e['sim_ns'])
    ordered = sorted(goals.items())
    begin = next((e['sim_ns'] for e in events if e['event'] == 'ego_control_handoff'), None)
    end = next(e['sim_ns'] for e in events
               if e['event'] in ('ego_trajectory_sampled_to_end', 'test_aborted'))
    if begin is None:
        if ordered:
            windows['planner_wait'] = (ordered[0][1], end)
    else:
        windows['ego_execution'] = (begin, end)
    for i, (index, stamp) in enumerate(ordered):
        windows[f'leg_{index}'] = (max(begin, stamp) if begin is not None else stamp,
                                  ordered[i+1][1] if i+1 < len(ordered) else end)
    landing = next((e['sim_ns'] for e in events if e['event'] == 'land_requested'), None)
    disarm = next((e['sim_ns'] for e in events if e['event'] == 'disarmed'), None)
    if landing is not None and disarm is not None:
        windows['landing'] = (landing, disarm)
    return windows


def audit(directory, make_plot=True):
    scan = json.loads((directory/'scan_flight_audit.json').read_text())
    motion = json.loads((directory/'motion_events.json').read_text())
    ev = json.loads((directory/'external_vision_audit.json').read_text())
    offsets = scan['ground_frozen_initial_truth_offset_m']
    if set(offsets) != {'fast_livo2', 'px4'}:
        raise ValueError('Both frozen ground references are required')
    rows = list(csv.DictReader((directory/'frame_samples.csv').open()))
    streams = {kind: sorted((r for r in rows if r['source'] == kind),
                           key=lambda r: int(r['measurement_time_ns']))
               for kind in ('gazebo', 'fast_livo2', 'px4')}
    windows = phase_windows(motion['events'])
    plot_phase = 'ego_execution' if 'ego_execution' in windows else 'planner_wait'
    series = {}
    report = dict(reference='Scan-audit ground-frozen offsets; fixed zero-yaw SITL ENU/NED axes; no airborne fit',
                  timestamp_pair_limit_ms=20, phases={},
                  complete_route_verified=scan['complete_route_verified'],
                  actual_route_legs=scan['actual_route_legs'],
                  ev_ground_frozen_agreement=ev['ground_frozen_livo_px4_agreement'],
                  ev_aid_sources=ev['aid_sources'], resets=ev['reset_events'])
    for name, (lo, hi) in windows.items():
        out = dict(start_measurement_ns=lo, end_measurement_ns=hi,
                   duration_sim_s=(hi-lo)/1e9)
        selected = {k: [r for r in streams[k] if lo <= int(r['measurement_time_ns']) <= hi]
                    for k in ('fast_livo2', 'px4', 'gazebo')}
        for kind in ('fast_livo2', 'px4'):
            values = pose_residuals(selected[kind], streams['gazebo'], offsets[kind],
                                   np.eye(3) if kind == 'fast_livo2' else ENU_FROM_NED)
            out[kind] = residual_summary(*values, len(selected[kind]))
            if name == plot_phase:
                series[kind] = values
        pp = pair(selected['fast_livo2'], streams['px4'])
        ground = motion['ground_height_alignment']['offset_m']
        out['supervisor_frozen_height_difference_m'] = describe([
            abs(float(a['z_m'])+float(b['z_m'])-ground) for a, b in pp])
        out['supervisor_height_pair_dt_ms'] = describe([
            abs(int(a['measurement_time_ns'])-int(b['measurement_time_ns']))/1e6
            for a, b in pp])
        truth = positions(selected['gazebo'])
        out['actual_horizontal_path_m'] = float(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1).sum())
        report['phases'][name] = out
    if make_plot and plot_phase in windows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        origin = windows[plot_phase][0]
        for kind, (stamp, residual, _) in series.items():
            axes[0].plot((stamp-origin)/1e9, residual[:, 2], label=kind, linewidth=1)
            axes[1].plot((stamp-origin)/1e9, np.linalg.norm(residual[:, :2], axis=1),
                         label=kind, linewidth=1)
        for name, (lo, _) in windows.items():
            if name.startswith('leg_'):
                for ax in axes:
                    ax.axvline((lo-origin)/1e9, color='gray', alpha=.25)
                axes[0].annotate(name, ((lo-origin)/1e9, .02), xycoords=('data', 'axes fraction'), fontsize=8)
        axes[0].axhline(0, color='black', linewidth=.5)
        axes[0].set(ylabel='Signed Z error (m)', title=directory.name+' — frozen ground reference')
        axes[1].set(ylabel='XY error norm (m)',
                    xlabel='Seconds after '+('planner handoff' if plot_phase == 'ego_execution' else 'goal request (no execution)'))
        for ax in axes:
            ax.grid(alpha=.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(directory/'long_route_drift.png', dpi=160)
        plt.close(fig)
    (directory/'long_route_drift_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    if active_isolated_simulators():
        parser.error('Stop simulation before offline drift analysis')
    print(json.dumps(audit(args.run_dir, not args.no_plot), indent=2))
