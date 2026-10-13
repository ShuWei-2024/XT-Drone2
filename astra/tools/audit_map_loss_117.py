"""Audit a map-stop SITL regression using recorded commands and measured events."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

# Use the existing PX4 log parser without changing its environment.
sys.path.append(str(Path.home() / 'dev/px4/venv/lib/python3.12/site-packages'))
from pyulog import ULog


def audit(directory):
    motion = json.loads((directory / 'motion_events.json').read_text())
    injection = json.loads((directory / 'fault_injection.json').read_text())
    events = {e['event']: e for e in motion['events']}
    abort, hold, land, disarm = [events[k] for k in (
        'test_aborted', 'hold_target_frozen', 'land_requested', 'disarmed')]
    with (directory / 'motion_sent.csv').open() as stream:
        held = [r for r in csv.DictReader(stream) if r['phase'] == 'fault_hold']
    targets = np.array([[float(r[k]) for k in ('target_x', 'target_y', 'target_z')]
                        for r in held])
    hold_range = np.ptp(targets, axis=0) if len(targets) else np.full(3, np.inf)
    delay_ms = (hold['wall_ns'] - injection['command_wall_ns']) / 1e6
    ulog = ULog(str(next((directory / 'log').rglob('*.ulg'))))
    status = next(x.data for x in ulog.data_list if x.name == 'vehicle_status')
    armed_failsafe = int(np.sum((status['arming_state'] == 2) & status['failsafe'].astype(bool)))
    with (directory / 'receipt.csv').open() as stream:
        ready_stamps = [int(r['wall_ns']) for r in csv.DictReader(stream)
                        if r['kind'] == 'ready' and int(r['wall_ns']) <= hold['wall_ns']]
    baseline = json.loads((directory / 'run_summary.json').read_text())['original_baseline_preserved']
    checks = {
        'fault_after_injection': abort['wall_ns'] >= injection['command_wall_ns'],
        'hold_within_500ms': 0 <= delay_ms <= 500.,
        'map_stale_fault': motion['fault'] == 'local map not ready or heartbeat stale',
        'hold_target_constant': len(held) >= 15 and float(hold_range.max()) < 1e-6,
        'disarmed': motion['disarmed'],
        'normal_route_not_reported_complete': not motion['flight_complete'],
        'no_armed_failsafe': armed_failsafe == 0,
        'baseline_preserved': baseline,
    }
    return {
        'run': str(directory), 'passed': all(checks.values()), 'checks': checks,
        'scope': 'Owned map process stopped with SIGINT during planner-driven SITL flight; '
                 'supervisor takes over; not instant physical stop',
        'fault': motion['fault'],
        'command_to_abort_ms': (abort['wall_ns'] - injection['command_wall_ns']) / 1e6,
        'command_to_frozen_hold_ms': delay_ms,
        'last_ready_to_hold_ms': (hold['wall_ns'] - max(ready_stamps)) / 1e6,
        'hold_to_land_request_sec': (land['wall_ns'] - hold['wall_ns']) / 1e9,
        'land_request_to_disarm_sec': (disarm['wall_ns'] - land['wall_ns']) / 1e9,
        'fixed_hold_setpoint_samples': len(held), 'hold_setpoint_range_m': hold_range.tolist(),
        'truth_xy_motion_during_hold_m': float(np.linalg.norm(
            np.array(land['truth'][:2]) - np.array(hold['truth'][:2]))),
        'disarmed': motion['disarmed'], 'armed_failsafe_samples': armed_failsafe,
        'original_baseline_preserved': baseline,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
