"""Evidence checks for deliberate, single-source failures in isolated SITL."""

import os
from pathlib import Path
import select


def open_owned_child_pidfd(launcher_pid, executable):
    """Pin one exact child executable in an owned launch session, not its clock."""
    expected = Path(executable).resolve()
    if os.getpgid(launcher_pid) != launcher_pid or os.getsid(launcher_pid) != launcher_pid:
        raise ValueError('Fault target launcher must own its isolated process group/session')
    candidates = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == launcher_pid:
            continue
        pid = int(entry.name)
        try:
            if (Path(os.readlink(entry/'exe')).resolve() == expected and
                    os.getpgid(pid) == launcher_pid and os.getsid(pid) == launcher_pid):
                candidates.append(pid)
        except (OSError, ProcessLookupError):
            continue
    if len(candidates) != 1:
        raise ValueError(f'Expected one owned {expected.name} child, found {len(candidates)}')
    pid = candidates[0]
    descriptor = os.pidfd_open(pid)
    try:
        # Recheck after pinning, including the executable and launch ownership.
        if (Path(os.readlink(f'/proc/{pid}/exe')).resolve() != expected or
                os.getpgid(pid) != launcher_pid or os.getsid(pid) != launcher_pid):
            raise ValueError('Owned child identity changed before injection')
    except BaseException:
        os.close(descriptor)
        raise
    return pid, descriptor


def pidfd_exited(descriptor):
    return bool(select.select([descriptor], [], [], 0)[0])

EXPECTED_REASONS = {
    'map': ('local map not ready or heartbeat stale',
            'endpoint hold state fault: stale local map heartbeat',
            'bridge fault: stale local map heartbeat'),
    'external_vision': ('External Vision fusion unavailable or heartbeat stale',),
    'livo': ('External Vision fusion unavailable or heartbeat stale',
             'FAST-LIVO2 receipt age exceeds 250 ms',
             'FAST-LIVO2 measurement age outside 0..250 ms',
             'local map not ready or heartbeat stale',
             'bridge fault: stale FAST-LIVO2 odometry',
             'bridge fault: stale local map heartbeat'),
    'planner': ('bridge fault: stale planner heartbeat',),
    'position_relay': ('PX4 or Gazebo state older than 250 ms',
                       'PX4 local receipt age >= 0.3 s',
                       'ground-aligned height has no measurement pair within 20 ms',
                       'bridge fault: stale PX4 position'),
}


def evaluate_fault(kind, injection, driver_exit_code, stopped_exit_code, motion,
                   observed_child_exit=False):
    events = motion.get('events', [])
    abort = next((e for e in events if e.get('event')=='test_aborted'), None)
    frozen = next((e for e in events if e.get('event')=='hold_target_frozen'), None)
    # Once PX4 position is already stale, inventing a new hold target from it
    # is invalid. Require an explicit latched fault and a Land request instead.
    response = (next((e for e in events if e.get('event')=='fault_latched'), None)
                if kind=='position_relay' else frozen)
    land = next((e for e in events if e.get('event')=='land_requested'), None)
    delta = ((response['wall_ns']-injection['command_wall_ns'])/1e6
             if response and injection else None)
    abort_after = bool(abort and injection and abort['wall_ns']>=injection['command_wall_ns'])
    stopped_normally = stopped_exit_code == 0 or observed_child_exit
    reason = motion.get('fault','')
    reason_matches = any(value in reason for value in EXPECTED_REASONS[kind])
    normal_rows_after_abort = bool(abort and any(
        row['wall_ns']>abort['wall_ns'] for row in motion.get('bridge_samples', [])))
    land_after = bool(land and abort and land['wall_ns']>=abort['wall_ns'])
    checks = dict(injected=bool(injection and 'command_wall_ns' in injection), driver_rejected_task=driver_exit_code==1,
                  stopped_process_exited=stopped_normally, abort_after_injection=abort_after,
                  bounded_response=delta is not None and 0<=delta<=500,
                  expected_reason=reason_matches, landed_and_disarmed=motion.get('disarmed') is True,
                  task_not_reported_complete=not motion.get('flight_complete',False),
                  no_normal_bridge_rows_after_abort=not normal_rows_after_abort)
    if kind=='position_relay':checks['land_requested_after_abort']=land_after
    return dict(expected_fault_observed=all(checks.values()), checks=checks,
                driver_exit_code=driver_exit_code, fault=reason, stopped_process=kind,
                stopped_process_exit_code=stopped_exit_code, disarmed=motion.get('disarmed'),
                child_exit_confirmed_by_pidfd=observed_child_exit,
                response_event=None if response is None else response['event'],
                response_after_injection_ms=delta,
                hold_after_injection_ms=((frozen['wall_ns']-injection['command_wall_ns'])/1e6
                                        if frozen and injection else None))
