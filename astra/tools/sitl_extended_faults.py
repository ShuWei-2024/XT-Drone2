"""Isolated real estimator resets and complete DDS transport interruption."""
import csv
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def latest_live_sample(path, now_ns):
    """Read the line-buffered journal tail; offline frame CSV does not exist yet."""
    with path.open('rb') as stream:
        stream.seek(0, 2)
        stream.seek(max(0, stream.tell()-65536))
        lines = stream.read().splitlines()
    for line in reversed(lines):
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError):
            continue
        if 'sim_ns' in row and 'wall_ns' in row:
            if not 0 <= now_ns-row['wall_ns'] <= 250_000_000:
                raise ValueError('Live bridge clock anchor is stale')
            return row
    raise ValueError('No complete live bridge clock anchor')


def latest_live_clock_anchor(output, now_ns):
    """Endpoint hold has no bridge rows; its sent-setpoint journal stays live."""
    candidates = []
    try:
        row = latest_live_sample(output/'bridge_samples.jsonl', now_ns)
        candidates.append(dict(row, clock_anchor_source='bridge_samples.jsonl'))
    except (OSError, ValueError):
        pass
    try:
        path = output/'motion_sent.csv'
        with path.open() as stream:
            fields = next(csv.reader([stream.readline()]))
        with path.open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell()-65536))
            tail = stream.read()
        # An incomplete trailing row may have a timestamp but still be in flight.
        lines = tail.splitlines()
        if not tail.endswith(b'\n'):
            lines = lines[:-1]
        for line in reversed(lines):
            values = next(csv.reader([line.decode()]))
            if len(values) != len(fields):
                continue
            row = dict(zip(fields, values))
            try:
                wall_ns, sim_ns = int(row['wall_ns']), int(row['sim_ns'])
            except (KeyError, ValueError):
                continue
            if 0 <= now_ns-wall_ns <= 250_000_000:
                candidates.append(dict(wall_ns=wall_ns, sim_ns=sim_ns,
                                       clock_anchor_source='motion_sent.csv'))
            break
    except (OSError, ValueError, UnicodeError, csv.Error):
        pass
    if not candidates:
        raise ValueError('No fresh complete live flight clock anchor')
    return max(candidates, key=lambda row: row['wall_ns'])


def fault_anchor(log_path, waypoint=None):
    """Select an observed mission event, never elapsed time from takeoff."""
    for line in log_path.read_text().splitlines():
        if not line.startswith('{'):
            continue
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if waypoint is None and event.get('event') == 'ego_control_handoff':
            return event
        if (waypoint is not None and event.get('event') == 'ego_waypoint_reached'
                and event.get('goal_index') == waypoint):
            return event
    return None


def wait_and_inject(driver, agent, output, env, tools, kind, axis, delay, timeout,
                    waypoint=None):
    """Inject only after actual EGO handoff; never stop the simulator or sensors."""
    deadline = time.monotonic()+timeout
    handoff = None; attempted = False
    while driver.poll() is None:
        if time.monotonic() > deadline:
            raise RuntimeError('Extended fault test exceeded flight deadline')
        if handoff is None:
            handoff = fault_anchor(output/'motion.log', waypoint)
        if handoff and not attempted and time.monotonic_ns()-handoff['wall_ns'] >= delay*1e9:
            attempted = True
            try:
                if kind == 'estimator_reset':
                    if not (output/'reset_injector_ready').exists():
                        raise ValueError('Reset participant was not prepared before arming')
                    (output/'fault_trigger.json').write_text(json.dumps(dict(
                        anchor=handoff, waypoint_index=waypoint, delay_s=delay,
                        trigger_wall_ns=time.monotonic_ns()), indent=2)+'\n')
                    (output/'reset_injector_trigger').touch()
                else:
                    if agent.poll() is not None or os.getpgid(agent.pid) != agent.pid:
                        raise RuntimeError('Owned DDS agent is not running in its isolated group')
                    latest = latest_live_clock_anchor(output, time.monotonic_ns())
                    stamp = time.monotonic_ns()
                    os.killpg(agent.pid, signal.SIGINT)
                    injection = dict(kind='stop_owned_dds_agent', command_wall_ns=stamp,
                        command_sim_us=int(latest['sim_ns'])//1000,
                        sim_time_reference='Last complete live flight journal sample before signal; not exact signal time',
                        sim_anchor_wall_age_ms=(stamp-latest['wall_ns'])/1e6,
                        clock_anchor_source=latest['clock_anchor_source'],
                        agent_process_group=agent.pid, handoff=handoff,
                        scope='Whole PX4 DDS transport: all outputs and command/EV inputs; '
                              'Gazebo sensors, PX4 EKF, MAVLink GCS heartbeat and onboard logger remain alive')
                    (output/'fault_injection.json').write_text(json.dumps(injection, indent=2)+'\n')
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
                # A failed test stimulus is not a reason to tear down an armed
                # vehicle. Keep the supervised mission running; verdict fails.
                (output/'fault_injection_attempt.json').write_text(json.dumps(
                    dict(injected=False, kind=kind, error=str(exc)), indent=2)+'\n')
        time.sleep(.05)
    code = driver.wait()
    if kind == 'dds_agent' and (output/'fault_injection.json').exists():
        path = output/'fault_injection.json'
        injection = json.loads(path.read_text())
        injection['agent_exit_code'] = agent.poll()
        path.write_text(json.dumps(injection, indent=2)+'\n')
    return code


def wait_onboard_disarm(listener, output, env, timeout=120.):
    """Keep owned SITL alive for autonomous RTL after the ROS landing timeout.

    Read-only daemon queries do not restore DDS or send control commands.
    The closed ULog remains the acceptance evidence, not this observer.
    """
    deadline = time.monotonic()+timeout
    observed = False
    with (output/'onboard_recovery_observer.jsonl').open('w', buffering=1) as stream:
        while time.monotonic() < deadline:
            row = dict(wall_ns=time.monotonic_ns())
            try:
                result = subprocess.run([str(listener), 'vehicle_status', '-n', '1'],
                    env=env, cwd=output, capture_output=True, text=True, timeout=2.)
                state = re.search(r'\barming_state:\s*(\d+)', result.stdout)
                row.update(returncode=result.returncode, output=result.stdout)
                if result.returncode == 0 and state:
                    row['arming_state'] = int(state.group(1))
                    observed = row['arming_state'] == 1
            except (OSError, subprocess.TimeoutExpired) as exc:
                row['error'] = str(exc)
            stream.write(json.dumps(row)+'\n')
            if observed:
                break
            time.sleep(1.)
    report = dict(disarm_observed=observed, timeout_s=timeout,
                  scope='Read-only owned SITL daemon; no PX4 commands or ROS status fabrication')
    (output/'onboard_recovery_observer.json').write_text(json.dumps(report, indent=2)+'\n')
    return observed


def evaluate_records(kind, injection, motion, data, parameters, frame_rows, receipt_rows=()):
    """Require independent onboard evidence, not a stale ROS disarm flag."""
    import numpy as np
    events = motion.get('events', [])
    abort = next((e for e in events if e.get('event') == 'test_aborted'), None)
    latch = next((e for e in events if e.get('event') == 'fault_latched'), None)
    detected = motion.get('execution_fault_detection', {})
    wall = injection.get('command_wall_ns')
    sim = injection.get('command_sim_us')
    after = lambda t: sim is not None and np.asarray(t) >= sim
    status = data.get('vehicle_status', {})
    stamps = np.asarray(status.get('timestamp', []))
    armed = np.asarray(status.get('arming_state', []))
    nav = np.asarray(status.get('nav_state', []))
    post = after(stamps)
    disarmed = np.flatnonzero(post & (armed != 2))
    airborne = np.flatnonzero(post & (armed == 2))
    disarm_us = int(stamps[disarmed[0]]) if len(disarmed) else None
    landed = data.get('vehicle_land_detected', {})
    land_mask = after(landed.get('timestamp', [])) & np.asarray(landed.get('landed', []), bool)
    land_us = int(np.asarray(landed.get('timestamp', []))[np.flatnonzero(land_mask)[0]]) if np.any(land_mask) else None
    detection_wall = detected.get('wall_ns')
    reaction_ms = (detection_wall-wall)/1e6 if wall and detection_wall else None
    no_rows = bool(abort and not any(row['wall_ns'] > abort['wall_ns']
                                   for row in motion.get('bridge_samples', [])))
    checks = dict(injected=bool(wall and sim),
        fault_detection_after_injection=reaction_ms is not None and 0 <= reaction_ms <= 500,
        fault_latched=bool(latch and wall and latch['wall_ns'] >= wall),
        no_normal_bridge_samples_after_abort=no_rows,
        task_not_reported_complete=not motion.get('flight_complete', False),
        onboard_armed_after_injection=bool(len(airborne)),
        onboard_landed_before_disarm=land_us is not None and disarm_us is not None and land_us <= disarm_us,
        onboard_disarmed=disarm_us is not None)
    report = dict(kind=kind, checks=checks, fault=motion.get('fault', ''),
        execution_response_ms=reaction_ms,
        latch_response_ms=(latch['wall_ns']-wall)/1e6 if latch and wall else None,
        onboard_land_after_injection_s=(land_us-sim)/1e6 if land_us and sim else None,
        onboard_disarm_after_injection_s=(disarm_us-sim)/1e6 if disarm_us and sim else None,
        mission_disarm_confirmed=motion.get('disarmed', False))
    if kind == 'estimator_reset':
        index = {'xy': 0, 'z': 1, 'yaw': 2}[injection['axis']]
        key = ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')[index]
        local = data.get('vehicle_local_position', {})
        ts = np.asarray(local.get('timestamp', [])); values = np.asarray(local.get(key, []))
        changed = np.flatnonzero((ts[1:] >= sim) & (values[1:] != values[:-1]))+1 if sim else []
        changes = [dict(timestamp_us=int(ts[i]), before=int(values[i-1]), after=int(values[i])) for i in changed]
        checks.update(real_requested_reset_observed=injection.get('requested_counter_changed', False) and bool(changes),
            reset_reason='PX4 estimator reset' in motion.get('fault', ''),
            accepted_reset_command=any(a['command'] == injection['command'] and a['result'] == 0
                                      for a in injection.get('acknowledgements', [])))
        report.update(axis=injection['axis'], onboard_reset_changes=changes)
    else:
        flags = data.get('failsafe_flags', {})
        fts = np.asarray(flags.get('timestamp', []))
        loss = after(fts) & np.asarray(flags.get('offboard_control_signal_lost', []), bool)
        lost_idx = np.flatnonzero(loss)
        land_nav = np.flatnonzero(post & (armed == 2) & (nav == 18))
        rtl_nav = np.flatnonzero(post & (armed == 2) & (nav == 5))
        first_loss = int(fts[lost_idx[0]]) if len(lost_idx) else None
        first_land = int(stamps[land_nav[0]]) if len(land_nav) else None
        first_rtl = int(stamps[rtl_nav[0]]) if len(rtl_nav) else None
        # Buffered DDS samples may arrive just after the stop. Require no PX4
        # telemetry throughout the remainder of the mission after a 0.5 s drain.
        late = [r for r in frame_rows if r.get('source') == 'px4'
                and wall and int(r['receipt_monotonic_ns']) > wall+500_000_000]
        late_status = [r for r in receipt_rows if r.get('kind') in ('px4_local', 'px4_status')
                       and wall and int(r['wall_ns']) > wall+500_000_000]
        modes = data.get('offboard_control_mode', {})
        commands = data.get('vehicle_command', {})
        cutoff = sim+500_000 if sim else None
        landed_before = (np.asarray(landed.get('timestamp', [])) < sim) if sim else np.zeros(0, bool)
        was_in_air = bool(np.any(landed_before) and not np.asarray(landed['landed'])[landed_before][-1])
        checks.update(all_direct_px4_position_telemetry_stopped=not late,
            direct_px4_status_and_position_stopped=not late_status,
            owned_agent_exit_observed=injection.get('agent_exit_code') is not None,
            offboard_loss_recorded=first_loss is not None,
            onboard_land_or_return_mode_recorded=first_land is not None or first_rtl is not None,
            stimulus_applied_in_air=was_in_air,
            no_new_offboard_heartbeat_in_px4=bool(cutoff and not np.any(np.asarray(modes.get('timestamp', [])) > cutoff)),
            no_delivered_land_command_after_cut=bool(cutoff and not np.any(
                (np.asarray(commands.get('timestamp', [])) > cutoff) &
                (np.asarray(commands.get('command', [])) == 21))))
        report.update(parameters={k: parameters.get(k) for k in
            ('COM_OF_LOSS_T', 'COM_OBL_RC_ACT', 'COM_FAIL_ACT_T', 'COM_RC_IN_MODE')},
            first_offboard_loss_after_injection_s=(first_loss-sim)/1e6 if first_loss and sim else None,
            first_auto_land_after_injection_s=(first_land-sim)/1e6 if first_land and sim else None,
            first_auto_rtl_after_injection_s=(first_rtl-sim)/1e6 if first_rtl and sim else None,
            post_cut_px4_frame_samples_after_drain=len(late),
            post_cut_status_and_position_receipts_after_drain=len(late_status),
            onboard_armed_nav_states=sorted(set(int(nav[i]) for i in airborne)),
            confirmation='Onboard ULog only; ROS cannot confirm disarm over the severed link')
    report['expected_fault_observed'] = all(checks.values())
    return report


def audit_run(output, kind):
    sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
    from pyulog import ULog
    logs = sorted((output/'log').rglob('*.ulg'))
    if not logs:
        raise ValueError('Missing onboard ULog')
    u = ULog(str(logs[-1]))
    data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    injection = json.loads((output/'fault_injection.json').read_text())
    motion = json.loads((output/'motion_events.json').read_text())
    if 'bridge_samples' not in motion:
        # After a severed link ROS cannot confirm disarm, so the flight process
        # intentionally never reconstructs the full list. Load it offline only.
        motion['bridge_samples'] = [json.loads(line) for line in
            (output/'bridge_samples.jsonl').read_text().splitlines()]
    if len(motion['bridge_samples']) != motion['bridge_sample_count']:
        raise ValueError('Bridge journal count mismatch')
    rows = list(csv.DictReader((output/'frame_samples.csv').open()))
    receipt = list(csv.DictReader((output/'receipt.csv').open()))
    result = evaluate_records(kind, injection, motion, data, u.initial_parameters, rows, receipt)
    (output/'extended_fault_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    return result
