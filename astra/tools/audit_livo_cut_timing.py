"""Offline snapshot timing evidence; does not change timestamps or flight gates.

Join core cuts to the adapter's selected image by measurement stamp, not by
nearest LiDAR stamp. An earlier cut can consume a previously buffered cloud;
image-before-selected-LiDAR alone is therefore not proof of an empty cut.
"""
import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import yaml


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def image_shift_ns(config):
    params = config['/**']['ros__parameters']
    shutter = params.get('rolling_shutter', {})
    if shutter.get('td_estimate_enable', False):
        raise ValueError('Online image offset estimation requires per-cut offset logs')
    offsets = params.get('time_offset', {})
    shift = offsets.get('img_time_offset', 0.) + offsets.get('exposure_time_init', 0.)
    if shutter.get('enable', False):
        shift += (.5 - shutter.get('anchor_factor', .5)) * shutter.get('tr_readout', 0.)
    if not math.isfinite(shift):
        raise ValueError('Nonfinite image timing configuration')
    return round(shift * 1e9)


def describe_cut(row, selection=None):
    result = {key: int(row[key]) for key in (
        'sequence', 'measurement_ns', 'odom_publish_ns', 'processing_us')}
    for key in ('sync_cut_points', 'undistorted_points', 'imu_initializing',
                'odom_publish_begin_wall_ns', 'odom_publish_end_wall_ns',
                'processing_start_wall_ns', 'processing_end_wall_ns'):
        if row.get(key, '') != '':
            result[key] = int(row[key])
    if row.get('odom_output_status'):
        result['odom_output_status'] = row['odom_output_status']
    if selection is not None:
        result.update(selected_lidar_ns=int(selection['stamp_ns']),
                      raw_image_ns=int(selection['image_ns']),
                      adapter_publish_done_wall_ns=int(selection['wall_ns']))
        result['cut_minus_selected_lidar_ns'] = (
            result['measurement_ns'] - result['selected_lidar_ns'])
    return result


def summarize_cuts(core, adapter, shift_ns, core_inputs=()):
    selected = defaultdict(list)
    for row in adapter:
        if row['event'] == 'publish_done' and int(row['image_ns']) > 0:
            selected[int(row['image_ns']) + shift_ns].append(row)
    buffered = defaultdict(list)
    for row in core_inputs:
        if row['event'] == 'lidar_buffered':
            buffered[int(row['measurement_ns'])].append(int(row['wall_ns']))
    lio = [r for r in core if r['stage'] == 'LIO']
    first_output = next((i for i, r in enumerate(lio)
                         if int(r['odom_publish_ns']) >= 0), len(lio))
    counts = Counter()
    missing = []
    joined = []
    for index, row in enumerate(lio):
        # Floating-point seconds in the core can round by a nanosecond.
        stamp = int(row['measurement_ns'])
        matches = [s for delta in (-1, 0, 1)
                   for s in selected.get(stamp + delta, [])]
        selection = matches[0] if len(matches) == 1 else None
        item = describe_cut(row, selection)
        if selection is not None and 'processing_start_wall_ns' in item:
            received = [t for t in buffered.get(int(selection['stamp_ns']), [])
                        if t <= item['processing_start_wall_ns']]
            if received:
                item['selected_lidar_buffered_wall_ns'] = max(received)
        joined.append(item)
        counts['lio_rows'] += 1
        counts['matched_adapter_image_rows'] += selection is not None
        counts['ambiguous_adapter_image_rows'] += len(matches) > 1
        no_output = int(row['odom_publish_ns']) < 0
        counts['no_output_total'] += no_output
        if index < first_output:
            counts['before_first_publication_rows'] += 1
            continue
        counts['after_first_publication_rows'] += 1
        early = selection is not None and stamp < int(selection['stamp_ns'])
        counts['early_cut_after_first_publication'] += early
        counts['no_output_after_first_publication'] += no_output
        counts['early_no_output_after_first_publication'] += early and no_output
        counts['confirmed_empty_cut_after_first_publication'] += (
            no_output and row.get('odom_output_status') == 'empty_cut')
        if no_output:
            missing.append(item)
    keys = ('lio_rows', 'matched_adapter_image_rows', 'ambiguous_adapter_image_rows',
            'no_output_total', 'before_first_publication_rows',
            'after_first_publication_rows', 'early_cut_after_first_publication',
            'no_output_after_first_publication', 'early_no_output_after_first_publication',
            'confirmed_empty_cut_after_first_publication')
    return dict(counts={key: counts[key] for key in keys},
                startup_boundary='Rows before first actual Odom publication; '
                                 'old logs do not identify every initialization stage',
                no_output_examples=missing[:8]), joined


def fault_context(events, callbacks, joined, monitor_rows=()):
    abort = next((e for e in events if e.get('event') == 'test_aborted'), None)
    if abort is None:
        return None
    when = abort['wall_ns']
    # Other nodes also subscribe to this topic. Only the supervisor enforces
    # the gate; receiving a message elsewhere does not prove it saw it.
    supervisor = [r for r in callbacks
                  if r['label'] == 'px4_calibration_flight:/fast_livo2/odom']
    before = [r for r in supervisor if r['wall_end_ns'] <= when]
    previous = max(before, key=lambda r: r['wall_end_ns']) if before else None
    result = dict(abort={k: abort[k] for k in (
        'wall_ns', 'sim_ns', 'reason', 'odom_age_sec', 'odom_receipt_age_sec')
        if k in abort})
    if previous is None:
        return dict(result, unavailable='No completed supervisor Odom callback before abort')
    result['last_supervisor_callback'] = previous
    stamp = previous['measurement_ns']
    result['no_output_cuts_since_last_supervisor_measurement'] = [
        r for r in joined if stamp < r['measurement_ns'] <= abort['sim_ns']
        and r['odom_publish_ns'] < 0]
    result['next_published_measurement'] = next((
        r for r in joined if r['measurement_ns'] > stamp
        and r['odom_publish_ns'] >= 0), None)
    monitor_next = [r for r in monitor_rows if r['kind'] == 'odom'
                    and int(r['stamp_ns']) > stamp]
    if monitor_next:
        row = min(monitor_next, key=lambda r: int(r['wall_ns']))
        result['independent_monitor_next_odom'] = {
            key: int(row[key]) for key in ('wall_ns', 'clock_ns', 'stamp_ns')}
        result['independent_monitor_next_odom_relative_to_abort_ms'] = (
            int(row['wall_ns']) - when) / 1e6
    after = [r for r in supervisor if r['wall_start_ns'] >= when
             and r['measurement_ns'] > stamp]
    following = min(after, key=lambda r: r['wall_start_ns']) if after else None
    if following:
        result['next_supervisor_callback'] = following
        result['next_supervisor_callback_after_abort_ms'] = (
            following['wall_start_ns'] - when) / 1e6
    return result


def visual_update_summary(core, tolerance=1e-9):
    """Count observed IMU-to-update changes, not proposed/rejected corrections.

    These logs omit roll/pitch and covariance; do not call this a complete
    estimator-state reconstruction or sum the increments into a truth error.
    """
    fields = ('x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'bax', 'bay', 'baz',
              'gravity_x', 'gravity_y', 'gravity_z')
    vio = [r for r in core if r['stage'] == 'VIO']
    changed = rejected = rejected_changed = with_patches = 0
    max_z = 0.
    for row in vio:
        delta = {k: float(row['update_' + k]) - float(row['imu_' + k]) for k in fields}
        delta['yaw'] = math.remainder(delta['yaw'], 2 * math.pi)
        observed = any(abs(d) > tolerance for d in delta.values())
        is_rejected = row['vio_update_rejected'] == '1'
        changed += observed
        rejected += is_rejected
        rejected_changed += is_rejected and observed
        with_patches += not is_rejected and int(row['vio_accepted_patches']) > 0
        max_z = max(max_z, abs(delta['z']))
    return dict(vio_rows=len(vio), rejected_rows=rejected,
                non_rejected_with_accepted_patches_rows=with_patches,
                observed_update_rows=changed, rejected_with_observed_update_rows=rejected_changed,
                maximum_observed_z_update_m=max_z, observed_fields=list(fields),
                numerical_tolerance=tolerance,
                scope='Entire core CSV; IMU-to-update logged columns only. '
                      'Roll/pitch and covariance are not logged; not a truth-error decomposition.')


def audit_run(directory):
    config = yaml.safe_load((directory / 'estimator_config.yaml').read_text())
    shift = image_shift_ns(config)
    core = rows(directory / 'livo_timing.csv')
    summary, joined = summarize_cuts(core,
                                     rows(directory / 'input_audit.csv'), shift,
                                     rows(directory / 'livo_timing.csv.inputs.csv'))
    result = dict(run=directory.name, effective_image_shift_ns=shift, **summary,
                  visual_update_summary=visual_update_summary(core))
    motion = directory / 'motion_events.json'
    callbacks = directory / 'motion_executor_audit.json'
    if motion.exists():
        data = json.loads(motion.read_text())
        result['flight'] = {k: data[k] for k in ('flight_complete', 'fault', 'disarmed')}
        if callbacks.exists():
            result['fault_context'] = fault_context(
                data['events'], json.loads(callbacks.read_text())['odom_callbacks'], joined,
                rows(directory / 'receipt.csv') if (directory / 'receipt.csv').exists() else ())
    run_summary = directory / 'run_summary.json'
    if run_summary.exists():
        result['run_summary'] = json.loads(run_summary.read_text())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = dict(scope='Gazebo snapshot cut diagnostic; not flight or frame certification',
                  publication_timing='ROS odom_publish_ns is sampled in publish_odometry; '
                  'only odom_publish_begin/end_wall_ns bracket the actual publish call. '
                  'processing_end_wall_ns includes later map work and is not a publish timestamp.',
                  runs=[audit_run(path) for path in args.runs])
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({r['run']: r['counts'] for r in result['runs']}, indent=2))


if __name__ == '__main__':
    main()
